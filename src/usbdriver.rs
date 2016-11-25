//!
//!
//! # Terminology
//!
//! * A IN transation transfers data from the device to the host. From the device
//!   perspective it is also called a Tx transaction and uses Tx endpoints.
//! * A OUT transaction transfers data from the host to the device. From the device
//!   perspective it is also called a Rx transaction and uses Rx endpoints.
//!
//! So IN/OUT uses the host perspective while Tx/Rx uses the devices perspective.
//!
//! * USB-FS: The hardware part of the USB module within the chip.
//!
//! # Design
//!
//! The driver is split into a common part (that is hopefully generic enough)
//! and a special part that takes care of the device specific stuff.
//!
//! If the common part encounters commands
//! it doesn't know about, it propagates those commands back to the specific part.
//! Propagation in this case means that the specific part of the implementation
//! needs to poll/ask for work.
//!
//! This is how interrupts are handled:
//!
//! ```
//!    +---------------------------+
//!    | USB interrupt             |
//!    |    get_reference().isr() +-----+
//!    +---------------------------+    |
//!                                     |
//!      +------------------------------+
//!      |
//!      |   +------------------------------------------------------------+
//!      |   | Specific part of the usb stack: e.g Serial Port Driver     |
//!      |   +------------------------------------------------------------+
//!      |   |                                                            |
//!      |   |                                                            |
//!      |   |                      +----------------------------------+  |
//!      |   |                      | Common part of the usb stack     |  |
//!      |   |                      +----------------------------------+  |
//!      +----> * isr()             |                                  |  |
//!          |    +------------------> * isr()                         |  |
//!          |    loop {            |  * take_next_action() -> Action  |  |
//!          |      poll and handle |                                  |  |
//!          |      pending Actions |                                  |  |
//!          |    }                 |                                  |  |
//!          |                      +----------------------------------+  |
//!          |                                                            |
//!          +------------------------------------------------------------+
//! ```
//!
//! The specific driver struct must be `'static`. This is because the USB-FS will use
//! pointers that point to some data buffers that live within the common part (pool).
//! Also the interrupt routine must always find the driver struct.
//!
//!
//! # ISR safety
//!
//! An ISR may occur at any time, even when the driver handles a non-isr user request
//! at the same time. We must make sure there is no concurrent access and no data races.
//! The problem is that the user holds a `&'static mut` reference. That means rusts
//! ownership/borrowing rules do not hold. The isr routine holds a second `&'static mut`
//! reference at the same time.
//!
//! So as an overview, here is a call graph:
//!
//! ```
//!
//! UsbSerial::isr()
//!     UsbDriver::isr()
//!
//!
//!
//!
//!
//! ```
//!
//! FIXME: Replace 'part' with 'layer'?
//! FIXME #inline

use zinc::hal::k20::regs::*;
use usbmempool::{MemoryPool, UsbPacket, AllocatedUsbPacket, MemoryPoolTrait, HandlePriorityAllocation};
use usbenum::{EndpointWithDirAndBank, EndpointWithDir, Endpoint, Bank, Direction};
use usbmem::Fifos;
use usb;
use generated;
use zinc;
use core::cmp;

use core::cell::UnsafeCell;

// use usbmem;

// use usbmem::{Fifos, Ep};
// use usbmem::{ForEnqueue};
use core::mem::{drop, transmute};
use core::ptr;
use zinc::hal::cortex_m4::irq::NoInterrupts;

pub const EP0_SIZE: usize = 64;

/// The common part of the driver uses this enum
/// to communicate to the specific part of usb driver.
#[derive(Copy, Clone)]
pub enum Action {
    /// Reset condition appeared.
    /// Action: Driver should reset its state.
    Reset,

    /// Unknown setup packet on Endpoint 0.
    /// Action: Check if the setup instruction is intended for the specific part.
    ///         If this will packet will not be followed by an OUT transaction,
    ///         you probably need to send an empty packet back
    ///         via endpoint0_execute_send_action(Ep0TxAction::SendEmpty).
    HandleEp0SetupPacket(usb::SetupPacket),

    /// The associated transfers with the last SetupPacket have been finished.
    /// Action: Clear cached setuppacket if any.
    /// Why: Sometimes it is necessary to wait for the OUT transaction that can
    ///      follow a SetupPacket. It is necessary to remember the last setuppacket
    ///      but the OUT transaction must only be handled once. So this action helps
    ///      to deliver that promise.
    /// Note: This Action can be sent anytime.
    HandleEp0SetupPacketFinished,

    /// Associated received OUT transaction of the last setup packet.
    /// Action: Do what the specific drivers specification says.
    HandleEp0RxTransactionSmall([u8;8]),

    /// Device has been configured via SET_CONFIGURATION
    /// Action: Driver should init configuration dependend internals if any.
    Configure(u8),
}

/// Endpoint buffer state.
///
/// Each Endpoint and direction has two buffer slots
/// that will be used alternating. These two Banks are called Odd and Even.
#[derive(Copy, Clone, Debug)]
enum TxBankOccupation {
    BothFreeEvenFirst,
    BothFreeOddFirst,
    EvenFree,
    OddFree,
    NoneFreeEvenFirst,
    NoneFreeOddFirst,
}

/// Endpoint 0 transmit state.
///
/// Endpoint 0 is allowed to transfer data from static program memory or from
/// a little array that lives in the driver struct.
///
/// This state also keeps track if there is remaining static data to be sent.
enum Ep0Tx {
    /// No data is currently send.
    /// Or some static data is transmitted just now, but the buffer descriptors
    /// already know about everything.
    NothingOrFinishingStatic,

    /// Some static data is transmitted just now, and there is more static data
    /// to transmit.
    StaticRemaining(&'static [u8]),

    /// Data from the commonpart.ep0data.tx_buf is transmitted just now.
    /// We assume that we do never need to split the data into multiple chunks
    /// because the length will be max 8 bytes and the host will always
    /// request the whole length in one step. We panic if the host does not do so.
    CustomOwnedByBd0,

}

/// Which action to execute as a response to a SETUP packet.
///
/// FIXME: This was a result of the C port but may just better be
/// split into multiple functions. Used only as argument in
/// `endpoint0_execute_send_action`. This function is part of the interface to
/// the common part of the driver.
pub enum Ep0TxAction {
    /// Please send n bytes from UsbDriver.ep0data.tx_buf
    SendCustom { len: u8 },
    /// Please send these bytes that point to flash memory
    SendStatic(&'static [u8]),
    /// Send packet with len 0
    SendEmpty,
    /// Do Nothing
    DoNothing,
}

/// Used solely as argument for endpoint0_process_set_and_clear_feature_of_endpoint
/// to distinguish between the two actions.
enum SetOrClear {
    Set,
    Clear,
}

/// The common part of the usb stack. It manages usb enumeration and other
/// endpoint 0 transfers, holds the memory pool and the FIFOs for all
/// other endpoints.
///
/// Endpoint 0 transfers are used for querying the different descriptors,
/// setting and clearing features and setting this device's usb address.
///
/// It is intended that a more specific implementation encapsulates this
/// struct.
pub struct UsbDriver {
    /// The descriptors and the shortened driver readable endpoint configuration.
    descriptors_and_more : DescriptorsAndMore,

    /// Maximum endpoint number used in the usb config descriptor.
    /// This is calculated from bufferdescriptors.len().
    max_endpoint_addr : u8,

    /// Reference to the (512-aligned) buffer descriptor array that is
    /// also known to the USB-FS and is used to communicate buffer pointers
    /// and buffer ownership to the USB-FS.
    bufferdescriptors : &'static mut [usb::BufferDescriptor],

    /// Some state variables, bundled into an UnsafeCell.
    state : UnsafeCell<UsbDriverState>,

    /// State variables for endpoint 0
    ep0data : UnsafeCell<Endpoint0Data>,

    /// Pool of UsbPackets ready for allocation. A reference to this pool is also needed when freeing.
    pub pool: &'static MemoryPool<[UsbPacket; 32]>,

    /// A Fifo<Item=AllocatedUsbPacket> for each endpoint and direction.
    pub fifos: Fifos,

}

/// Usb configuration data
///
/// The descriptors and the shortened driver readable endpoint configuration.
pub struct DescriptorsAndMore {
    /// Reference to the device descriptor. Set on construction.
    /// FIXME: Refactor all these &'static stuff out.
    pub devicedescriptor : &'static [u8],
    /// Reference to the configuration descriptor. Set on construction.
    pub configdescriptortree : &'static [u8],
    /// Function pointer to a function that takes a string descriptor id
    /// and returns a reference to that string. Set on construction.
    pub get_str : fn(u8) -> &'static [u8],
    /// Reference to an array of Usb_endpt_endpt that will be copied 1:1 to
    /// configure the registers in USB().endpt[]. The configuration (which
    /// endpoint is rx/tx) must be the same as in configdescriptortree.
    pub endpointconfig_for_registers : &'static [Usb_endpt_endpt],
}

/// A bunch of bundled state variables that are protected by a single mutex mechanism.
/// FIXME: doc
struct UsbDriverState {
    /// Selected configuration id (by host via SET_CONFIGURATION)
    usb_configuration: u8,
    /// Reboot timer. Reboot can be triggered by the teensy upload
    /// command line util `teensy_loader_cli`.
    /// FIXME: implement
    usb_reboot_timer: u8,
    /// The currently available received data (stored in the fifo) for each endpoint.
    /// Will be decreased if packets are popped and increased if packets become available.
    /// FIXME: check if implemented correctly
    /// FIXME: need a private fifo
    usb_rx_byte_count_data: [u16; 16], // [;NUM_ENDPOINTS]
    /// The state of the `Bank` for each endpoint. (Odd/even. Which is next?)
    /// See `TxBankOccupation` for more information.
    tx_bank_occupation: [TxBankOccupation; 16],

    /// Action queue for communication with the specific part of the driver.
    /// This is actually a little FIFO.
    /// FIXME: untested. Refactor out and test.
    action : [Option<Action>; 5],
    /// Index of next (empty) slot for enqueue
    action_enqueue : u8,
    /// Index of next (empty) element for dequeue
    action_dequeue : u8,
}

/// A bunch of bundled state variables that are protected by a single mutex mechanism.
/// FIXME: doc
/// FIXME: seperate by access pattern (isr/usermode)
struct Endpoint0Data {
    /// Two buffers for the two receive banks of endpoint 0.
    rx0_buf: [u8; EP0_SIZE], // FIXME __attribute__ ((aligned (4)));
    rx1_buf: [u8; EP0_SIZE], // FIXME __attribute__ ((aligned (4)));

    /// The last received SetupPacket
    /// FIXME: Option<> instead of nulling?
    setuppacket: usb::SetupPacket,

    /// A buffer for transmitting custom data through endpoint 0.
    /// If self.tx_state == CustomOwnedByBd0 then
    ///    it signals that the buffer is currently in use
    ///    and owned by the USB-FS.
    /// Otherwise the buffer is available for modification.
    tx_buf: [u8; 8], // FIXME __attribute__ ((aligned (4)));

    /// Current Transfer state of endpoint 0, FIXME: rename
    tx_state: Ep0Tx,

    /// Which bank to use for next chunk on ep0
    next_tx_bank: Bank,

    /// Data0 or 1 for next chunk on ep0?
    next_tx_data01_state: usb::BufferDescriptor_control_data01,
}

/// Action fifo implementation
impl UsbDriverState {
    fn enqueue(&mut self, action : Action) -> Result<(), Action> {
        if self.action_enqueue == self.action_dequeue
            && self.action[self.action_enqueue as usize].is_some() {
            // full
            return Err(action);
        }
        self.action[self.action_enqueue as usize] = Some(action);
        self.action_enqueue = (self.action_enqueue + 1) % self.action.len() as u8;
        Ok(())
    }
    fn dequeue(&mut self) -> Option<Action> {
        if self.action_enqueue == self.action_dequeue
            && self.action[self.action_enqueue as usize].is_none() {
            // empty
            return None;
        }
        let r = self.action[self.action_dequeue as usize].take();
        self.action_dequeue = (self.action_dequeue + 1) % self.action.len() as u8;
        r
    }
}

impl UsbDriver {
    /// Instanciate. Should be done by lazy_static! or similar construct.
    pub fn new(pool: &'static MemoryPool<[UsbPacket; 32]>,
               bufferdescriptors : &'static mut [usb::BufferDescriptor],
               descriptors_and_more : DescriptorsAndMore) -> UsbDriver {

        // Each endpoint has 4 buffers (tx-even, tx-odd, rx-even, rx-odd).
        assert!(bufferdescriptors.len() % 4 == 0);

        // Init bufferdescriptors because they may come from uninitalized memory.
        for bd in bufferdescriptors.iter_mut() {
            bd.control.ignoring_state().zero_all();
            bd.addr.set_addr(0);
        }

        let common_part = UsbDriver {
            descriptors_and_more : descriptors_and_more,

            max_endpoint_addr : bufferdescriptors.len() as u8 / 4 - 1,
            bufferdescriptors : bufferdescriptors,

            state : UnsafeCell::new(UsbDriverState {
                usb_configuration: 0,
                usb_reboot_timer: 0,

                usb_rx_byte_count_data: [0u16; 16], // [;NUM_ENDPOINTS] TODO via associated const?
                tx_bank_occupation: [TxBankOccupation::BothFreeEvenFirst; 16],

                action : [None; 5],
                action_enqueue : 0,
                action_dequeue : 0,

            }),

            pool: pool,
            fifos: Fifos::new(),

            ep0data : UnsafeCell::new(Endpoint0Data {
                rx0_buf: [0u8; EP0_SIZE],
                rx1_buf: [0u8; EP0_SIZE],
                setuppacket: usb::SetupPacket::default(),

                tx_buf: [0u8; 8],
                tx_state: Ep0Tx::NothingOrFinishingStatic,
                next_tx_bank: Bank::Even,
                next_tx_data01_state: usb::BufferDescriptor_control_data01::Data0,
            }),
        };

        // This basically follows the flowchart in the Kinetis
        // Quick Reference User Guide, Rev. 1, 03/2012, page 141

        // Allow the USB-FS to access the flash.
        // See K20 doc, chapter 3.3.6.1 Crossbar Switch Master Assignments
        FMC().pfapr.set_m3ap(Fmc_pfapr_m3ap::ReadAndWrite);

        // Assume 48 MHz clock already running
        // SIM - enable clock
        SIM().scgc4.set_usbotg(Sim_scgc4_usbotg::ClockEnabled);

        // The following two lines were commented out in the original C implementation.
        // USB().usbtrc0.ignoring_state().clear_usbreset();
        // while USB().usbtrc0.usbreset() == true {} // or waitfor!

        // Set BufferDesciptor Table base addr.
        let buffertableaddr = &common_part.bufferdescriptors[0] as *const _ as usize;
        USB().bdtpage1.set_bdtba((buffertableaddr >> 8) as u8);
        USB().bdtpage2.set_bdtba((buffertableaddr >> 16) as u8);
        USB().bdtpage3.set_bdtba((buffertableaddr >> 24) as u8);

        // Clear all ISR flags.
        USB().istat.ignoring_state().clear_all();
        USB().errstat.ignoring_state().clear_all();
        USB().otgistat.ignoring_state().clear_all();

        // The following line was commented out in the original C implementation.
        // USB().usbtrc0.set_undocumented(true); // undocumented bit

        // Enable USB.
        USB().ctl.ignoring_state().set_usbensofen(Usb_ctl_usbensofen::EnableUSBModule);

        USB()
            .usbctrl
            .ignoring_state()
            .set_susp(Usb_usbctrl_susp::NotSuspended)
            .set_pde(Usb_usbctrl_pde::PulldownsDisabled);

        // Enable reset interrupt.
        USB().inten.ignoring_state().set_usbrsten(Usb_inten_usbrsten::InterruptEnabled);

        // Enable interrupt in NVIC.
        let IRQ_USBOTG = 73; // mcu spcific!! FIXME
        zinc::hal::cortex_m4::nvic::set_priority(IRQ_USBOTG, 112);
        zinc::hal::cortex_m4::nvic::enable_irq(IRQ_USBOTG);

        // Enable d+ pullup
        USB()
            .control
            .ignoring_state()
            .set_dppullupnonotg(Usb_control_dppullupnonotg::PullupEnabled);

        common_part
    }

    /// Returns the Data01-state that belongs to `bank`.
    /// This relationship is fixed because USB protocol starts with DATA0,
    /// and USB-FS starts with the Even bank. Then the two toogle synchronously.
    fn bank_to_data01(bank : Bank) -> usb::BufferDescriptor_control_data01 {
        match bank {
            Bank::Odd => usb::BufferDescriptor_control_data01::Data1,
            Bank::Even => usb::BufferDescriptor_control_data01::Data0,
        }
    }

    /// Open unsafe cell to endpoint 0 state.
    /// FIXME: Mutex or similar
    fn ep0data(&'static self) -> &'static mut Endpoint0Data {
        unsafe { transmute(self.ep0data.get()) }
    }

    /// Open unsafe cell to internal state struct.
    /// FIXME: Mutex or similar
    fn state(& self) -> &mut UsbDriverState {
        unsafe { transmute(self.state.get()) }
    }

    /// Returns true if this device has been successfully configured via SET_CONFIGURATION.
    pub fn is_configured(&self) -> bool {
        self.state().usb_configuration != 0
    }

    /// Return a reference to a `BufferDescriptor` for the given endpoint.
    fn get_bufferdescriptor(&self, endpoint: EndpointWithDirAndBank) -> &usb::BufferDescriptor {
        let bufferdescr_id = endpoint.as_bufferdescriptorarray_index();
        assert!(bufferdescr_id < self.bufferdescriptors.len());
        &self.bufferdescriptors[bufferdescr_id]
    }

    /// Transmits a packet over the selected endpoint.
    /// The packet will be moved to the USB-FS or appended to the fifo if the
    /// USB-FS is too busy.
    /// TThe interrupt routines will later move the buffer to the USB-FS.
    pub fn tx(&self, endpoint : Endpoint, packet : AllocatedUsbPacket) {
        let guard = NoInterrupts::new();
        let tx_bank_occupation : &mut TxBankOccupation = &mut self.state().tx_bank_occupation[endpoint.ep_index()];

        // Check where to put the packet.
        let (new_tx_bank_occupation, bank) = match *tx_bank_occupation {
            TxBankOccupation::BothFreeEvenFirst => (TxBankOccupation::OddFree, Bank::Even),
            TxBankOccupation::BothFreeOddFirst => (TxBankOccupation::EvenFree, Bank::Odd),
            TxBankOccupation::EvenFree => (TxBankOccupation::NoneFreeOddFirst, Bank::Even),
            TxBankOccupation::OddFree => (TxBankOccupation::NoneFreeEvenFirst, Bank::Odd),
            TxBankOccupation::NoneFreeEvenFirst | TxBankOccupation::NoneFreeOddFirst => {
                self.fifos.enqueue(endpoint.with_dir(Direction::Tx), packet);
                return;
            }
        };
        *tx_bank_occupation = new_tx_bank_occupation;

        let next_data01 = Self::bank_to_data01(bank);

        let packet_len = packet.len();

        let b = self.get_bufferdescriptor(endpoint.with_dir_bank(Direction::Tx, bank));

        // FIXME multiple: refactor into single call?
        b.swap_usb_packet(Some(packet));
        b.control
            .ignoring_state()
            .give_back(packet_len as usize, next_data01);
    }


    /// Will be called during isr() if a IN, OUT or SETUP
    /// transaction happened on endpoint 0.
    /// Will take apropiate actions.
    pub fn endpoint0_process_transaction(&'static self, last_transaction: Usb_stat_Get) {
        let ep0data = self.ep0data();
        let b = self.get_bufferdescriptor(last_transaction.into());

        let pid = b.control.pid_tok();
        match pid {
            0x0D => {
                // SETUP transaction from host

                // Copy buffer content
                ep0data.setuppacket = unsafe { b.interpret_buf_as_setup_packet() };

                // Buffer not needed any more.
                // Give the buffer back to the USB-FS for reuse.
                b.control
                    .ignoring_state()
                    .give_back(EP0_SIZE, usb::BufferDescriptor_control_data01::Data1);

                // The new SETUP packet means that remaining past transactions are rubbish.
                self.endpoint0_clear_all_pending_tx();

                // First IN/Tx after SETUP is always DATA1
                ep0data.next_tx_data01_state = usb::BufferDescriptor_control_data01::Data1;

                // Actually "do" the setup request (e.g. GET_DESCRIPTOR)
                self.endpoint0_process_setup_transaction().map_err(|e| {
                    info!("{}", e);
                    Self::endpoint0_stall();
                });
            }
            0x01 | 0x02 => {
                // OUT/Rx transaction received from host

                info!("PID=OUT {}, r&t {:x}",
                      pid,
                      ep0data.setuppacket.request_and_type());

                // The common part does not expect any OUT packets for its functionality,
                // so propagate packet to specific driver part.

                let data_len = b.control.bc() as usize;
                if data_len == 0 {
                    // Do nothing. Don't know when this is gonna happen.

                } else if data_len <= 8 {
                    // Small packets will be copied
                    // FIXME: We don't have packets here :D just an array within the
                    // driver struct.
                    let packet = b.swap_usb_packet(None).unwrap();
                    // Did the setuppacket told the truth?
                    assert!(data_len == ep0data.setuppacket.wLength() as usize);
                    let mut buf = [0u8 ; 8];
                    (&mut buf[0..data_len]).copy_from_slice(packet.buf());
                    // FIXME: At least this is the reason nothing bad is happening
                    b.swap_usb_packet(Some(packet));
                    self.state().enqueue(Action::HandleEp0RxTransactionSmall(buf));
                } else {
                    info!("large unhandled ep0 packets unsupported. len = {}", data_len);
                }

                // Give the buffer back.
                b.control
                    .ignoring_state()
                    .give_back(EP0_SIZE, usb::BufferDescriptor_control_data01::Data1);
            }
            0x09 => {
                // IN/Tx transaction completed to host

                self.endpoint0_send_remaining_data();

                // Deferred: SET_ADDRESS (must be handled here, see usb specification)
                if ep0data.setuppacket.request_and_type() == 0x0500 {
                    ep0data.setuppacket.clear_request();
                    USB()
                        .addr
                        .set_addr(ep0data.setuppacket.wValue() as u8)
                        .set_lsen(Usb_addr_lsen::RegularSpeed);
                }
            }
            _ => {
                info!("usb control. PID unknown");
            }
        }

        // Unfreeze the USB, now that we're ready
        // FIXME: Refactor into function
        USB().ctl.ignoring_state().set_usbensofen(Usb_ctl_usbensofen::EnableUSBModule); // clear TXSUSPENDTOKENBUSY bit
    }

    /// Transmit `buf` via endpoint 0
    /// FIXME: why pub?
    pub fn endpoint0_transmit(&'static self, buf: &'static [u8]) -> &'static [u8] {
        let chunksize = self.endpoint0_transmit_ptr(buf.as_ptr(), buf.len());
        &buf[chunksize..]
    }

    /// Helper for some unsafe borrowing stuff
    fn endpoint0_transmit_ptr(&'static self, buf: *const u8, len: usize) -> usize {
        let ep0data = self.ep0data();
        let chunksize = cmp::min(EP0_SIZE, len);
        let bd =
            self.get_bufferdescriptor(EndpointWithDirAndBank::new(0, Direction::Tx, ep0data.next_tx_bank));

        bd.addr.ignoring_state().set_addr(buf as u32);
        bd.control.ignoring_state().give_back(chunksize, ep0data.next_tx_data01_state);

        ep0data.next_tx_data01_state.toggle();
        ep0data.next_tx_bank.toggle();

        chunksize
    }

    /// The handlers of endpoint0 SETUP transactions want to send data sometimes.
    /// FIXME: single functions.
    pub fn endpoint0_execute_send_action(&'static self, action: Ep0TxAction) {
        match action {
            Ep0TxAction::SendStatic(mut data) => {
                data = self.endpoint0_transmit(data);
                if data.len() > 0 {
                    // put second chunk to second bank (odd/even)
                    data = self.endpoint0_transmit(data);
                }
                if data.len() > 0 {
                    // third chunk and following
                    self.ep0data().tx_state = Ep0Tx::StaticRemaining(data);
                } else {
                    self.ep0data().tx_state = Ep0Tx::NothingOrFinishingStatic;
                }
            }
            Ep0TxAction::SendCustom { len } => {
                // There will be no remaining data.
                let buf = self.ep0data().tx_buf.as_ptr();
                self.endpoint0_transmit_ptr(buf, len as usize);
                self.ep0data().tx_state = Ep0Tx::CustomOwnedByBd0;
            }
            Ep0TxAction::SendEmpty => {
                self.endpoint0_transmit(&[]);
                self.ep0data().tx_state = Ep0Tx::NothingOrFinishingStatic;
            }
            _ => {}
        }
    }

    /// Should be called whenever a IN transaction on Ep0 has been completed.
    /// This then copies the next chunk of data into the ep0 transmit buffer.
    fn endpoint0_send_remaining_data(&'static self) {
        // Send remaining data, if any.
        self.ep0data().tx_state = match self.ep0data().tx_state {
            // Remaining slice of data that needs to be pushed into buffers chunk by chunk.
            Ep0Tx::StaticRemaining(data) => {
                let remaining = self.endpoint0_transmit(data);
                if remaining.len() == 0 {
                    // The last chunk has been copied into buffer,
                    // but expect one or two more IN-finished interrupts.
                    Ep0Tx::NothingOrFinishingStatic
                } else {
                    Ep0Tx::StaticRemaining(remaining)
                }
            }
            // The last bit of static data has arrived at the host (may be called twice)
            Ep0Tx::NothingOrFinishingStatic => Ep0Tx::NothingOrFinishingStatic,
            // If custom data is sent, there will be no second part.
            Ep0Tx::CustomOwnedByBd0 => Ep0Tx::NothingOrFinishingStatic,
        };
    }

    /// Clears all pending transations on endpoint 0.
    pub fn endpoint0_clear_all_pending_tx(&'static self) {
        // prevent remaining static data to be sent.
        self.ep0data().tx_state = Ep0Tx::NothingOrFinishingStatic;

        self.get_bufferdescriptor(EndpointWithDirAndBank::new(0, Direction::Tx, Bank::Even))
            .control
            .ignoring_state()
            .zero_all();

        self.get_bufferdescriptor(EndpointWithDirAndBank::new(0, Direction::Tx, Bank::Odd))
            .control
            .ignoring_state()
            .zero_all();
    }

    /// Stalls endpoint 0.
    pub fn endpoint0_stall() {
        USB().endpt[0]
            .endpt
            .ignoring_state()
            .set_epstall(Usb_endpt_endpt_epstall::Stalled)
            .set_eprxen(Usb_endpt_endpt_eprxen::RxEnabled)
            .set_eptxen(Usb_endpt_endpt_eptxen::TxEnabled)
            .set_ephshk(Usb_endpt_endpt_ephshk::Handshake);
    }

    /// Handles a SETUP transaction on endpoint 0, for example
    /// SET_ADDRESS or GET_DESCRIPTOR.
    pub fn endpoint0_process_setup_transaction(&'static self) -> Result<(), &'static str> {

        let action = match self.ep0data().setuppacket.request_and_type() {
            0x0500 => {
                // SET_ADDRESS (do nothing here, defer until IN is completed)
                // But make a IN transaction with a len of 0
                Ep0TxAction::SendEmpty
            }
            0x0900 => {
                // SET_CONFIGURATION
                try!(self.endpoint0_process_set_configuration())
            }
            0x0880 => {
                // GET_CONFIGURATION
                self.ep0data().tx_buf[0] = self.state().usb_configuration;
                Ep0TxAction::SendCustom { len: 1 }
            }
            0x0080 => {
                // GET_STATUS (device)
                self.ep0data().tx_buf[0] = 0;
                self.ep0data().tx_buf[1] = 0;
                Ep0TxAction::SendCustom { len: 2 }
            }
            0x0082 => {
                // GET_STATUS (endpoint)
                try!(self.endpoint0_process_get_status_of_endpoint())
            }
            0x0102 => {
                // CLEAR_FEATURE (endpoint)
                try!(self.endpoint0_process_set_and_clear_feature_of_endpoint(SetOrClear::Clear))
            }
            0x0302 => {
                // SET_FEATURE (endpoint)
                try!(self.endpoint0_process_set_and_clear_feature_of_endpoint(SetOrClear::Set))
            }
            0x0680 | 0x0681 => {
                // GET_DESCRIPTOR (0x0680 calls on device level, 0x0681 on interface level)
                // Normally 0x0681 should never happen, but I guess there is a misbehaving host somewhere in the wild
                try!(self.endpoint0_process_get_descriptor())
            }
            _ => {
                // Propagate the packet to the specific part of the usb stack using the
                // action polling mechanism.
                self.state().enqueue(Action::HandleEp0SetupPacket(self.ep0data().setuppacket));
                return Ok(());
            }
        };

        // The received SETUP transaction was intended for the common part of the usb stack.
        // So prevent the specific part to handle a possible OUT packet or similar stuff
        // following this new SETUP transaction.
        self.state().enqueue(Action::HandleEp0SetupPacketFinished);

        // FIXME: I guess we should use something like this, but this will need
        // duplication with the intended refactoring
        // CC  if (datalen > setup.wLength) datalen = setup.wLength;

        self.endpoint0_execute_send_action(action);

        Ok(())
    }

    /// Handle SET_CONFIGURATION.
    /// Initialize endpoints and clean up. This is a new beginning :)
    fn endpoint0_process_set_configuration(&'static self) -> Result<Ep0TxAction, &'static str> {

        self.state().usb_configuration = self.ep0data().setuppacket.wValue() as u8;

        // Tell the specific part that we got a new configuration.
        self.state().enqueue(Action::Configure(self.state().usb_configuration));

        // Free packets currently owned by the usb controller.
        for bd in self.bufferdescriptors.iter().skip(4) {
            // WHY free the packets owned by the controller?
            if bd.control.own() == usb::BufferDescriptor_control_own::Controller {
                match bd.swap_usb_packet(None) {
                    Some(p) => {
                        p.recycle(&self.pool, &self);
                    }
                    None => {}
                }
            }
        }

        // Clear pending packets in all fifos.
        self.fifos.clear_all(&self.pool, &self);

        // Reset rx count
        for i in self.state().usb_rx_byte_count_data.iter_mut() {
            *i = 0;
        }

        // Reset tx state.
        // WHY: Why don't we reset to zero?
        //      Odd/Even is coupled with DATA0/1.
        //      So: is DATA0/1 stuff resetted on configuration?
        for i in self.state().tx_bank_occupation.iter_mut() {
            match *i {
                TxBankOccupation::EvenFree |
                TxBankOccupation::NoneFreeEvenFirst => {
                    *i = TxBankOccupation::BothFreeEvenFirst;
                }
                TxBankOccupation::OddFree |
                TxBankOccupation::BothFreeOddFirst => {
                    *i = TxBankOccupation::BothFreeOddFirst;
                }
                _ => {}
            }
        }

        // Previously requested priority allocations will not be fetched anymore because
        // the next `for` loop will reset packet-pending endpoints (or select other
        // endpoints for transfers). So clear them.
        self.pool.clear_priority_allocation_requests();

        // Configure Endpoints 1-15. Set type and set buffers to Rx channels.
        for (i, (epconf, endptreg)) in self.descriptors_and_more.endpointconfig_for_registers
            .iter()
            .zip(USB().endpt.iter())
            .enumerate()
            .skip(1)
            .take(self.max_endpoint_addr as usize) {

            // Use the `endpointconfig_for_registers` preconfigured data to define
            // which endpoint is Tx/Rx/Both.
            // This was easier that parsing the descriptor.
            endptreg.endpt.ignoring_state().set_raw(epconf.raw());

            // Rx EndPoints need packets in both banks (even and odd) that are ready for
            // receiving data.
            if endptreg.endpt.eprxen() == Usb_endpt_endpt_eprxen::RxEnabled {
                self.setup_rx_endpoint_or_do_priority_allocation(
                    EndpointWithDirAndBank::new(i as u8, Direction::Rx, Bank::Odd));
                self.setup_rx_endpoint_or_do_priority_allocation(
                    EndpointWithDirAndBank::new(i as u8, Direction::Rx, Bank::Even));
            }

            // Tx endpoints only own packets if they send data.
            // Make sure the Processor (and not the USB-FS) owns the buffer.
            self.get_bufferdescriptor(EndpointWithDirAndBank::new(i as u8, Direction::Tx, Bank::Even))
                .control
                .ignoring_state()
                .zero_all();
            self.get_bufferdescriptor(EndpointWithDirAndBank::new(i as u8, Direction::Tx, Bank::Odd))
                .control
                .ignoring_state()
                .zero_all();

        } // end for

        // Empty IN transaction on required as ACQ.
        Ok((Ep0TxAction::SendEmpty))
    }

    /// Allocates new packet for `ep`.
    /// Assumes `ep` to be a Rx endpoint and assumes that `ep` does not contain a packet.
    fn setup_rx_endpoint_or_do_priority_allocation(&'static self, ep : EndpointWithDirAndBank) {
        let bd = self.get_bufferdescriptor(ep);
        match self.pool.allocate() {
            Some(p) => {
                // Move packet ownership to buffer descriptor.
                bd.swap_usb_packet(Some(p));
                bd.control
                    .ignoring_state()
                    .give_back(UsbPacket::capacity(),
                               usb::BufferDescriptor_control_data01::Data0);
            }
            None => {
                // No more free packets. Reserve the next free'd packet.
                // Because of this functionality it is necessary to provide a reference to
                // this struct during recycling of packets.
                bd.control.ignoring_state().zero_all();
                self.pool.allocate_priority();
            }
        }
    }

    /// Handle GET_STATUS of endpoint.
    /// Is the endpoint[wIndex] halted?
    fn endpoint0_process_get_status_of_endpoint(&'static self) -> Result<Ep0TxAction, &'static str> {
        let ep0data = self.ep0data();
        /// FIXME: is ep0data() already opened in the parent functions? then propagate via reference
        let endpoint_nr = ep0data.setuppacket.wIndex() as usize;
        if endpoint_nr > 16 {
            return Err("illegal endpt");
        }
        ep0data.tx_buf[0] = match USB().endpt[endpoint_nr].endpt.epstall() {
            Usb_endpt_endpt_epstall::Stalled => 1,
            Usb_endpt_endpt_epstall::NotStalled => 0,
        };
        ep0data.tx_buf[1] = 0;
        Ok(Ep0TxAction::SendCustom { len: 2 })
    }

    /// Handle SET/CLEAR_FEATURE of endpoint
    /// FIXME: refactor (or not, i thought set/clear were something different)
    /// There is only one feature available for any endpoint: ENDPOINT_HALT <=> wValue=0
    fn endpoint0_process_set_and_clear_feature_of_endpoint(&'static self,
                                                           action: SetOrClear)
                                                           -> Result<Ep0TxAction, &'static str> {
        let endpoint_nr = self.ep0data().setuppacket.wIndex() as usize;
        if endpoint_nr > 16 || self.ep0data().setuppacket.wValue() != 0 {
            return Err("illegal endpt or unknown feature");
        }

        let new_state = match action {
            SetOrClear::Set => Usb_endpt_endpt_epstall::Stalled,
            SetOrClear::Clear => Usb_endpt_endpt_epstall::NotStalled,
        };

        USB().endpt[endpoint_nr].endpt.set_epstall(new_state);

        // CC    // TODO: do we need to clear the data toggle here?
        Ok((Ep0TxAction::SendEmpty))
    }

    /// Handle GET_DESCRIPTOR
    /// There are different kinds of descriptors. For string descriptors some extern
    /// function (pointer given during creation of self) is used to get the slice.
    fn endpoint0_process_get_descriptor(&'static self) -> Result<Ep0TxAction, &'static str> {
        let descr_type = (self.ep0data().setuppacket.wValue() >> 8) as u8;
        let mut data: &'static [u8] = match descr_type {
            1 => self.descriptors_and_more.devicedescriptor,
            2 => self.descriptors_and_more.configdescriptortree,
            3 => {
                // String Descriptors.
                let index = self.ep0data().setuppacket.wValue() as u8;
                (self.descriptors_and_more.get_str)(index)
            }
            _ => {
                // not found
                return Err("descriptor not found");
            }
        };
        let requested_len = self.ep0data().setuppacket.wLength() as usize;
        if data.len() > requested_len {
            data = &data[0..requested_len];
        }
        Ok(Ep0TxAction::SendStatic(data))
    }

    /// Handle reset condition on usb line.
    pub fn usb_reset(&'static self) {

        // Initialize/reset buffer descriptor bank toggle bits.
        USB().ctl.ignoring_state().set_oddrst(true);
        self.ep0data().next_tx_bank = Bank::Even;

        // Set up endpoint 0 buffers to receive SETUP and OUT packets.
        self.get_bufferdescriptor(EndpointWithDirAndBank::new(0, Direction::Rx, Bank::Even))
            .addr
            .set_addr(&self.ep0data().rx0_buf as *const u8 as u32);
        self.get_bufferdescriptor(EndpointWithDirAndBank::new(0, Direction::Rx, Bank::Even))
            .control
            .ignoring_state()
            .give_back(EP0_SIZE, usb::BufferDescriptor_control_data01::Data0);

        self.get_bufferdescriptor(EndpointWithDirAndBank::new(0, Direction::Rx, Bank::Odd))
            .addr
            .set_addr(&self.ep0data().rx1_buf as *const u8 as u32);
        self.get_bufferdescriptor(EndpointWithDirAndBank::new(0, Direction::Rx, Bank::Odd))
            .control
            .ignoring_state()
            .give_back(EP0_SIZE, usb::BufferDescriptor_control_data01::Data0);

        // No tx buffer until there is data to send.
        self.get_bufferdescriptor(EndpointWithDirAndBank::new(0, Direction::Tx, Bank::Even))
            .control
            .ignoring_state()
            .zero_all();
        self.get_bufferdescriptor(EndpointWithDirAndBank::new(0, Direction::Tx, Bank::Odd))
            .control
            .ignoring_state()
            .zero_all();


        // Activate endpoint 0.
        // FIXME: endpoint0_stall() stalls ep0. unstall it??
        USB().endpt[0]
            .endpt
            .set_eprxen(Usb_endpt_endpt_eprxen::RxEnabled)
            .set_eptxen(Usb_endpt_endpt_eptxen::TxEnabled)
            .set_ephshk(Usb_endpt_endpt_ephshk::Handshake);

        // Clear all ending interrupts.
        USB().errstat.ignoring_state().clear_all();
        USB().istat.ignoring_state().clear_all();

        // Set the address to zero during enumeration.
        USB().addr.set_addr(0).set_lsen(Usb_addr_lsen::RegularSpeed);

        // Enable other interrupts.
        USB().erren.ignoring_state().enable_all();
        USB().inten.ignoring_state().set_tokdneen(Usb_inten_tokdneen::InterruptEnabled)
                                    .set_softoken(Usb_inten_softoken::InterruptEnabled)
                                    .set_stallen(Usb_inten_stallen::InterruptEnabled)
                                    .set_erroren(Usb_inten_erroren::InterruptEnabled)
                                    .set_usbrsten(Usb_inten_usbrsten::InterruptEnabled)
                                    .set_sleepen(Usb_inten_sleepen::InterruptEnabled)
                                    .set_resumeen(Usb_inten_resumeen::InterruptDisabled) // disabled
                                    .set_attachen(Usb_inten_attachen::InterruptDisabled); // disabled

        // USB Enable Setting
        USB().ctl.ignoring_state().set_usbensofen(Usb_ctl_usbensofen::EnableUSBModule);
    }

    /// Pop next action.
    /// To be called from the specific part of the usb stack right after propagating isr().
    /// FIXME: rename
    pub fn action(&'static self) -> Option<Action> {
        self.state().dequeue()
    }

    /// The interrupt service routine handler
    /// To be called from the specific part of the usb stack.
    pub fn isr(&'static self) {

        // (cant create fake value until loop, so read twice)
        // FIXME: loop break return?
        let mut status = USB().istat.get();

        loop {
            status = USB().istat.get();

            // SOFTOK: Start-of-frame token. Will occour every milli second as long as usb is active.
            if status.softok() {
                // FIXME reboot timer
                //     if (usb_configuration) { // *usb_configuration is the value set by host with SET_CONFIGURATION
                //       t = usb_reboot_timer; // *I guess for restart triggered by firmware upload utility
                //       if (t) {
                //         usb_reboot_timer = --t;
                //         if (!t) _reboot_Teensyduino_();
                //       }
                // #ifdef CDC_DATA_INTERFACE
                //       t = usb_cdc_transmit_flush_timer; // *todo
                //       if (t) {
                //         usb_cdc_transmit_flush_timer = --t;
                //         if (t == 0) usb_serial_flush_callback();
                //       }
                // #endif
                //     }
                // Not again.
                USB().istat.ignoring_state().clear_softok();
            }


            // TOKDNE: This bit is set when the current token being processed has completed. The processor must immediately
            // read the STATUS (STAT) register to determine the EndPoint and BD used for this token. Clearing this bit
            // (by writing a one) causes STAT to be cleared or the STAT holding register to be loaded into the STAT
            // register.
            if status.tokdne() {
                /// FIXME: Do the .into() here? There isn't any info lost
                let last_transaction = USB().stat.get();

                // Split handling endpoint 0 and handling all other endpoints.
                if last_transaction.endp() == 0 {
                    self.endpoint0_process_transaction(last_transaction);
                } else {
                    self.process_transaction(last_transaction);
                }
                // Not again.
                USB().istat.ignoring_state().clear_tokdne();
            } else {
                break;
            }
        } // end loop

        // Reset condition
        if status.usbrst() {
            info!("r");
            self.usb_reset();
            return;
        }

        // Stall interrupt.
        if status.stall() {
            info!("isr stall");
            // FIXME: correct handeling?
            USB().endpt[0]
                .endpt
                .set_eprxen(Usb_endpt_endpt_eprxen::RxEnabled)
                .set_eptxen(Usb_endpt_endpt_eptxen::TxEnabled)
                .set_ephshk(Usb_endpt_endpt_ephshk::Handshake);
            USB().istat.ignoring_state().clear_stall(); // Not again.
        }

        // Error interrupt.
        if status.error() {
            let err = USB().errstat.get().raw();
            USB().errstat.ignoring_state().clear_raw(err); // Not again.
            info!("isr err {:b}", err);
            USB().istat.ignoring_state().clear_error(); // Not again.
        }

        // Sleep interrupt.
        if status.sleep() {
            info!("isr sleep");
            USB().istat.ignoring_state().clear_sleep(); // Not again.
        }
    }

    /// Handle IN or OUT transaction on endoints 1-15
    fn process_transaction(&'static self, last_transaction : Usb_stat_Get) {

        let ep0data = self.ep0data();
        let endpoint : EndpointWithDirAndBank = last_transaction.into();

        let next_data01 = Self::bank_to_data01(endpoint.bank());

        let b = self.get_bufferdescriptor(endpoint);

        // TX or RX tansaction?
        if last_transaction.tx().eq(&Usb_stat_tx::Tx) {
            // => Tx/IN has finished.

            let tx_bank_occupation : &mut TxBankOccupation = &mut self.state().tx_bank_occupation[endpoint.ep_index()];

            // Free old transmitted AllocatedUsbPacket.
            b.swap_usb_packet(None).unwrap().recycle(&self.pool, &self);

            // Check corresponding fifo for more data to send
            let next = self.fifos.dequeue(endpoint.into());

            if let Some(packet) = next {

                // Move Packet into BufferDescriptor and propagate tx_bank_occupation ()
                let packet_len = packet.len();
                b.swap_usb_packet(Some(packet));
                *tx_bank_occupation = match *tx_bank_occupation {
                    TxBankOccupation::BothFreeEvenFirst => TxBankOccupation::OddFree,
                    TxBankOccupation::BothFreeOddFirst =>  TxBankOccupation::EvenFree,
                    TxBankOccupation::EvenFree => TxBankOccupation::NoneFreeOddFirst,
                    TxBankOccupation::OddFree => TxBankOccupation::NoneFreeEvenFirst,
                    TxBankOccupation::NoneFreeEvenFirst => TxBankOccupation::NoneFreeEvenFirst,
                    TxBankOccupation::NoneFreeOddFirst => TxBankOccupation::NoneFreeOddFirst,
                };

                // Give the buffer back to the USB-FS.
                b.control
                    .ignoring_state()
                    .give_back(packet_len as usize, next_data01);

            } else {
                // No packet in queue.
                *tx_bank_occupation = match *tx_bank_occupation {
                    TxBankOccupation::BothFreeEvenFirst => TxBankOccupation::BothFreeEvenFirst,
                    TxBankOccupation::BothFreeOddFirst => TxBankOccupation::BothFreeOddFirst,
                    TxBankOccupation::EvenFree => TxBankOccupation::BothFreeEvenFirst,
                    TxBankOccupation::OddFree => TxBankOccupation::BothFreeOddFirst,
                    TxBankOccupation::NoneFreeEvenFirst | TxBankOccupation::NoneFreeOddFirst => {
                        match endpoint.bank() {
                            Bank::Odd => TxBankOccupation::OddFree,
                            Bank::Even => TxBankOccupation::EvenFree,
                        }
                    },
                };
            }
        } else {
            // => Rx/OUT finished.

            // Move received packet from BufferDesciptor to fifo
            let mut packet = b.swap_usb_packet(None).unwrap();

            if packet.len() > 0 {
                packet.set_index(0);
                self.state().usb_rx_byte_count_data[endpoint.ep_index()] += packet.len();
                self.fifos.enqueue(endpoint.into(), packet);
                // Comment from C version:
                // TODO: implement a per-endpoint maximum # of allocated
                // packets, so a flood of incoming data on 1 endpoint
                // doesn't starve the others if the user isn't reading
                // it regularly

                // Move a fresh packet into the BufferDesciptor so that more data
                // can be received.
                if let Some(next) = self.pool.allocate() {
                    b.swap_usb_packet(Some(next));

                    // Give the buffer back to the USB-FS/
                    b.control
                        .ignoring_state()
                        .give_back(UsbPacket::capacity(), next_data01);

                } else {
                    // Pool is empty. Register for next free packet.
                    self.pool.allocate_priority();
                    b.control.ignoring_state().zero_all();
                }
            } else {
                // packet.len() == 0: No data in packet.
                // Do not move packet to fifo, instead just reuse it for next transfer
                b.swap_usb_packet(Some(packet));
                b.control
                    .ignoring_state()
                    .give_back(UsbPacket::capacity(), next_data01);

            }
        }
    }

}

impl<'a> HandlePriorityAllocation for &'a UsbDriver {
    /// Just now there are some new free packets
    /// Use them to fill all Rx endpoints that are active but do not own a packet.
    fn handle_priority_allocation(&self, packet : AllocatedUsbPacket) -> Option<AllocatedUsbPacket> {
        let guard = NoInterrupts::new();
        for (i, epconf) in self.descriptors_and_more.endpointconfig_for_registers
                               .iter().enumerate().skip(1) {

            if epconf.eprxen() == Usb_endpt_endpt_eprxen::RxEnabled {

                let bd_even = self.get_bufferdescriptor(
                    EndpointWithDirAndBank::new(i as u8, Direction::Rx, Bank::Even));
                if bd_even.control.is_zero() {
                    bd_even.swap_usb_packet(Some(packet)).unwrap();
                    // Give the buffer back to the USB-FS.
                    bd_even.control
                        .ignoring_state()
                        .give_back(UsbPacket::capacity(),
                                   usb::BufferDescriptor_control_data01::Data0);
                    return None;
                }

                let bd_odd = self.get_bufferdescriptor(
                    EndpointWithDirAndBank::new(i as u8, Direction::Rx, Bank::Odd));
                if bd_odd.control.is_zero() {
                    bd_odd.swap_usb_packet(Some(packet)).unwrap();
                    bd_odd.control
                        .ignoring_state()
                        .give_back(UsbPacket::capacity(),
                                   usb::BufferDescriptor_control_data01::Data1);
                    return None;
                }
            }
        }
        // We should never reach this point.  If we get here, it means
        // that a priority packet allocation was requested, but no memory
        // was actually needed.

        // Return packet. The pool will free it.
        Some(packet)
    }
}


enum FlashCommands {
    ReadFromIFR { record_index: u8 },
}

enum FlashCommandReturns {
    FourBytes { data: [u8; 4] },
}

impl FlashCommands {
    fn execute_flash_command(command: FlashCommands) -> FlashCommandReturns {
        // pp 573, section 28.3.4 Register Descriptions
        // pp 591, section 28.4.9 Flash Command Operatrions
        // pp 613, section 28.4.11.10 Read Once Command
        // 12 bytes of memory for input and output

        let _guard = NoInterrupts::new();
        while Ftfl().stat.ccif().eq(&Ftfl_stat_ccif::InProgress) {}

        Ftfl().stat.ignoring_state().clear_rdcolerr().clear_accerr().clear_fpviol();
        let ret = match &command {
            &FlashCommands::ReadFromIFR { record_index: record_index } => {
                Ftfl().fccob0.ignoring_state().set_ccob0(0x41);
                Ftfl().fccob1.ignoring_state().set_ccob1(record_index);
            }
        };

        Ftfl().stat.ignoring_state().clear_ccif();
        while Ftfl().stat.ccif().eq(&Ftfl_stat_ccif::InProgress) {}

        match &command {
            &FlashCommands::ReadFromIFR { record_index: _ } => {
                let mut data = [0u8; 4];
                data[0] = Ftfl().fccob4.ccob4();
                data[1] = Ftfl().fccob5.ccob5();
                data[2] = Ftfl().fccob6.ccob6();
                data[3] = Ftfl().fccob7.ccob7();
                FlashCommandReturns::FourBytes { data: data }
            }
        }
    }
}








