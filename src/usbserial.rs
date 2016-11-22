
use usb::SetupPacket;
use usbdriver::{ UsbDriver, Action, Ep0TxAction};
use usbmempool::{AllocatedUsbPacket, HandlePriorityAllocation, MemoryPoolTrait};
use zinc::drivers::chario::CharIO;
use volatile_cell::VolatileCell;
use usbenum::{EndpointWithDir, Direction};

use core::cmp::min;
use core::cell::UnsafeCell;
use core::mem::{transmute, transmute_copy};
use core_io::Write as IoWrite;
use core_io::Read as IoRead;
use core_io::Error as IoError;
use core_io::ErrorKind as IoErrorKind;
use core_io::Result as IoResult;
use core::fmt::Write as FmtWrite;
use core::fmt::Result as FmtResult;
use core::fmt::Error as FmtError;

const CDC_STATUS_INTERFACE : usize = 0;
const CDC_DATA_INTERFACE : usize = 1;
const CDC_ACM_ENDPOINT : usize = 2;
const CDC_RX_ENDPOINT : u8 = 3;
const CDC_TX_ENDPOINT : u8 = 4;
const CDC_ACM_SIZE : usize = 16;
const CDC_RX_SIZE : usize = 64;
const CDC_TX_SIZE : usize = 64;

// Maximum number of transmit packets to queue so we don't starve other endpoints for memory
const TX_PACKET_LIMIT : usize = 8;
// When the PC isn't listening, how long do we wait before discarding data?
const TX_TIMEOUT_MSEC : usize = 70;
const TX_TIMEOUT : usize = (TX_TIMEOUT_MSEC * 512); // TODO F_CPU dependent

pub struct UsbSerial {
    base : UsbDriver,

    cdc_state : UnsafeCell<CdcState>,

    rx_packet : UnsafeCell<Option<AllocatedUsbPacket>>,
    tx_packet : UnsafeCell<Option<AllocatedUsbPacket>>,

    count_consecutive_unsuccessfull_alloc_attempts : usize,

    setuppacket: UnsafeCell<Option<SetupPacket>>,
}

struct CdcState {
    line_coding : [u32; 2],
    line_rtsdtr_millis :  VolatileCell<u32>,
    line_rtsdtr : VolatileCell<u8>,
    transmit_flush_timer : VolatileCell<u8>,
}

impl UsbSerial {
    pub fn new(base : UsbDriver) -> UsbSerial {
        UsbSerial{
            base : base,

            cdc_state : UnsafeCell::new(CdcState {
                line_coding : [0, 0],
                line_rtsdtr_millis : VolatileCell::new(0),
                line_rtsdtr : VolatileCell::new(0),
                transmit_flush_timer : VolatileCell::new(0),
            }),

            tx_packet : UnsafeCell::new(None),
            rx_packet : UnsafeCell::new(None),

            count_consecutive_unsuccessfull_alloc_attempts : 0,

            setuppacket : UnsafeCell::new(None),
        }
    }

    // TODO
    // CC operator bool() { return usb_configuration &&
    // CC     (usb_cdc_line_rtsdtr & (USB_SERIAL_DTR | USB_SERIAL_RTS)) &&
    // CC     ((uint32_t)(systick_millis_count - usb_cdc_line_rtsdtr_millis) >= 25);
    // CC }

    /// open unsafe cell
    fn cdc_state(&'static self) -> &'static mut CdcState {
        unsafe { transmute(self.cdc_state.get()) }
    }

    pub unsafe fn isr(&'static self) {
        self.base.isr();
        self.process();
    }

    fn process(&'static self) {
        loop {
            match self.base.action() {
                Some(Action::Reset) => { self.reset(); }
                Some(Action::Configure(_)) => { }
                Some(Action::HandleEp0RxTransactionSmall(ref buf)) => { self.handle_ep0_out(buf); }
                Some(Action::HandleEp0SetupPacket(ref setuppacket)) => { self.handle_ep0_setup(setuppacket); }
                Some(Action::HandleEp0SetupPacketFinished) => { self.handle_ep0_setup_finished(); }
                None => { break; },
            }
        }
    }

    fn reset(&'static self) {

    }

    fn handle_ep0_setup_finished(&'static self) {
        unsafe { (*self.setuppacket.get()) = None; }
    }

    fn handle_ep0_setup(&'static self, setuppacket : &SetupPacket) {

        unsafe { (*self.setuppacket.get()) = Some(*setuppacket); }

        let action = match setuppacket.request_and_type() {
            0x2221 => {
                // CDC_SET_CONTROL_LINE_STATE
                // self.cdc_state().line_rtsdtr_millis.set( systick_millis_count); // TODO with bool()
                self.cdc_state().line_rtsdtr.set(setuppacket.wValue() as u8);
                //serial_print("set control line state\n");
                Ep0TxAction::SendEmpty
            }
            0x2321 => {
                // CDC_SEND_BREAK
                Ep0TxAction::SendEmpty
            }
            0x2021 => {
                // CDC_SET_LINE_CODING
                //serial_print("set coding, waiting...\n");
                info!("set enc wait");
                Ep0TxAction::DoNothing // Awaiting data
            }
            _ => {
                info!("usb setup UNKNOWN @serial");
                // return Err("usb setup UNKNOWN");
                return;
            }
        };

        self.base.endpoint0_execute_send_action(action);

    }

    fn handle_ep0_out(&'static self, buf : &[u8;8]) {
        if let Some(setuppacket) = unsafe { (*self.setuppacket.get()).take() } {

            if setuppacket.request_and_type() == 0x2021 {
                // CDC_SET_LINE_CODING
                unsafe { self.cdc_state().line_coding = transmute_copy(buf); }
                info!("gle {:?}", self.cdc_state().line_coding);
                self.base.endpoint0_execute_send_action(Ep0TxAction::SendEmpty);

            }

        }
        // CC ifdef CDC_STATUS_INTERFACE
    // CC   if (setup.wRequestAndType == 0x2021 /*CDC_SET_LINE_CODING*/) {
    // CC     int i;
    // CC     uint8_t *dst = (uint8_t *)usb_cdc_line_coding;
    // CC     //serial_print("set line coding ");
    // CC     for (i=0; i<7; i++) {
    // CC       //serial_phex(*buf);
    // CC       *dst++ = *buf++;
    // CC     }
    // CC     //serial_phex32(usb_cdc_line_coding[0]);
    // CC     //serial_print("\n");
    // CC     if (usb_cdc_line_coding[0] == 134) usb_reboot_timer = 15;
    // CC     endpoint0_transmit(NULL, 0);
    // CC   }
    // CC endif
    }

    /// Returns next packet. If it isn't needed anymore, it should be stored into self.rx_packet
    /// via self.store_half_read_packet()
    ///
    /// It is guaranteed that the returned packet has more that zero bytes to read from
    fn read_next_packet(&self) -> Option<AllocatedUsbPacket> {
        let mut packet_option = unsafe { (*self.rx_packet.get()).take() };
        loop {
            if let Some(mut rx_packet) = packet_option.take() {
                if rx_packet.index() == rx_packet.len() || rx_packet.len() == 0 {
                    rx_packet.recycle(&self.base.pool, self);
                } else {
                    return Some(rx_packet);
                }
            } else {
                packet_option = self.base.fifos.dequeue(EndpointWithDir::new(CDC_RX_ENDPOINT, Direction::Rx));
                if packet_option.is_none() {
                    return None;
                }
                // TODO: reset index?
            }
        }
    }

    fn store_half_read_packet(&self, packet : AllocatedUsbPacket) {
        unsafe { *self.rx_packet.get() = Some(packet) };
    }


    pub fn discard_all_input(&self) {
        if let Some(rx_packet) = unsafe { (*self.rx_packet.get()).take() } {
            rx_packet.recycle(&self.base.pool, self);
        }
        loop {
            if let Some(rx_packet) = self.base.fifos.dequeue(EndpointWithDir::new(CDC_RX_ENDPOINT, Direction::Rx)) {
                rx_packet.recycle(&self.base.pool, self);
            } else {
                break;
            }
        }

    }

    fn flush_tx(&mut self) {
        if let Some(mut tx_packet) = unsafe { (*self.tx_packet.get()).take() } {
            let ind = tx_packet.index();
            tx_packet.set_len(ind);
            self.base.tx(CDC_TX_ENDPOINT.into(), tx_packet);
        }
    }

    fn alloc_tx(&mut self) -> IoResult<usize> {
        self.flush_tx();
        loop {
            unsafe { *self.tx_packet.get() = self.base.pool.allocate() };
            if let Some(mut tx_packet) = unsafe { (*self.tx_packet.get()).as_mut() } {
                tx_packet.buf_mut(64);
                tx_packet.set_index(0);
                self.count_consecutive_unsuccessfull_alloc_attempts = 0;
                return Ok(0)
            } else {
                self.count_consecutive_unsuccessfull_alloc_attempts += 1;
                if self.count_consecutive_unsuccessfull_alloc_attempts >= TX_TIMEOUT {
                    return Err(IoError::new(IoErrorKind::TimedOut, ""))
                }
            }
        }
    }
}

impl HandlePriorityAllocation for UsbSerial {
    /// Just now there are some new free packets
    fn handle_priority_allocation(&self, packet : AllocatedUsbPacket) -> Option<AllocatedUsbPacket> {
        (&self.base).handle_priority_allocation(packet)
    }
}

impl IoRead for UsbSerial {
    fn read(&mut self, buf: &mut [u8]) -> IoResult<usize> {
        if let Some(mut packet) = self.read_next_packet() {
            let index = packet.index() as usize;
            let packet_len = packet.len() as usize;
            let remaining = packet_len - index;
            if remaining == 0 {
                return Err(IoError::new(IoErrorKind::Other, "internal usb serial logic error 3"))
            } else {
                let copy = min(buf.len(), remaining);
                packet.set_index((index + copy) as u16);
                let mut buf = &mut buf[0..copy];
                buf.copy_from_slice(&packet.buf()[index..index+copy]);

                if copy != remaining {
                    self.store_half_read_packet(packet);
                } else {
                    packet.recycle(&self.base.pool, self);
                }

                return Ok(copy);
            }

        } else {
            Err(IoError::new(IoErrorKind::Other, ""))
        }
    }
}

impl IoWrite for UsbSerial {
    fn write(&mut self, buf: &[u8]) -> IoResult<usize> {
        if unsafe { (*self.tx_packet.get()).as_mut().is_none() } {
            try!(self.alloc_tx());
        }
        // We assume that we flush the packet as soon as it is full.
        // So it is not possible that we have a full packet here.
        // TODO: is this true if we limit the number of tx packets?

        let (flush, written) = if let Some(mut tx_packet) = unsafe { (*self.tx_packet.get()).as_mut() } {
            let index = tx_packet.index() as usize;
            let packet_len = tx_packet.len() as usize;
            let remaining = packet_len - index;
            if remaining == 0 {
                return Err(IoError::new(IoErrorKind::Other, "internal usb serial logic error"))
            } else {
                let copy = min(buf.len(), remaining);
                tx_packet.set_index((index + copy) as u16);
                let mut usb_buf = &mut tx_packet.buf_mut(packet_len as u16)[index..index+copy];
                usb_buf.copy_from_slice(&buf[0..copy]);
                (copy == remaining, copy)
            }
        } else {
            return Err(IoError::new(IoErrorKind::Other, "internal usb serial logic error 2"))
        };
        if flush {
            self.flush_tx();
        }
        Ok(written)
    }

    fn flush(&mut self) -> IoResult<()> {
        self.flush_tx();
        Ok(())
        // TODO: wait until buffer is read by USB?
    }

}

impl FmtWrite for UsbSerial {
    fn write_str(&mut self, mut s: &str) -> FmtResult {
        match self.write_all(s.as_bytes()) {
            Ok(..) => Ok(()), // write all writes all bytes
            Err(..) => Err(FmtError)
        }
    }
}