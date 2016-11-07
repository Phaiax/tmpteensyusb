
use zinc::hal::k20::regs::*;
use usbmempool::{MemoryPoolRef, UsbPacket, AllocatedUsbPacket, MemoryPoolTrait};
use usbmem::Fifos;
use usb;
use generated;
use zinc;
use core::cmp;
// use usbmem;

// use usbmem::{Fifos, Ep};
// use usbmem::{ForEnqueue};
use core::mem::drop;
use zinc::hal::cortex_m4::irq::NoInterrupts;


pub const EP0_SIZE: usize = 64;

#[derive(Copy, Clone)]
enum TxState {
    BothFreeEvenFirst = 0,
    BothFreeOddFirst = 1,
    EvenFree = 2,
    OddFree = 3,
    NoneFreeEvenFirst = 4,
    NoneFreeOddFirst = 5,
}

enum Ep0Tx {
    /// Nothing needs to be sent. (May be that a Static)
    Nothing,
    /// same as Nothing, but the static ptr is still in the BufferDescriptor
    StaticFinishing,
    /// Transmit the data in this packet.
    /// We assume that we do not need to split the data
    /// into multiple chunks because the length will be
    /// max 3 or max 8 bytes and the host will always
    /// request the whole length in one step. We panic if
    /// the host does not do so. The packet is owned by
    /// the buffer descriptor.
    CustomOwnedByBd0(usb::OddEven),
    /// request
    SendCustom,
    /// Some fat pointer to static data. This data probably
    /// needs to be sent in multiple steps. We move the
    /// fat pointer along. The `keep` is to
    Static(&'static [u8]),
    /// request
    SendStatic(&'static [u8]),
}


/// This struct helps with the lazy initialization of a `'static` usb driver so that the isr can access it.
pub struct UsbDriverOption(Option<UsbDriver>);

impl UsbDriverOption {
    /// Const function to allow definition of `static mut USBDRIVER`. The USBDRIVER needs to be initialized.
    pub const fn none() -> UsbDriverOption {
        UsbDriverOption(None)
    }
    /// Initialize self, that means transition from `None` to `Some`. Afterwards calls to `unwrap` are possible.
    pub fn init(&mut self, pool: MemoryPoolRef<[UsbPacket; 32]>) {
        match self.0 {
            None => {
                *self = UsbDriverOption(Some(UsbDriver::new(pool)));
            }
            Some(_) => {
                panic!();
            }
        }
    }
    /// Get a reference to the UsbDriver. Panics if not initialized.
    pub fn unwrap(&'static mut self) -> UsbDriverRef {
        match self.0 {
            Some(ref mut p) => UsbDriverRef(p),
            None => {
                panic!("Call UsbDriverOption::init() first");
            }
        }
    }
}



/// This type wraps a `'static` driver. We must only allocate `'static`
/// memory because the interrupt service routine needs to find the driver struct.
///
/// Also the only way to get this type is by using a `UsbDriverOption`.
pub struct UsbDriverRef(&'static mut UsbDriver);


pub struct UsbDriver {
    /// Selected configuration id (by host via SET_CONFIGURATION)
    usb_configuration: u8,
    usb_reboot_timer: u8,
    /// ?
    usb_rx_byte_count_data: [u16; 16], // [;NUM_ENDPOINTS]
    tx_state: [TxState; 16],

    pool: MemoryPoolRef<[UsbPacket; 32]>,
    fifos: Fifos,

    ep0_rx0_buf: [u8; EP0_SIZE], // __attribute__ ((aligned (4)));
    ep0_rx1_buf: [u8; EP0_SIZE], // __attribute__ ((aligned (4)));
    ep0_tx: Ep0Tx, // ep0_tx_len;
    ep0_tx_custom: Option<AllocatedUsbPacket>,
    ep0_tx_bdt_bank: usb::OddEven,
    ep0_tx_data_toggle: usb::BufferDescriptor_control_data01,
    ep0_setuppacket: usb::SetupPacket,
}


struct UsbSerial {
  //  #ifdef CDC_DATA_INTERFACE
  //extern uint32_t usb_cdc_line_coding[2];
  //extern volatile uint32_t usb_cdc_line_rtsdtr_millis;
  //extern volatile uint32_t systick_millis_count;
  //extern volatile uint8_t usb_cdc_line_rtsdtr;
  //extern volatile uint8_t usb_cdc_transmit_flush_timer;
  //extern void usb_serial_flush_callback(void);
  //#endif

}

use core::ptr;
impl UsbDriver {
    pub fn new(pool: MemoryPoolRef<[UsbPacket; 32]>) -> UsbDriver {
        info!("usb init");
        for bd in generated::BufferDescriptors().iter_mut() {
            bd.control.ignoring_state().zero_all();
            bd.addr.set_addr(0);
        }

        let packet_for_ep0 = pool.allocate().unwrap();
        let result = UsbDriver {
            usb_configuration: 0,
            usb_reboot_timer: 0,

            usb_rx_byte_count_data: [0u16; 16], // [;NUM_ENDPOINTS]
            tx_state: [TxState::BothFreeEvenFirst; 16],

            pool: pool,
            fifos: Fifos::new(),

            ep0_rx0_buf: [0u8; EP0_SIZE], // __attribute__ ((aligned (4)));
            ep0_rx1_buf: [0u8; EP0_SIZE], // __attribute__ ((aligned (4)));
            ep0_tx: Ep0Tx::Nothing, // ep0_tx_len;
            ep0_tx_custom: Some(packet_for_ep0),
            ep0_tx_bdt_bank: usb::OddEven::Even,
            ep0_tx_data_toggle: usb::BufferDescriptor_control_data01::Data0,
            ep0_setuppacket: usb::SetupPacket::default(),
        };

        // this basically follows the flowchart in the Kinetis
        // Quick Reference User Guide, Rev. 1, 03/2012, page 141

        // Allow the USB-FS to access the flash. See K20 doc, chapter 3.3.6.1 Crossbar Switch Master Assignments
        FMC().pfapr.set_m3ap(Fmc_pfapr_m3ap::ReadAndWrite);

        // assume 48 MHz clock already running
        // SIM - enable clock
        SIM().scgc4.set_usbotg(Sim_scgc4_usbotg::ClockEnabled);

        // CC   // reset USB module
        // CC   //USB0_USBTRC0 = USB_USBTRC_USBRESET;
        // USB().usbtrc0.ignoring_state().clear_usbreset();
        // CC   //while ((USB0_USBTRC0 & USB_USBTRC_USBRESET) != 0) ; // wait for reset to end
        // while USB().usbtrc0.usbreset() == true {}

        // set desc table base addr
        let buffertableaddr = &generated::BufferDescriptors()[0] as *const _ as usize;
        USB().bdtpage1.set_bdtba((buffertableaddr >> 8) as u8);
        USB().bdtpage2.set_bdtba((buffertableaddr >> 16) as u8);
        USB().bdtpage3.set_bdtba((buffertableaddr >> 24) as u8);

        // clear all ISR flags
        USB().istat.ignoring_state().clear_all();
        USB().errstat.ignoring_state().clear_all();
        USB().otgistat.ignoring_state().clear_all();

        // CC   //USB0_USBTRC0 |= 0x40; // undocumented bit
        // USB().usbtrc0.set_undocumented(true);

        // enable USB
        USB().ctl.ignoring_state().set_usbensofen(Usb_ctl_usbensofen::EnableUSBModule);
        // CC   USB0_USBCTRL = 0;
        USB()
            .usbctrl
            .ignoring_state()
            .set_susp(Usb_usbctrl_susp::NotSuspended)
            .set_pde(Usb_usbctrl_pde::PulldownsDisabled);

        // enable reset interrupt
        USB().inten.ignoring_state().set_usbrsten(Usb_inten_usbrsten::InterruptEnabled);

        // enable interrupt in NVIC...
        let IRQ_USBOTG = 73; // mcu spcific!!
        zinc::hal::cortex_m4::nvic::set_priority(IRQ_USBOTG, 112);
        zinc::hal::cortex_m4::nvic::enable_irq(IRQ_USBOTG);
        // enable d+ pullup
        USB()
            .control
            .ignoring_state()
            .set_dppullupnonotg(Usb_control_dppullupnonotg::PullupEnabled);

        result
    }

    /// *
    fn get_bd_ep(&self,
                 ep: usize,
                 txrx: usb::TxRx,
                 odd: usb::OddEven)
                 -> &'static usb::BufferDescriptor {
        let bdid = (ep << 2) | txrx as usize | odd as usize;
        assert!(bdid < generated::NUM_BUFFERDESCRIPTORS);
        &generated::BufferDescriptors()[bdid]
    }

    /// )
    fn get_bd(&self,
              ep: usb::Ep,
              txrx: usb::TxRx,
              odd: usb::OddEven)
              -> &'static usb::BufferDescriptor {
        let bdid = (ep as usize) | txrx as usize | odd as usize;
        assert!(bdid < generated::NUM_BUFFERDESCRIPTORS);
        &generated::BufferDescriptors()[bdid]
    }

    /// Will be called if a IN, OUT or SETUP transaction happened on endpoint 0
    pub fn endpoint0_process_transaction(&mut self, last_transaction: Usb_stat_Get) {

        // last_transaction: [ 7..4 endp, 3 txrx, 2 odd, 1..0 _]
        // should work if host does correct stuff
        let bdid = (last_transaction.raw() >> 2) as usize;
        assert!(bdid < generated::NUM_BUFFERDESCRIPTORS);
        let b = &generated::BufferDescriptors()[bdid];


        let pid = b.control.pid_tok();
        match pid {
            0x0D => {
                // SETUP from host
                self.ep0_setuppacket = unsafe { b.interpret_buf_as_setup_packet() };

                // give the buffer back
                b.control
                    .ignoring_state()
                    .give_back(EP0_SIZE, usb::BufferDescriptor_control_data01::Data1);

                // clear any leftover pending IN transactions
                self.ep0_tx = match self.ep0_tx {
                    Ep0Tx::CustomOwnedByBd0(oddeven) => {
                        let bd = self.get_bd(usb::Ep::Ep0, usb::TxRx::Tx, oddeven);
                        self.ep0_tx_custom = Some(unsafe { bd.swap_usb_packet(None).unwrap() });
                        Ep0Tx::Nothing
                    }
                    Ep0Tx::Nothing => Ep0Tx::Nothing,
                    Ep0Tx::SendCustom => Ep0Tx::Nothing,
                    Ep0Tx::StaticFinishing => Ep0Tx::Nothing,
                    Ep0Tx::SendStatic(_) => Ep0Tx::Nothing,
                    Ep0Tx::Static(_) => Ep0Tx::Nothing,
                };

                self.get_bd(usb::Ep::Ep0, usb::TxRx::Tx, usb::OddEven::Even)
                    .control
                    .ignoring_state()
                    .zero_all();
                self.get_bd(usb::Ep::Ep0, usb::TxRx::Tx, usb::OddEven::Odd)
                    .control
                    .ignoring_state()
                    .zero_all();
                // first IN after Setup is always DATA1
                self.ep0_tx_data_toggle = usb::BufferDescriptor_control_data01::Data1;

                // actually "do" the setup request
                self.endpoint0_process_setup_transaction();
                // unfreeze the USB, now that we're ready
                USB().ctl.ignoring_state().set_usbensofen(Usb_ctl_usbensofen::EnableUSBModule); // clear TXSUSPENDTOKENBUSY bit
            }

            0x01 | 0x02 => {
                // OUT transaction received from host
                info!("usb control PID=OUT {}, r&t {:x}",
                      pid,
                      self.ep0_setuppacket.request_and_type());
                // CC fdef CDC_STATUS_INTERFACE
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
                // CC ndif
                // give the buffer back
                b.control
                    .ignoring_state()
                    .give_back(EP0_SIZE, usb::BufferDescriptor_control_data01::Data1);
            }
            0x09 => {
                // IN transaction completed to host
                // send remaining data, if any...
                self.ep0_tx = match self.ep0_tx {
                    Ep0Tx::Nothing => {
                        // interesting
                        info!("Interesting 123");
                        Ep0Tx::Nothing
                    }
                    Ep0Tx::SendCustom => {
                        info!("Interesting 342");
                        Ep0Tx::Nothing
                    }
                    Ep0Tx::StaticFinishing => Ep0Tx::Nothing,
                    Ep0Tx::SendStatic(_) => {
                        info!("Interesting 324");
                        Ep0Tx::Nothing
                    }
                    Ep0Tx::Static(data) => {
                        let remaining = self.endpoint0_transmit_s(data);
                        if remaining.len() == 0 {
                            Ep0Tx::StaticFinishing
                        } else {
                            Ep0Tx::Static(remaining)
                        }
                    }
                    Ep0Tx::CustomOwnedByBd0(oddeven) => {
                        let bd2 = self.get_bd(usb::Ep::Ep0, usb::TxRx::Tx, oddeven);
                        assert_eq!(bd2 as *const usb::BufferDescriptor,
                                   b as *const usb::BufferDescriptor);
                        self.ep0_tx_custom = Some(unsafe { b.swap_usb_packet(None).unwrap() });
                        Ep0Tx::Nothing
                    }
                };

                if self.ep0_setuppacket.request_and_type() == 0x0500 {
                    // SET_ADDRESS
                    self.ep0_setuppacket.clear_request();
                    USB()
                        .addr
                        .set_addr(self.ep0_setuppacket.wValue() as u8)
                        .set_lsen(Usb_addr_lsen::RegularSpeed);
                }
            }
            _ => {
                info!("usb control. PID unknown");
            }
        }
        USB().ctl.ignoring_state().set_usbensofen(Usb_ctl_usbensofen::EnableUSBModule); // clear TXSUSPENDTOKENBUSY bit
    }

    pub fn endpoint0_process_setup_transaction(&mut self) {
        match self.ep0_setuppacket.request_and_type() {
            0x0500 => {
                // SET_ADDRESS (do nothing here, delay until IN is completed)
                self.ep0_tx = Ep0Tx::SendStatic(&generated::DEVICEDESCRIPTOR[0..0]);
            } // SET_ADDRESS
            0x0900 => {
                // SET_CONFIGURATION
                info!("usb setup set config {}", self.ep0_setuppacket.wValue());
                self.usb_configuration = self.ep0_setuppacket.wValue() as u8;
                for bd in generated::BufferDescriptors().iter().skip(4) {
                    // WHY free the packets owned by the controller?
                    if bd.control.own() == usb::BufferDescriptor_control_own::Controller {
                        match bd.swap_usb_packet(None) {
                            Some(p) => {
                                p.recycle(&self.pool);
                            }
                            None => {}
                        }
                    }
                }
                self.fifos.clear_all(&self.pool);
                for i in self.usb_rx_byte_count_data.iter_mut() {
                    *i = 0;
                }
                for i in self.tx_state.iter_mut() {
                    match *i {
                        TxState::EvenFree |
                        TxState::NoneFreeEvenFirst => {
                            *i = TxState::BothFreeEvenFirst;
                        }
                        TxState::OddFree |
                        TxState::BothFreeOddFirst => {
                            *i = TxState::BothFreeOddFirst;
                        }
                        _ => {}
                    }
                }
                self.pool.clear_priority_allocation_requests();
                for (i, (epconf, endptreg)) in generated::EndpointconfigForRegisters()
                    .iter()
                    .zip(USB().endpt.iter())
                    .enumerate()
                    .skip(1) {

                    endptreg.endpt.ignoring_state().set_raw(*epconf);
                    if i > generated::MAX_ENDPOINT_ADDR as usize {
                        continue;
                    }

                    if endptreg.endpt.eprxen() == Usb_endpt_endpt_eprxen::RxEnabled {
                        let bd = self.get_bd_ep(i, usb::TxRx::Rx, usb::OddEven::Even);
                        match self.pool.allocate() {
                            Some(p) => {
                                bd.swap_usb_packet(Some(p));
                                bd.control
                                    .ignoring_state()
                                    .give_back(64, usb::BufferDescriptor_control_data01::Data0);
                            }
                            None => {
                                bd.control.ignoring_state().zero_all();
                                self.pool.allocate_priority();
                            }
                        }
                        let bd = self.get_bd_ep(i, usb::TxRx::Rx, usb::OddEven::Odd);
                        match self.pool.allocate() {
                            Some(p) => {
                                bd.swap_usb_packet(Some(p));
                                bd.control
                                    .ignoring_state()
                                    .give_back(64, usb::BufferDescriptor_control_data01::Data1);
                            }
                            None => {
                                bd.control.ignoring_state().zero_all();
                                self.pool.allocate_priority();
                            }
                        }
                    }
                    self.get_bd_ep(i, usb::TxRx::Tx, usb::OddEven::Even)
                        .control
                        .ignoring_state()
                        .zero_all();
                    self.get_bd_ep(i, usb::TxRx::Tx, usb::OddEven::Odd)
                        .control
                        .ignoring_state()
                        .zero_all();
                } // end for
                self.ep0_tx = Ep0Tx::SendStatic(&generated::DEVICEDESCRIPTOR[0..0]);

            }
            0x0880 => {
                // GET_CONFIGURATION
                info!("usb setup get config");
                if let Ep0Tx::Nothing = self.ep0_tx {
                    self.ep0_tx_custom.as_mut().unwrap().buf_mut(1)[0] = self.usb_configuration;
                    self.ep0_tx = Ep0Tx::SendCustom;
                } else {
                    info!("adwe4g");
                }
            }
            0x0080 => {
                // GET_STATUS (device)
                info!("usb setup get device status");
                if let Ep0Tx::Nothing = self.ep0_tx {
                    let buf = self.ep0_tx_custom.as_mut().unwrap().buf_mut(2);
                    buf[0] = 0;
                    buf[1] = 0;
                    self.ep0_tx = Ep0Tx::SendCustom;
                } else {
                    info!("2av4");
                }
            }
            0x0082 => {
                // GET_STATUS (endpoint)
                info!("usb setup get endpoint status");
                if self.ep0_setuppacket.wIndex() > 16 {
                    // CC      // TODO: do we need to handle IN vs OUT here?
                    Self::endpoint0_stall();
                    return;
                }
                if let Ep0Tx::Nothing = self.ep0_tx {
                    let buf = self.ep0_tx_custom.as_mut().unwrap().buf_mut(2);
                    buf[0] =
                        if USB().endpt[self.ep0_setuppacket.wIndex() as usize].endpt.epstall() ==
                           Usb_endpt_endpt_epstall::Stalled {
                            1
                        } else {
                            0
                        };
                    buf[1] = 0;
                    self.ep0_tx = Ep0Tx::SendCustom;
                } else {
                    info!("2av4");
                }
            }
            0x0102 => {
                // CLEAR_FEATURE (endpoint)
                info!("usb setup: cear feature");
                // there is only one feature availabel for an endpoint: ENDPOINT_HALT <=> wValue=0
                if self.ep0_setuppacket.wIndex() > 16 || self.ep0_setuppacket.wValue() != 0 {
                    Self::endpoint0_stall();
                    return;
                }
                USB().endpt[self.ep0_setuppacket.wIndex() as usize]
                    .endpt
                    .set_epstall(Usb_endpt_endpt_epstall::NotStalled);
                // CC    // TODO: do we need to clear the data toggle here?
                self.ep0_tx = Ep0Tx::SendStatic(&generated::DEVICEDESCRIPTOR[0..0]);
            }
            0x0302 => {
                // SET_FEATURE (endpoint)
                info!("usb setup set feature");
                // there is only one feature availabel for an endpoint: ENDPOINT_HALT <=> wValue=0
                if self.ep0_setuppacket.wIndex() > 16 || self.ep0_setuppacket.wValue() != 0 {
                    // CC      // TODO: do we need to handle IN vs OUT here?
                    Self::endpoint0_stall();
                    return;
                }
                USB().endpt[self.ep0_setuppacket.wIndex() as usize]
                    .endpt
                    .set_epstall(Usb_endpt_endpt_epstall::Stalled);
                // CC    // TODO: do we need to clear the data toggle here?
                self.ep0_tx = Ep0Tx::SendStatic(&generated::DEVICEDESCRIPTOR[0..0]);
            }
            0x0680 | 0x0681 => {
                // GET_DESCRIPTOR (0x0680 calls on device level, 0x0681 on interface level)
                // Normally 0x0681 should never happen, but I guess there is a misbehaving host somewhere in the wild
                let descr_type = (self.ep0_setuppacket.wValue() >> 8) as u8;
                let mut data: &'static [u8] = match descr_type {
                    1 => {
                        // device
                        generated::DEVICEDESCRIPTOR
                    }
                    2 => {
                        // configuration
                        generated::CONFIGDESCRIPTORTREE
                    }
                    3 => {
                        // str
                        let index = self.ep0_setuppacket.wValue() as u8;
                        generated::get_str(index)
                    }
                    _ => {
                        // not found
                        Self::endpoint0_stall();
                        return;
                    }
                };
                let requested_len = self.ep0_setuppacket.wLength() as usize;
                if data.len() > requested_len {
                    data = &data[0..requested_len];
                }
                self.ep0_tx = Ep0Tx::SendStatic(data);
            }
            _ => {
                info!("usb setup UNKNOWN");
                Self::endpoint0_stall();
                return;
            }
        } // match req and type

        // CC  if (datalen > setup.wLength) datalen = setup.wLength;

        match self.ep0_tx {
            Ep0Tx::Nothing => {}
            Ep0Tx::Static(data) => {
                info!("3y334tgv");
            }
            Ep0Tx::SendStatic(mut data) => {
                // -!!! info!("usb setup : send static");
                data = self.endpoint0_transmit_s(data);
                if data.len() > 0 {
                    // -!!! info!("more");
                    data = self.endpoint0_transmit_s(data);
                }
                if data.len() > 0 {
                    // -!!! info!("more1");
                    self.ep0_tx = Ep0Tx::SendStatic(data);
                } else {
                    // -!!! info!("thats it");
                    self.ep0_tx = Ep0Tx::StaticFinishing;
                }
            }
            Ep0Tx::SendCustom => {
                // -!!! info!("usb setup : snd custom");
                let p = self.ep0_tx_custom.take().unwrap();
                let bank = self.endpoint0_transmit_c(p);
                self.ep0_tx = Ep0Tx::CustomOwnedByBd0(bank);
            }
            Ep0Tx::StaticFinishing => {
                info!("3y334t");
            }
            Ep0Tx::CustomOwnedByBd0(_) => {
                info!("3y34y");
            }
        }
    }

    pub fn endpoint0_transmit_s(&mut self, buf: &'static [u8]) -> &'static [u8] {
        let chunksize = cmp::min(EP0_SIZE, buf.len());
        let bd = self.get_bd(usb::Ep::Ep0, usb::TxRx::Tx, self.ep0_tx_bdt_bank);

        bd.addr.ignoring_state().set_addr(buf.as_ptr() as u32);
        bd.control.ignoring_state().give_back(chunksize, self.ep0_tx_data_toggle);

        self.ep0_tx_data_toggle.toggle();
        self.ep0_tx_bdt_bank.toggle();

        &buf[chunksize..]
    }

    pub fn endpoint0_transmit_c(&mut self, buf: AllocatedUsbPacket) -> usb::OddEven {
        let len = buf.buf().len();
        let bd = self.get_bd(usb::Ep::Ep0, usb::TxRx::Tx, self.ep0_tx_bdt_bank);
        bd.swap_usb_packet(Some(buf));
        bd.control.ignoring_state().give_back(len, self.ep0_tx_data_toggle);
        self.ep0_tx_data_toggle = match self.ep0_tx_data_toggle {
            usb::BufferDescriptor_control_data01::Data1 => {
                usb::BufferDescriptor_control_data01::Data0
            }
            usb::BufferDescriptor_control_data01::Data0 => {
                usb::BufferDescriptor_control_data01::Data1
            }
        };
        let used_bank = self.ep0_tx_bdt_bank;
        self.ep0_tx_bdt_bank = match self.ep0_tx_bdt_bank {
            usb::OddEven::Odd => usb::OddEven::Even,
            usb::OddEven::Even => usb::OddEven::Odd,
        };
        used_bank
    }

    pub fn endpoint0_stall() {
        USB().endpt[0]
            .endpt
            .ignoring_state()
            .set_epstall(Usb_endpt_endpt_epstall::Stalled)
            .set_eprxen(Usb_endpt_endpt_eprxen::RxEnabled)
            .set_eptxen(Usb_endpt_endpt_eptxen::TxEnabled)
            .set_ephshk(Usb_endpt_endpt_ephshk::Handshake);
    }

    pub fn usb_reset(&self) {
        // initialize BDT toggle bits
        USB().ctl.ignoring_state().set_oddrst(true);
        let drv = generated::driver_ref().0;
        drv.ep0_tx_bdt_bank = usb::OddEven::Even;

        // set up buffers to receive Setup and OUT packets
        self.get_bd(usb::Ep::Ep0, usb::TxRx::Rx, usb::OddEven::Even)
            .addr
            .set_addr(&drv.ep0_rx0_buf as *const u8 as u32);
        self.get_bd(usb::Ep::Ep0, usb::TxRx::Rx, usb::OddEven::Even)
            .control
            .ignoring_state()
            .give_back(EP0_SIZE, usb::BufferDescriptor_control_data01::Data0);
        self.get_bd(usb::Ep::Ep0, usb::TxRx::Rx, usb::OddEven::Odd)
            .addr
            .set_addr(&drv.ep0_rx1_buf as *const u8 as u32);
        self.get_bd(usb::Ep::Ep0, usb::TxRx::Rx, usb::OddEven::Odd)
            .control
            .ignoring_state()
            .give_back(EP0_SIZE, usb::BufferDescriptor_control_data01::Data0);
        self.get_bd(usb::Ep::Ep0, usb::TxRx::Tx, usb::OddEven::Even)
            .control
            .ignoring_state()
            .zero_all();
        self.get_bd(usb::Ep::Ep0, usb::TxRx::Tx, usb::OddEven::Odd)
            .control
            .ignoring_state()
            .zero_all();


        // activate endpoint 0
        USB().endpt[0]
            .endpt
            .set_eprxen(Usb_endpt_endpt_eprxen::RxEnabled)
            .set_eptxen(Usb_endpt_endpt_eptxen::TxEnabled)
            .set_ephshk(Usb_endpt_endpt_ephshk::Handshake);

        // clear all ending interrupts
        USB().errstat.ignoring_state().clear_all();
        USB().istat.ignoring_state().clear_all();

        // set the address to zero during enumeration
        USB().addr.set_addr(0).set_lsen(Usb_addr_lsen::RegularSpeed);

        // enable other interrupts
        USB().erren.ignoring_state().enable_all();
        USB().inten.ignoring_state().set_tokdneen(Usb_inten_tokdneen::InterruptEnabled)
                                    .set_softoken(Usb_inten_softoken::InterruptEnabled)
                                    .set_stallen(Usb_inten_stallen::InterruptEnabled)
                                    .set_erroren(Usb_inten_erroren::InterruptEnabled)
                                    .set_usbrsten(Usb_inten_usbrsten::InterruptEnabled)
                                    .set_sleepen(Usb_inten_sleepen::InterruptEnabled)
                                    .set_resumeen(Usb_inten_resumeen::InterruptDisabled) // disabled
                                    .set_attachen(Usb_inten_attachen::InterruptDisabled); // disabled

        // USB Enable Setting this bit causes the SIE to reset all of its ODD bits to the BDTs.
        // is this necessary? //*yes because we disabled usb above
        USB().ctl.ignoring_state().set_usbensofen(Usb_ctl_usbensofen::EnableUSBModule);
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








#[allow(dead_code)]
#[no_mangle]
pub unsafe extern "C" fn isr_usb() {

    //   restart:
    let mut status = USB().istat.get();
    loop {
        //   status = USB0_ISTAT;
        status = USB().istat.get();

        if status.softok() {
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
            USB().istat.ignoring_state().clear_softok();
        }

        if status.tokdne() {
            // This bit is set when the current token being processed has completed. The processor must immediately
            // read the STATUS (STAT) register to determine the EndPoint and BD used for this token. Clearing this bit
            // (by writing a one) causes STAT to be cleared or the STAT holding register to be loaded into the STAT
            // register.
            let last_transaction = USB().stat.get();
            if last_transaction.endp() == 0 {
                generated::driver_ref().0.endpoint0_process_transaction(last_transaction);
            } else {
                info!("unhandeled 34")
                //       bdt_t *b = stat2bufferdescriptor(stat);
                //       usb_packet_t *packet = (usb_packet_t *)((uint8_t *)(b->addr) - 8);
                // #if 0
                //       serial_print("ep:");
                //       serial_phex(endpoint);
                //       serial_print(", pid:");
                //       serial_phex(BDT_PID(b->desc));
                //       serial_print(((uint32_t)b & 8) ? ", odd" : ", even");
                //       serial_print(", count:");
                //       serial_phex(b->desc >> 16);
                //       serial_print("\n");
                // #endif
                //       endpoint--; // endpoint is index to zero-based arrays

                //       if (stat & 0x08) { // transmit
                //         usb_free(packet);
                //         packet = tx_first[endpoint];
                //         if (packet) {
                //           //serial_print("tx packet\n");
                //           tx_first[endpoint] = packet->next;
                //           b->addr = packet->buf;
                //           switch (tx_state[endpoint]) {
                //             case TX_STATE_BOTH_FREE_EVEN_FIRST:
                //             tx_state[endpoint] = TX_STATE_ODD_FREE;
                //             break;
                //             case TX_STATE_BOTH_FREE_ODD_FIRST:
                //             tx_state[endpoint] = TX_STATE_EVEN_FREE;
                //             break;
                //             case TX_STATE_EVEN_FREE:
                //             tx_state[endpoint] = TX_STATE_NONE_FREE_ODD_FIRST;
                //             break;
                //             case TX_STATE_ODD_FREE:
                //             tx_state[endpoint] = TX_STATE_NONE_FREE_EVEN_FIRST;
                //             break;
                //             default:
                //             break;
                //           }
                //           b->desc = BDT_DESC(packet->len,
                //             ((uint32_t)b & 8) ? DATA1 : DATA0);
                //         } else {
                //           //serial_print("tx no packet\n");
                //           switch (tx_state[endpoint]) {
                //             case TX_STATE_BOTH_FREE_EVEN_FIRST:
                //             case TX_STATE_BOTH_FREE_ODD_FIRST:
                //             break;
                //             case TX_STATE_EVEN_FREE:
                //             tx_state[endpoint] = TX_STATE_BOTH_FREE_EVEN_FIRST;
                //             break;
                //             case TX_STATE_ODD_FREE:
                //             tx_state[endpoint] = TX_STATE_BOTH_FREE_ODD_FIRST;
                //             break;
                //             default:
                //             tx_state[endpoint] = ((uint32_t)b & 8) ?
                //               TX_STATE_ODD_FREE : TX_STATE_EVEN_FREE;
                //             break;
                //           }
                //         }
                //       } else { // receive
                //         packet->len = b->desc >> 16;
                //         if (packet->len > 0) {
                //           packet->index = 0;
                //           packet->next = NULL;
                //           if (rx_first[endpoint] == NULL) {
                //             //serial_print("rx 1st, epidx=");
                //             //serial_phex(endpoint);
                //             //serial_print(", packet=");
                //             //serial_phex32((uint32_t)packet);
                //             //serial_print("\n");
                //             rx_first[endpoint] = packet;
                //           } else {
                //             //serial_print("rx Nth, epidx=");
                //             //serial_phex(endpoint);
                //             //serial_print(", packet=");
                //             //serial_phex32((uint32_t)packet);
                //             //serial_print("\n");
                //             rx_last[endpoint]->next = packet;
                //           }
                //           rx_last[endpoint] = packet;
                //           usb_rx_byte_count_data[endpoint] += packet->len;
                //           // TODO: implement a per-endpoint maximum # of allocated
                //           // packets, so a flood of incoming data on 1 endpoint
                //           // doesn't starve the others if the user isn't reading
                //           // it regularly
                //           packet = usb_malloc();
                //           if (packet) {
                //             b->addr = packet->buf;
                //             b->desc = BDT_DESC(64,
                //               ((uint32_t)b & 8) ? DATA1 : DATA0);
                //           } else {
                //             //serial_print("starving ");
                //             //serial_phex(endpoint + 1);
                //             b->desc = 0;
                //             usb_rx_memory_needed++;
                //           }
                //         } else {
                //           b->desc = BDT_DESC(64, ((uint32_t)b & 8) ? DATA1 : DATA0);
                //         }
                //       }
            }
            USB().istat.ignoring_state().clear_tokdne();
        } else {
            break;
        }
    } // end loop

    if status.usbrst() {
        info!("r");
        generated::driver_ref().0.usb_reset();
        return;
    }

    if status.stall() {
        info!("isr stall");
        USB().endpt[0]
            .endpt
            .set_eprxen(Usb_endpt_endpt_eprxen::RxEnabled)
            .set_eptxen(Usb_endpt_endpt_eptxen::TxEnabled)
            .set_ephshk(Usb_endpt_endpt_ephshk::Handshake);
        USB().istat.ignoring_state().clear_stall();
    }

    if status.error() {
        let err = USB().errstat.get().raw();
        USB().errstat.ignoring_state().clear_raw(err);
        info!("isr err {:b}", err);
        USB().istat.ignoring_state().clear_error();
    }

    if status.sleep() {
        info!("isr sleep");
        USB().istat.ignoring_state().clear_sleep();
    }

}
