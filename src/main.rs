#![feature(plugin, start, core_intrinsics)]
#![no_std]

#[macro_use]
pub extern crate zinc;

pub mod generated;
#[macro_use]
pub mod usbmem;
pub mod usb;
// mod test;


use core::option::Option::Some;
use core::mem;
use core::cmp;

use zinc::hal::cortex_m4::systick;
use zinc::hal::k20::{pin, watchdog};
use zinc::hal::pin::Gpio;
use zinc::hal::k20::regs::*;
use zinc::hal::k20::uart_logger;
use zinc::hal::k20::uart::{Uart, UARTPeripheral};
use zinc::hal::uart::Parity;

pub use zinc as z;


/// Wait the given number of SysTick ticks
#[inline(never)]
pub fn wait(ticks: u32) {
  let mut n = ticks;
  // Reset the tick flag
  systick::tick();
  loop {
    if systick::tick() {
      n -= 1;
      if n == 0 {
        break;
      }
    }
  }
}

use zinc::drivers::chario::CharIO;
use core::fmt::Write;

pub fn main() {
  uart_logger::init( Uart::new(UARTPeripheral::UART0, 38400, 8, Parity::Disabled, 1,
    pin::Pin::new(pin::Port::PortB, 16, pin::Function::Gpio, Some(zinc::hal::pin::Out)),
    pin::Pin::new(pin::Port::PortB, 17, pin::Function::Gpio, Some(zinc::hal::pin::Out))) );

  let mut uartref = unsafe { &mut uart_logger::LOGGING_UART }.unwrap();

  // Pins for MC HCK (http://www.mchck.org/)
  let led1 = pin::Pin::new(pin::Port::PortC, 5, pin::Function::Gpio, Some(zinc::hal::pin::Out));


  info!("STARTING UP");
  wait(500);

  test_fifo_2();
  test_fifo();

  unsafe { usbdriver = Some(UsbDriver::new()); }


  loop {
    led1.set_high();
    wait(500);
    led1.set_low();
    wait(500);
  }

}

static mut usbdriver : Option<UsbDriver> = None;

use usbmem::{Fifos, Ep, UsbPacket, WithNP};
use core::mem::drop;

fn test_fifo_2() {

 {
  let pool = fifos!(0, packetbufsize: 0);
  info!("Pool is {} bytes", core::mem::size_of_val(&pool));
  let pool = fifos!();
  info!("Pool is {} bytes", core::mem::size_of_val(&pool));

 }

  // Create static pool. Must not be dropped as long as usb is active. Max 32 elements.
  let pool = fifos!(); // Defaults to fifos!(32, packetbufsize: 64)
  // let pool = fifos!(10); // Changes the memory pool size from 32 to 10 packets
  let pool = fifos!(10, packetbufsize: 30); // Changes the buf size of a packet to 30 bytes.

  // Enqueue into FIFO. Use endpoint 3

  // max 1 helper for each endpoint at a time
  let mut enq_helper_ep3 = pool.for_enqueuing(Ep::Tx3).unwrap();
  {
    let mut packet1 = enq_helper_ep3.enqueue().unwrap(); // allocate
    packet1.len = 1;
    packet1.buf[0] = b'a'; // buf is uninitialized! Buf defaults to 64 bytes.
    // The DROP does the enqueue
  }
  // enq_helper_ep3 can be reused as many times as needed.


  // Dequeue from FIFO. Use endpoint 3

  // max 1 helper for each endpoint at a time
  let mut deq_helper_ep3 = pool.for_dequeuing(Ep::Tx3).unwrap();
  {
    let packet1 = deq_helper_ep3.dequeue().unwrap(); // dequeue
    info!("Got {} bytes", packet1.len);
    work(packet1.buf()); // packet1.buf[packet1.len..] is uninitialized!
    // The DROP does the deallocate
  }
  // deq_helper_ep3 can be reused as many times as needed.

  fn work(data : &[u8]) {}


}

fn test_fifo() {
  let f = fifos!(32);

  let mut enq1 = f.for_enqueuing(Ep::Tx1).expect("Cant get enqueuer for EP 1");
  let mut deq1 = f.for_dequeuing(Ep::Tx1).expect("Cant get dequeuer for EP 1");
  enq1.enqueue().map(|mut packet| {packet.as_mut().len = 2; });
  deq1.dequeue().map(|packet| { packet.as_ref().len });
  assert!(deq1.dequeue().is_none(), "Can dequeue from empty queue.");

  // fill
  for i in 0..16 {
    enq1.enqueue().expect("Cant fill 16 Elements to EP1").as_mut().len = i;
  }


  assert!(f.for_enqueuing(Ep::Tx1).is_none(), "Can create two enqueuers for the same EP");
  drop(enq1);
  let mut enq1 = f.for_enqueuing(Ep::Tx1).expect("Cant get enqueuer for EP1 a second time");
  let mut enq2 = f.for_enqueuing(Ep::Rx15).expect("Cant get enqueuer for EP1 a second time");

  // fill more
  for i in 16..30 {
    enq1.enqueue().expect("Cant fill 16..30 Elements to EP1").as_mut().len = i;
  }
  enq2.enqueue().expect("Cant fill nr 31 to EP15").as_mut().len = 30;
  enq2.enqueue().expect("Cant fill nr 32 to EP15").as_mut().len = 31;
  assert!(enq2.enqueue().is_none(), "Can insert more packets than limit");

  // Test order
  for i in 0..20 {
    assert_eq!(deq1.dequeue().expect("Cant dequeue from EP1").as_ref().len, i);
  }
  drop(deq1);
  let mut deq15 = f.for_dequeuing(Ep::Rx15).expect("Cant get dequeuer for EP 15");
  assert!(f.for_dequeuing(Ep::Rx15).is_none(), "Can create two dequeuers for the same EP");
  assert_eq!(deq15.dequeue().expect("Cant dequeue from EP15").as_ref().len, 30);
  let mut deq1 = f.for_dequeuing(Ep::Tx1).expect("Cant get dequeuer for EP 1");
  assert_eq!(deq1.dequeue().expect("Cant dequeue from EP15").as_ref().len, 20);
  assert_eq!(deq15.dequeue().expect("Cant dequeue from EP15").as_ref().len, 31);
  for i in 21..30 {
    assert_eq!(deq1.dequeue().expect("Cant dequeue from EP1").as_ref().len, i);
  }
  info!(target: "usbmem", "FIFO test successfull");
}

use zinc::hal::cortex_m4::irq::NoInterrupts;

/// *
pub fn get_bd_ep(ep : usize, txrx : usb::TxRx, odd : usb::OddEven) -> &'static usb::BufferDescriptor {
  let bdid = ep | txrx as usize | odd as usize;
  assert!(bdid < generated::NUM_BUFFERDESCRIPTORS);
  &generated::BufferDescriptors()[bdid]
}

/// )
pub fn get_bd(ep : usb::Ep, txrx : usb::TxRx, odd : usb::OddEven) -> &'static usb::BufferDescriptor {
  get_bd_ep(ep as usize, txrx, odd)
}

pub const EP0_SIZE : usize = 64;

struct UsbDriver {
    /// Selected configuration id (by host via SET_CONFIGURATION)
    usb_configuration : u8,
    usb_reboot_timer : u8,
    /// ?
    // extern uint16_t usb_rx_byte_count_data[NUM_ENDPOINTS];
    usb_rx_byte_count_data : [u16 ; 2], // [;NUM_ENDPOINTS]
    fifos : Fifos<[WithNP<UsbPacket>; 32]>,

    ep0_rx0_buf : [u8; EP0_SIZE], // __attribute__ ((aligned (4)));
    ep0_rx1_buf : [u8; EP0_SIZE], // __attribute__ ((aligned (4)));
    ep0_tx_ptr : &'static [u8], // ep0_tx_len;
    ep0_tx_bdt_bank : u8,
    ep0_tx_data_toggle : u8,
    ep0_setuppacket : usb::SetupPacket,
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
  // CC void usb_init(void)
  fn new() -> UsbDriver {
    // CC {
    // CC   int i;
    // CC
    // CC   //serial_begin(BAUD2DIV(115200));
    // CC   //serial_print("usb_init\n");
    // CC
    // CC   usb_init_serialnumber();
    // will be generated by build.rs
    // CC
    // CC   for (i=0; i <= NUM_ENDPOINTS*4; i++) {
    // CC     table[i].desc = 0;
    // CC     table[i].addr = 0;
    // CC   }
    for bd in generated::BufferDescriptors().iter_mut() {
      bd.control.ignoring_state().zero_all();
      bd.addr.set_addr(0);
    }
    // CC
    // CC   // this basically follows the flowchart in the Kinetis
    // CC   // Quick Reference User Guide, Rev. 1, 03/2012, page 141
    // CC
    // CC   // assume 48 MHz clock already running
    // CC   // SIM - enable clock
    // CC   SIM_SCGC4 |= SIM_SCGC4_USBOTG;
    SIM().scgc4.set_usbotg(Sim_scgc4_usbotg::ClockEnabled);
    // CC
    // CC   // reset USB module
    // CC   //USB0_USBTRC0 = USB_USBTRC_USBRESET;
    USB().usbtrc0.ignoring_state().clear_usbreset();
    // CC   //while ((USB0_USBTRC0 & USB_USBTRC_USBRESET) != 0) ; // wait for reset to end
    while USB().usbtrc0.usbreset() == true {}
    // CC
    // CC   // set desc table base addr
    let buffertableaddr = &generated::BufferDescriptors() as *const _ as usize;
    // CC   USB0_BDTPAGE1 = ((uint32_t)table) >> 8;
    USB().bdtpage1.set_bdtba((buffertableaddr >> 8) as u8);
    // CC   USB0_BDTPAGE2 = ((uint32_t)table) >> 16;
    USB().bdtpage2.set_bdtba((buffertableaddr >> 16) as u8);
    // CC   USB0_BDTPAGE3 = ((uint32_t)table) >> 24;
    USB().bdtpage3.set_bdtba((buffertableaddr >> 24) as u8);
    // CC
    // CC   // clear all ISR flags
    // CC   USB0_ISTAT = 0xFF;
    USB().istat.ignoring_state().clear_all();
    // CC   USB0_ERRSTAT = 0xFF;
    USB().errstat.ignoring_state().clear_all();
    // CC   USB0_OTGISTAT = 0xFF;
    USB().otgistat.ignoring_state().clear_all();
    // CC
    // CC   //USB0_USBTRC0 |= 0x40; // undocumented bit
    USB().usbtrc0.set_undocumented(true);
    // CC
    // CC   // enable USB
    // CC   USB0_CTL = USB_CTL_USBENSOFEN;
    USB().ctl.ignoring_state().set_usbensofen(Usb_ctl_usbensofen::EnableUSBModule);
    // CC   USB0_USBCTRL = 0;
    USB().usbctrl.ignoring_state().set_susp(Usb_usbctrl_susp::NotSuspended)
                                  .set_pde(Usb_usbctrl_pde::PulldownsDisabled);
    // CC
    // CC   // enable reset interrupt
    // CC   USB0_INTEN = USB_INTEN_USBRSTEN;
    USB().inten.ignoring_state().set_usbrsten(Usb_inten_usbrsten::InterruptEnabled);
    // CC
    // CC   // enable interrupt in NVIC...
    // CC   NVIC_SET_PRIORITY(IRQ_USBOTG, 112);
    let IRQ_USBOTG = 73; // mcu spcific!!
    zinc::hal::cortex_m4::nvic::set_priority(IRQ_USBOTG, 112);
    // CC   NVIC_ENABLE_IRQ(IRQ_USBOTG);
    zinc::hal::cortex_m4::nvic::enable_irq(IRQ_USBOTG);
    // CC
    // CC   // enable d+ pullup
    // CC   USB0_CONTROL = USB_CONTROL_DPPULLUPNONOTG;
    USB().control.ignoring_state().set_dppullupnonotg(Usb_control_dppullupnonotg::PullupEnabled);
    // CC }

    UsbDriver {
      usb_configuration : 0,
      usb_reboot_timer : 0,

      usb_rx_byte_count_data : [0u16 ; 2], // [;NUM_ENDPOINTS]
      fifos : fifos!(),

      ep0_rx0_buf : [0u8; EP0_SIZE], // __attribute__ ((aligned (4)));
      ep0_rx1_buf : [0u8; EP0_SIZE], // __attribute__ ((aligned (4)));
      ep0_tx_ptr : &[], // ep0_tx_len;
      ep0_tx_bdt_bank : 0,
      ep0_tx_data_toggle : 0,
      ep0_setuppacket : usb::SetupPacket::default(),
    }

  }


  pub fn usb_control(&mut self, last_transaction : Usb_stat_Get) {

    // CC bdt_t *b;
    // CC uint32_t pid, size;
    // CC uint8_t *buf;
    // CC const uint8_t *data;

    // CC #define stat2bufferdescriptor(stat) (table + ((stat) >> 2))
    // CC b = stat2bufferdescriptor(stat);
    // last_transaction: [ 7..4 endp, 3 txrx, 2 odd, 1..0 _]
    // should work if host does correct stuff
    let bdid = (last_transaction.raw() >> 2) as usize;
    assert!(bdid < generated::NUM_BUFFERDESCRIPTORS);
    let b = generated::BufferDescriptors()[bdid];

    // CC #define BDT_PID(n)  (((n) >> 2) & 15)
    // CC pid = BDT_PID(b->desc);
    let pid = b.control.pid_tok();
    // CC //count = b->desc >> 16;
    // CC buf = b->addr;
    //let buf : &[u8] = b.buffer();
    // CC //serial_print("pid:");
    // CC //serial_phex(pid);
    // CC //serial_print(", count:");
    // CC //serial_phex(count);
    // CC //serial_print("\n");

    // CC switch (pid) {
    match pid {
    // CC case 0x0D: // Setup received from host
      0x0D => { // SETUP from host
    // CC   //serial_print("PID=Setup\n");
    // CC   //if (count != 8) ; // panic?
    // CC   // grab the 8 byte setup info
    // CC   setup.word1 = *(uint32_t *)(buf);
    // CC   setup.word2 = *(uint32_t *)(buf + 4);
        self.ep0_setuppacket = unsafe { b.interpret_buf_as_setup_packet() };

    // CC   // give the buffer back
    // CC #define BDT_DESC(count, data) (BDT_OWN | BDT_DTS \
    // CC    | ((data) ? BDT_DATA1 : BDT_DATA0) \
    // CC    | ((count) << 16))
    // CC   b->desc = BDT_DESC(EP0_SIZE, DATA1);
    // CC   //table[index(0, RX, EVEN)].desc = BDT_DESC(EP0_SIZE, 1);
    // CC   //table[index(0, RX, ODD)].desc = BDT_DESC(EP0_SIZE, 1);
      b.control.ignoring_state().give_back(EP0_SIZE, usb::BufferDescriptor_control_data01::Data1);

    // CC   // clear any leftover pending IN transactions
    // CC   ep0_tx_ptr = NULL;
      self.ep0_tx_ptr = &[];
    // CC   if (ep0_tx_data_toggle) {
    // CC   }
    // CC   //if (table[index(0, TX, EVEN)].desc & 0x80) {
    // CC     //serial_print("leftover tx even\n");
    // CC   //}
    // CC   //if (table[index(0, TX, ODD)].desc & 0x80) {
    // CC     //serial_print("leftover tx odd\n");
    // CC   //}
    // CC   table[index(0, TX, EVEN)].desc = 0;
      get_bd(usb::Ep::Ep0, usb::TxRx::Tx, usb::OddEven::Even).control.ignoring_state().zero_all();
    // CC   table[index(0, TX, ODD)].desc = 0;
      get_bd(usb::Ep::Ep0, usb::TxRx::Tx, usb::OddEven::Odd).control.ignoring_state().zero_all();
    // CC   // first IN after Setup is always DATA1
    // CC   ep0_tx_data_toggle = 1;
      self.ep0_tx_data_toggle = 1;

    // CC f 0
    // CC   serial_print("bmRequestType:");
    // CC   serial_phex(setup.bmRequestType);
    // CC   serial_print(", bRequest:");
    // CC   serial_phex(setup.bRequest);
    // CC   serial_print(", wValue:");
    // CC   serial_phex16(setup.wValue);
    // CC   serial_print(", wIndex:");
    // CC   serial_phex16(setup.wIndex);
    // CC   serial_print(", len:");
    // CC   serial_phex16(setup.wLength);
    // CC   serial_print("\n");
    // CC ndif
    // CC   // actually "do" the setup request
    // CC   usb_setup();
      self.usb_setup();
    // CC   // unfreeze the USB, now that we're ready
    // CC   USB0_CTL = USB_CTL_USBENSOFEN; // clear TXSUSPENDTOKENBUSY bit
       USB().ctl.ignoring_state().set_usbensofen(Usb_ctl_usbensofen::EnableUSBModule); // clear TXSUSPENDTOKENBUSY bit
    // CC   break;
      },
      0x01 | 0x02 => {
    // CC case 0x01:  // OUT transaction received from host
    // CC case 0x02:
    // CC   //serial_print("PID=OUT\n");
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
    // CC   // give the buffer back
    // CC   b->desc = BDT_DESC(EP0_SIZE, DATA1);
        b.control.ignoring_state().give_back(EP0_SIZE, usb::BufferDescriptor_control_data01::Data1);
    // CC   break;
      },
      0x09 => {
    // CC case 0x09: // IN transaction completed to host
    // CC   //serial_print("PID=IN:");
    // CC   //serial_phex(stat);
    // CC   //serial_print("\n");

    // CC   // send remaining data, if any...
    // CC   data = ep0_tx_ptr;
    // CC   if (data) {
        if self.ep0_tx_ptr.len() > 0 {
    // CC     size = ep0_tx_len;
    // CC     if (size > EP0_SIZE) size = EP0_SIZE;
          let chunksize = cmp::min(EP0_SIZE, self.ep0_tx_ptr.len());
    // CC     endpoint0_transmit(data, size);
          self.endpoint0_transmit(&self.ep0_tx_ptr[0..chunksize]);
    // CC     data += size;
    // CC     ep0_tx_len -= size;
    // CC     ep0_tx_ptr = (ep0_tx_len > 0 || size == EP0_SIZE) ? data : NULL;
          self.ep0_tx_ptr = &self.ep0_tx_ptr[chunksize..];
    // CC   }
        }

    // CC   if (setup.bRequest == 5 && setup.bmRequestType == 0) {
        if self.ep0_setuppacket.request_and_type() == 0x0500 { // SET_ADDRESS
    // CC     setup.bRequest = 0;
            self.ep0_setuppacket.clear_request();
    // CC     //serial_print("set address: ");
    // CC     //serial_phex16(setup.wValue);
    // CC     //serial_print("\n");
    // CC     USB0_ADDR = setup.wValue;
              USB().addr.set_addr(self.ep0_setuppacket.wValue() as u8);
    // CC   }
        }

    // CC   break;
      },
      _ => {
    // CC //default:
    // CC   //serial_print("PID=unknown:");
    // CC   //serial_phex(pid);
    // CC   //serial_print("\n");
      }
    // CC }
    }
    // CC USB0_CTL = USB_CTL_USBENSOFEN; // clear TXSUSPENDTOKENBUSY bit
    USB().ctl.ignoring_state().set_usbensofen(Usb_ctl_usbensofen::EnableUSBModule); // clear TXSUSPENDTOKENBUSY bit

  }

  pub fn usb_setup(&mut self) {
    // CC  const uint8_t *data = NULL;
    // CC  uint32_t datalen = 0;
    // CC  const usb_descriptor_list_t *list;
    // CC  uint32_t size;
    // CC  volatile uint8_t *reg;
    // CC  uint8_t epconf;
    // CC  const uint8_t *cfg;
    // CC  int i;
    // CC
    // CC  switch (setup.wRequestAndType) {
    match self.ep0_setuppacket.request_and_type() {
    // CC    case 0x0500: // SET_ADDRESS
    // CC    break;
      0x0500 => {}, // SET_ADDRESS
    // CC    case 0x0900: // SET_CONFIGURATION
      0x0900 => { // SET_CONFIGURATION
    // CC    //serial_print("configure\n");
    // CC    usb_configuration = setup.wValue;
        self.usb_configuration = self.ep0_setuppacket.wValue() as u8;
    // CC    reg = &USB0_ENDPT1;
    // CC    cfg = usb_endpoint_config_table;
    // CC    // clear all BDT entries, free any allocated memory...
    // CC    for (i=4; i < (NUM_ENDPOINTS+1)*4; i++) {
    // CC      if (table[i].desc & BDT_OWN) {
    // CC        usb_free((usb_packet_t *)((uint8_t *)(table[i].addr) - 8));
    // CC      }
    // CC    }
        // TODO
    // CC    // free all queued packets
    // CC    for (i=0; i < NUM_ENDPOINTS; i++) {
    // CC      usb_packet_t *p, *n;
    // CC      p = rx_first[i];
    // CC      while (p) {
    // CC        n = p->next;
    // CC        usb_free(p);
    // CC        p = n;
    // CC      }
    // CC      rx_first[i] = NULL;
    // CC      rx_last[i] = NULL;
    // CC      p = tx_first[i];
    // CC      while (p) {
    // CC        n = p->next;
    // CC        usb_free(p);
    // CC        p = n;
    // CC      }
    // CC      tx_first[i] = NULL;
    // CC      tx_last[i] = NULL;
    // CC      usb_rx_byte_count_data[i] = 0;
    // CC      switch (tx_state[i]) {
    // CC        case TX_STATE_EVEN_FREE:
    // CC        case TX_STATE_NONE_FREE_EVEN_FIRST:
    // CC        tx_state[i] = TX_STATE_BOTH_FREE_EVEN_FIRST;
    // CC        break;
    // CC        case TX_STATE_ODD_FREE:
    // CC        case TX_STATE_NONE_FREE_ODD_FIRST:
    // CC        tx_state[i] = TX_STATE_BOTH_FREE_ODD_FIRST;
    // CC        break;
    // CC        default:
    // CC        break;
    // CC      }
    // CC    }
    // CC    usb_rx_memory_needed = 0;
    // CC    for (i=1; i <= NUM_ENDPOINTS; i++) {
    // CC      epconf = *cfg++;
    // CC      *reg = epconf;
    // CC      reg += 4;
    // CC      if (epconf & USB_ENDPT_EPRXEN) {
    // CC        usb_packet_t *p;
    // CC        p = usb_malloc();
    // CC        if (p) {
    // CC          table[index(i, RX, EVEN)].addr = p->buf;
    // CC          table[index(i, RX, EVEN)].desc = BDT_DESC(64, 0);
    // CC        } else {
    // CC          table[index(i, RX, EVEN)].desc = 0;
    // CC          usb_rx_memory_needed++;
    // CC        }
    // CC        p = usb_malloc();
    // CC        if (p) {
    // CC          table[index(i, RX, ODD)].addr = p->buf;
    // CC          table[index(i, RX, ODD)].desc = BDT_DESC(64, 1);
    // CC        } else {
    // CC          table[index(i, RX, ODD)].desc = 0;
    // CC          usb_rx_memory_needed++;
    // CC        }
    // CC      }
    // CC      table[index(i, TX, EVEN)].desc = 0;
    // CC      table[index(i, TX, ODD)].desc = 0;
    // CC    }
    // CC    break;
      }
    // CC    case 0x0880: // GET_CONFIGURATION
    // CC    reply_buffer[0] = usb_configuration;
    // CC    datalen = 1;
    // CC    data = reply_buffer;
    // CC    break;
    // CC    case 0x0080: // GET_STATUS (device)
    // CC    reply_buffer[0] = 0;
    // CC    reply_buffer[1] = 0;
    // CC    datalen = 2;
    // CC    data = reply_buffer;
    // CC    break;
    // CC    case 0x0082: // GET_STATUS (endpoint)
    // CC    if (setup.wIndex > NUM_ENDPOINTS) {
    // CC      // TODO: do we need to handle IN vs OUT here?
    // CC      endpoint0_stall();
    // CC      return;
    // CC    }
    // CC    reply_buffer[0] = 0;
    // CC    reply_buffer[1] = 0;
    // CC    if (*(uint8_t *)(&USB0_ENDPT0 + setup.wIndex * 4) & 0x02) reply_buffer[0] = 1;
    // CC    data = reply_buffer;
    // CC    datalen = 2;
    // CC    break;
    // CC    case 0x0102: // CLEAR_FEATURE (endpoint)
    // CC    i = setup.wIndex & 0x7F;
    // CC    if (i > NUM_ENDPOINTS || setup.wValue != 0) {
    // CC      // TODO: do we need to handle IN vs OUT here?
    // CC      endpoint0_stall();
    // CC      return;
    // CC    }
    // CC    (*(uint8_t *)(&USB0_ENDPT0 + i * 4)) &= ~0x02;
    // CC    // TODO: do we need to clear the data toggle here?
    // CC    break;
    // CC    case 0x0302: // SET_FEATURE (endpoint)
    // CC    i = setup.wIndex & 0x7F;
    // CC    if (i > NUM_ENDPOINTS || setup.wValue != 0) {
    // CC      // TODO: do we need to handle IN vs OUT here?
    // CC      endpoint0_stall();
    // CC      return;
    // CC    }
    // CC    (*(uint8_t *)(&USB0_ENDPT0 + i * 4)) |= 0x02;
    // CC    // TODO: do we need to clear the data toggle here?
    // CC    break;
    // CC    case 0x0680: // GET_DESCRIPTOR
    // CC    case 0x0681:
    // CC    //serial_print("desc:");
    // CC    //serial_phex16(setup.wValue);
    // CC    //serial_print("\n");
    // CC    for (list = usb_descriptor_list; 1; list++) {
    // CC      if (list->addr == NULL) break;
    // CC      //if (setup.wValue == list->wValue &&
    // CC      //(setup.wIndex == list->wIndex) || ((setup.wValue >> 8) == 3)) {
    // CC      if (setup.wValue == list->wValue && setup.wIndex == list->wIndex) {
    // CC        data = list->addr;
    // CC        if ((setup.wValue >> 8) == 3) {
    // CC          // for string descriptors, use the descriptor's
    // CC          // length field, allowing runtime configured
    // CC          // length.
    // CC          datalen = *(list->addr);
    // CC        } else {
    // CC          datalen = list->length;
    // CC        }
    // CC#if 0
    // CC        serial_print("Desc found, ");
    // CC        serial_phex32((uint32_t)data);
    // CC        serial_print(",");
    // CC        serial_phex16(datalen);
    // CC        serial_print(",");
    // CC        serial_phex(data[0]);
    // CC        serial_phex(data[1]);
    // CC        serial_phex(data[2]);
    // CC        serial_phex(data[3]);
    // CC        serial_phex(data[4]);
    // CC        serial_phex(data[5]);
    // CC        serial_print("\n");
    // CC#endif
    // CC        goto send;
    // CC      }
    // CC    }
    // CC    //serial_print("desc: not found\n");
    // CC    endpoint0_stall();
    // CC    return;
    // CC
    // CC    default:
      _ => {
    // CC    endpoint0_stall();
    // CC    return;
      }
    // CC  }
    } // match req and type
    // CC  send:
    // CC  //serial_print("setup send ");
    // CC  //serial_phex32(data);
    // CC  //serial_print(",");
    // CC  //serial_phex16(datalen);
    // CC  //serial_print("\n");
    // CC
    // CC  if (datalen > setup.wLength) datalen = setup.wLength;
    // CC  size = datalen;
    // CC  if (size > EP0_SIZE) size = EP0_SIZE;
    // CC  endpoint0_transmit(data, size);
    // CC  data += size;
    // CC  datalen -= size;
    // CC  if (datalen == 0 && size < EP0_SIZE) return;
    // CC
    // CC  size = datalen;
    // CC  if (size > EP0_SIZE) size = EP0_SIZE;
    // CC  endpoint0_transmit(data, size);
    // CC  data += size;
    // CC  datalen -= size;
    // CC  if (datalen == 0 && size < EP0_SIZE) return;
    // CC
    // CC  ep0_tx_ptr = data;
    // CC  ep0_tx_len = datalen;
  }

  pub fn endpoint0_transmit(&mut self, buf : &[u8]) {
    // CC#if 0
    // CC  serial_print("tx0:");
    // CC  serial_phex32((uint32_t)data);
    // CC  serial_print(",");
    // CC  serial_phex16(len);
    // CC  serial_print(ep0_tx_bdt_bank ? ", odd" : ", even");
    // CC  serial_print(ep0_tx_data_toggle ? ", d1\n" : ", d0\n");
    // CC#endif
    // CC  table[index(0, TX, ep0_tx_bdt_bank)].addr = (void *)data;
    // CC  table[index(0, TX, ep0_tx_bdt_bank)].desc = BDT_DESC(len, ep0_tx_data_toggle);
    // CC  ep0_tx_data_toggle ^= 1;
    // CC  ep0_tx_bdt_bank ^= 1;
    // CC
  }

}

enum FlashCommands {
  ReadFromIFR{ record_index : u8 }
}

enum FlashCommandReturns {
  FourBytes { data : [u8 ; 4] }
}

impl FlashCommands {

  fn execute_flash_command(command : FlashCommands) -> FlashCommandReturns {
    // pp 573, section 28.3.4 Register Descriptions
    // pp 591, section 28.4.9 Flash Command Operatrions
    // pp 613, section 28.4.11.10 Read Once Command
    // 12 bytes of memory for input and output

    // CC    __disable_irq();
    let _guard = NoInterrupts::new();
    while Ftfl().stat.ccif().eq(&Ftfl_stat_ccif::InProgress) {}
    // CC    FTFL_FSTAT = FTFL_FSTAT_RDCOLERR | FTFL_FSTAT_ACCERR | FTFL_FSTAT_FPVIOL;
    Ftfl().stat.ignoring_state().clear_rdcolerr().clear_accerr().clear_fpviol();
    let ret = match &command {
      &FlashCommands::ReadFromIFR { record_index : record_index } => {
        // CC    FTFL_FCCOB0 = 0x41;
        Ftfl().fccob0.ignoring_state().set_ccob0(0x41);
        // CC    FTFL_FCCOB1 = 15;
        Ftfl().fccob1.ignoring_state().set_ccob1(record_index);
      }
    };
    // CC    FTFL_FSTAT = FTFL_FSTAT_CCIF;
    Ftfl().stat.ignoring_state().clear_ccif();
    // CC    while (!(FTFL_FSTAT & FTFL_FSTAT_CCIF)) ; // wait
    while Ftfl().stat.ccif().eq(&Ftfl_stat_ccif::InProgress) {}
    // CC    num = *(uint32_t *)&FTFL_FCCOB7;
    match &command {
      &FlashCommands::ReadFromIFR { record_index : _ } => {
        let mut data = [0u8; 4];
        data[0] = Ftfl().fccob4.ccob4();
        data[1] = Ftfl().fccob5.ccob5();
        data[2] = Ftfl().fccob6.ccob6();
        data[3] = Ftfl().fccob7.ccob7();
        FlashCommandReturns::FourBytes { data : data }
      }
    }
    // CC    __enable_irq();
  }
}








#[allow(dead_code)]
#[no_mangle]
pub unsafe extern fn isr_usb() {

//   uint8_t status, stat, t;

//   //serial_print("isr");
//   //status = USB0_ISTAT;
//   //serial_phex(status);
//   //serial_print("\n");
//   restart:
//   status = USB0_ISTAT;
  let status = USB().istat.get();



//   if ((status & USB_ISTAT_SOFTOK /* 04 */ )) { // *got start of frame
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
//     USB0_ISTAT = USB_ISTAT_SOFTOK; // write one to the interrupt flag to clear (ignore the other flags!!)
       USB().istat.ignoring_state().clear_softok();
//   }
     }

//   if ((status & USB_ISTAT_TOKDNE /* 08 */ )) {
     if status.tokdne() {
  // This bit is set when the current token being processed has completed. The processor must immediately
  //read the STATUS (STAT) register to determine the EndPoint and BD used for this token. Clearing this bit
  //(by writing a one) causes STAT to be cleared or the STAT holding register to be loaded into the STAT
  //register.
//     uint8_t endpoint;
//     stat = USB0_STAT;
       let last_transaction = USB().stat.get();
//     //serial_print("token: ep=");
//     //serial_phex(stat >> 4);
//     //serial_print(stat & 0x08 ? ",tx" : ",rx");
//     //serial_print(stat & 0x04 ? ",odd\n" : ",even\n");
//     endpoint = stat >> 4;
//     if (endpoint == 0) {
       if last_transaction.endp() == 0 {
//       usb_control(stat);
         usbdriver.as_mut().unwrap().usb_control(last_transaction);
//     } else {
       } else {
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

//     }
//     USB0_ISTAT = USB_ISTAT_TOKDNE;
       USB().istat.ignoring_state().clear_tokdne();

//     goto restart;
//   }
     }


//   if (status & USB_ISTAT_USBRST /* 01 */ ) {
     if status.usbrst() {
//     //serial_print("reset\n");

//     // initialize BDT toggle bits
//     USB0_CTL = USB_CTL_ODDRST;
       USB().ctl.ignoring_state().set_oddrst(true);
//     ep0_tx_bdt_bank = 0;

//     // set up buffers to receive Setup and OUT packets
//     table[index(0, RX, EVEN)].desc = BDT_DESC(EP0_SIZE, 0);
//     table[index(0, RX, EVEN)].addr = ep0_rx0_buf;
//     table[index(0, RX, ODD)].desc = BDT_DESC(EP0_SIZE, 0);
//     table[index(0, RX, ODD)].addr = ep0_rx1_buf;
//     table[index(0, TX, EVEN)].desc = 0;
//     table[index(0, TX, ODD)].desc = 0;

//     // activate endpoint 0
//     USB0_ENDPT0 = USB_ENDPT_EPRXEN | USB_ENDPT_EPTXEN | USB_ENDPT_EPHSHK;
       USB().endpt[0].endpt.set_eprxen(Usb_endpt_endpt_eprxen::RxEnabled)
                           .set_eptxen(Usb_endpt_endpt_eptxen::TxEnabled)
                           .set_ephshk(Usb_endpt_endpt_ephshk::Handshake);

//     // clear all ending interrupts
//     USB0_ERRSTAT = 0xFF;
       USB().errstat.ignoring_state().clear_all();
//     USB0_ISTAT = 0xFF;
       USB().istat.ignoring_state().clear_all();

//     // set the address to zero during enumeration
//     USB0_ADDR = 0;
       USB().addr.set_addr(0);

//     // enable other interrupts
//     USB0_ERREN = 0xFF;
       USB().erren.ignoring_state().enable_all();
//     USB0_INTEN = USB_INTEN_TOKDNEEN |
//       USB_INTEN_SOFTOKEN |
//       USB_INTEN_STALLEN |
//       USB_INTEN_ERROREN |
//       USB_INTEN_USBRSTEN |
//       USB_INTEN_SLEEPEN;
       USB().inten.ignoring_state().set_tokdneen(Usb_inten_tokdneen::InterruptEnabled)
                                   .set_softoken(Usb_inten_softoken::InterruptEnabled)
                                   .set_stallen(Usb_inten_stallen::InterruptEnabled)
                                   .set_erroren(Usb_inten_erroren::InterruptEnabled)
                                   .set_usbrsten(Usb_inten_usbrsten::InterruptEnabled)
                                   .set_sleepen(Usb_inten_sleepen::InterruptEnabled)
                                   .set_resumeen(Usb_inten_resumeen::InterruptDisabled) // disabled
                                   .set_attachen(Usb_inten_attachen::InterruptDisabled) ;

//     // is this necessary? //*yes because we disabled usb above
//     USB0_CTL = USB_CTL_USBENSOFEN;
       // USB Enable Setting this bit causes the SIE to reset all of its ODD bits to the BDTs.
       // 1 Enables the USB Module.
       USB().ctl.ignoring_state().set_usbensofen(Usb_ctl_usbensofen::EnableUSBModule);
//     return;
       return;
//   }
     }


//   if ((status & USB_ISTAT_STALL /* 80 */ )) {
     if status.stall() {
//     //serial_print("stall:\n");
//     USB0_ENDPT0 = USB_ENDPT_EPRXEN | USB_ENDPT_EPTXEN | USB_ENDPT_EPHSHK;
       USB().endpt[0].endpt.set_eprxen(Usb_endpt_endpt_eprxen::RxEnabled)
                           .set_eptxen(Usb_endpt_endpt_eptxen::TxEnabled)
                           .set_ephshk(Usb_endpt_endpt_ephshk::Handshake);
//     USB0_ISTAT = USB_ISTAT_STALL;
       USB().istat.ignoring_state().clear_stall();
//   }
     }
//   if ((status & USB_ISTAT_ERROR /* 02 */ )) {
     if status.error() {
       let err = USB().errstat.get().raw();
       USB().errstat.ignoring_state().clear_raw(err);
//     uint8_t err = USB0_ERRSTAT;
//     USB0_ERRSTAT = err;
//     //serial_print("err:");
//     //serial_phex(err);
//     //serial_print("\n");
//     USB0_ISTAT = USB_ISTAT_ERROR;
       USB().istat.ignoring_state().clear_error();
//   }
     }

//   if ((status & USB_ISTAT_SLEEP /* 10 */ )) {
     if status.sleep() {
//     //serial_print("sleep\n");
//     USB0_ISTAT = USB_ISTAT_SLEEP;
       USB().istat.ignoring_state().clear_sleep();
//   }
     }

}


