#![feature(plugin, start, core_intrinsics)]
#![no_std]

#[macro_use]
pub extern crate zinc;

pub mod generated;
#[macro_use]
pub mod usbmem;
// mod test;


use core::option::Option::Some;
use core::mem;

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

  loop {
    led1.set_high();
    wait(500);
    led1.set_low();
    wait(500);
  }

}

use usbmem::{Fifos, Ep};
use core::mem::drop;

fn test_fifo_2() {

 {
  let pool = fifos!(0, packetbufsize: 0);
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




struct UsbDriver {
    /// Selected configuration id (by host via SET_CONFIGURATION)
    usb_configuration : u8,
    /// ?
    // extern uint16_t usb_rx_byte_count_data[NUM_ENDPOINTS];
    usb_rx_byte_count_data : [u16 ; 2], // [;NUM_ENDPOINTS]

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















pub fn usb_control(last_transaction : Usb_stat_Get) {

//   bdt_t *b;
//   uint32_t pid, size;
//   uint8_t *buf;
//   const uint8_t *data;

//   b = stat2bufferdescriptor(stat);
//   pid = BDT_PID(b->desc);
//   //count = b->desc >> 16;
//   buf = b->addr;
//   //serial_print("pid:");
//   //serial_phex(pid);
//   //serial_print(", count:");
//   //serial_phex(count);
//   //serial_print("\n");

//   switch (pid) {
//   case 0x0D: // Setup received from host
//     //serial_print("PID=Setup\n");
//     //if (count != 8) ; // panic?
//     // grab the 8 byte setup info
//     setup.word1 = *(uint32_t *)(buf);
//     setup.word2 = *(uint32_t *)(buf + 4);

//     // give the buffer back
//     b->desc = BDT_DESC(EP0_SIZE, DATA1);
//     //table[index(0, RX, EVEN)].desc = BDT_DESC(EP0_SIZE, 1);
//     //table[index(0, RX, ODD)].desc = BDT_DESC(EP0_SIZE, 1);

//     // clear any leftover pending IN transactions
//     ep0_tx_ptr = NULL;
//     if (ep0_tx_data_toggle) {
//     }
//     //if (table[index(0, TX, EVEN)].desc & 0x80) {
//       //serial_print("leftover tx even\n");
//     //}
//     //if (table[index(0, TX, ODD)].desc & 0x80) {
//       //serial_print("leftover tx odd\n");
//     //}
//     table[index(0, TX, EVEN)].desc = 0;
//     table[index(0, TX, ODD)].desc = 0;
//     // first IN after Setup is always DATA1
//     ep0_tx_data_toggle = 1;

// #if 0
//     serial_print("bmRequestType:");
//     serial_phex(setup.bmRequestType);
//     serial_print(", bRequest:");
//     serial_phex(setup.bRequest);
//     serial_print(", wValue:");
//     serial_phex16(setup.wValue);
//     serial_print(", wIndex:");
//     serial_phex16(setup.wIndex);
//     serial_print(", len:");
//     serial_phex16(setup.wLength);
//     serial_print("\n");
// #endif
//     // actually "do" the setup request
//     usb_setup();
//     // unfreeze the USB, now that we're ready
//     USB0_CTL = USB_CTL_USBENSOFEN; // clear TXSUSPENDTOKENBUSY bit
//     break;
//   case 0x01:  // OUT transaction received from host
//   case 0x02:
//     //serial_print("PID=OUT\n");
// #ifdef CDC_STATUS_INTERFACE
//     if (setup.wRequestAndType == 0x2021 /*CDC_SET_LINE_CODING*/) {
//       int i;
//       uint8_t *dst = (uint8_t *)usb_cdc_line_coding;
//       //serial_print("set line coding ");
//       for (i=0; i<7; i++) {
//         //serial_phex(*buf);
//         *dst++ = *buf++;
//       }
//       //serial_phex32(usb_cdc_line_coding[0]);
//       //serial_print("\n");
//       if (usb_cdc_line_coding[0] == 134) usb_reboot_timer = 15;
//       endpoint0_transmit(NULL, 0);
//     }
// #endif
//     // give the buffer back
//     b->desc = BDT_DESC(EP0_SIZE, DATA1);
//     break;

//   case 0x09: // IN transaction completed to host
//     //serial_print("PID=IN:");
//     //serial_phex(stat);
//     //serial_print("\n");

//     // send remaining data, if any...
//     data = ep0_tx_ptr;
//     if (data) {
//       size = ep0_tx_len;
//       if (size > EP0_SIZE) size = EP0_SIZE;
//       endpoint0_transmit(data, size);
//       data += size;
//       ep0_tx_len -= size;
//       ep0_tx_ptr = (ep0_tx_len > 0 || size == EP0_SIZE) ? data : NULL;
//     }

//     if (setup.bRequest == 5 && setup.bmRequestType == 0) {
//       setup.bRequest = 0;
//       //serial_print("set address: ");
//       //serial_phex16(setup.wValue);
//       //serial_print("\n");
//       USB0_ADDR = setup.wValue;
//     }

//     break;
//   //default:
//     //serial_print("PID=unknown:");
//     //serial_phex(pid);
//     //serial_print("\n");
//   }
//   USB0_CTL = USB_CTL_USBENSOFEN; // clear TXSUSPENDTOKENBUSY bit

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
         usb_control(last_transaction);
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


