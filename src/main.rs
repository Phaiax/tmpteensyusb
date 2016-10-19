#![feature(plugin, start, core_intrinsics)]
#![feature(const_fn, drop_types_in_const)]
#![no_std]

#[macro_use]
pub extern crate zinc;

pub mod generated;
#[macro_use]
pub mod usb;
pub mod usbmempool;
pub mod usbmem;
pub mod usbdriver;
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

use zinc::drivers::chario::CharIO;
use core::fmt::Write;

use usbdriver::UsbDriver;
use usbmempool::{MemoryPoolRef, MemoryPoolOption, AllocatedUsbPacket, UsbPacket};

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


static mut POOL : MemoryPoolOption<[UsbPacket; 32]> = MemoryPoolOption::none();

fn pool_ref() -> MemoryPoolRef<[UsbPacket; 32]> {
  unsafe { POOL.unwrap() }
}

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

  unsafe { POOL.init(); };
  let pool = pool_ref();

  {
     let p = pool.allocate().unwrap();
     use usbmempool::{BufferPointerMagic};
     let ptr = unsafe { p.into_buf_ptr() };
     ///unsafe { POOL = MemoryPoolOption::none() }
     let recovered = unsafe { AllocatedUsbPacket::from_raw_buf_ptr(ptr) };
     pool.free(recovered);
  }


  //let usb = UsbDriver::new();


  loop {
    led1.set_high();
    wait(500);
    led1.set_low();
    wait(500);
  }

}


fn test_fifo_2() {

 {
  //let pool = fifos!(0);
  //info!("Pool is {} bytes", core::mem::size_of_val(&pool));
  //let pool = fifos!();
  //info!("Pool is {} bytes", core::mem::size_of_val(&pool));

 }

/*  // Create static pool. Must not be dropped as long as usb is active. Max 32 elements.
  let pool = fifos!(); // Defaults to fifos!(32, packetbufsize: 64)
  // let pool = fifos!(10); // Changes the memory pool size from 32 to 10 packets
  //let pool = fifos!(10, packetbufsize: 30); // Changes the buf size of a packet to 30 bytes.


  // Enqueue into FIFO. Use endpoint 3

  // max 1 helper for each endpoint at a time
  let mut enq_helper_ep3 = pool.for_enqueuing(Ep::Tx3).unwrap();
  let mut later = Later { packet: None };
  {
    let mut packet1 = enq_helper_ep3.enqueue().unwrap(); // allocate
    packet1.len = 1;
    packet1.buf[0] = b'a'; // buf is uninitialized! Buf defaults to 64 bytes.
    // The DROP does the enqueue
  }
  {
    let mut packet1 = enq_helper_ep3.enqueue().unwrap(); // allocate
    packet1.len = 1;
    packet1.buf[0] = b'a'; // buf is uninitialized! Buf defaults to 64 bytes.
    later.packet = Some(packet1);
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


  {
    later.packet.as_mut().unwrap().len = 2;
    later.packet.as_mut().unwrap().buf[1] = 13;
  }

  fn work(data : &[u8]) {}
*/

}
/*
struct Later<'a> {
  packet : Option<ForEnqueue<'a, UsbPacket<'a>>>,
}
*/
fn test_fifo() {
  //let f = fifos!(32);

  /*let mut enq1 = f.for_enqueuing(Ep::Tx1).expect("Cant get enqueuer for EP 1");
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
  info!(target: "usbmem", "FIFO test successfull");*/
}

