#![feature(plugin, start, core_intrinsics)]
#![feature(const_fn, drop_types_in_const, linkage)]
#![plugin(ioreg)]
#![no_std]

#[macro_use] pub extern crate zinc;
#[macro_use] #[no_link] extern crate ioreg;
#[macro_use] extern crate volatile_cell;

pub mod generated;
//pub use generated2 as generated;
pub mod usb;
pub mod usbmempool;
#[macro_use]
pub mod usbmem;
pub mod usbdriver;
pub mod usbserial;
// mod test;

pub use core as c;

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

use usbmempool::{MemoryPool, AllocatedUsbPacket, UsbPacket, MemoryPoolTrait};
use usbmem::{Fifos, Ep};

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

static mut POOL : Option<MemoryPool<[UsbPacket; 32]>> = None;
pub fn pool_ref() -> &'static MemoryPool<[UsbPacket; 32]> {
    let r = unsafe { &mut POOL };
    if r.is_none() {
        *r = Some(MemoryPool::new());
    }
    &r.as_ref().unwrap()
}



pub fn main() {
  uart_logger::init( Uart::new(UARTPeripheral::UART0, 38400, 8, Parity::Disabled, 1,
    pin::Pin::new(pin::Port::PortB, 16, pin::Function::Gpio, Some(zinc::hal::pin::Out)),
    pin::Pin::new(pin::Port::PortB, 17, pin::Function::Gpio, Some(zinc::hal::pin::Out))) );

  let mut uartref = unsafe { &mut uart_logger::LOGGING_UART }.unwrap();

  // Pins for MC HCK (http://www.mchck.org/)
  let led1 = pin::Pin::new(pin::Port::PortC, 5, pin::Function::Gpio, Some(zinc::hal::pin::Out));
  led1.set_high();

  info!("STARTING UP");
  wait(500);


  let pool = pool_ref(); // init
  generated::usb_ref(); // init


  // let usb1 = USB().clone();


 // test_fifo_2();
 // test_fifo();

  {
     let p = pool.allocate().unwrap();
     use usbmempool::{BufferPointerMagic};
     let ptr = unsafe { p.into_buf_ptr() };
     ///unsafe { POOL = MemoryPoolOption::none() }
     //info!("Killing");
     let recovered = unsafe { AllocatedUsbPacket::from_raw_buf_ptr(ptr) };
     pool.free(recovered);
  }


  //let usb = UsbDriver::new();

    wait(7000);
  // let usb2 = USB().clone();
  // info!("#######################################################################");
  // info!("{:?}", usb1);
  // info!("#######################################################################");
  // info!("{:?}", usb2);
  // info!("#######################################################################");
  // info!("{:?}", generated::BufferDescriptors());

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
  let f = Fifos::new();
  let pool = pool_ref();

  let mut p = pool.allocate().unwrap();
  p.buf_mut(30).iter_mut().enumerate().map(|(i,b)| *b = i as u8).count();
  f.enqueue(Ep::Tx8, p);

  let pr = f.dequeue(Ep::Tx8).unwrap();
  assert!(pr.buf()[4] == 4);
  assert!(f.dequeue(Ep::Tx8).is_none(), "Can dequeue from empty queue.");
  pool.free(pr);

  // fill
  for i in 0..16 {
    let mut p = pool.allocate().unwrap();
    assert_eq!(p.buf().len(), 0);
    assert_eq!(i, p.buf_mut(i as u16).iter().count());
    f.enqueue(Ep::Tx3, p);
    assert_eq!(f.len(Ep::Tx3), i + 1);
  }

  assert_eq!(pool.available(), 16);

  // fill more
  for i in 16..32 {
    let mut p = pool.allocate().unwrap();
    p.set_index(i);
    f.enqueue(Ep::Tx6, p);
  }

  assert_eq!(pool.available(), 0);
  assert!(pool.allocate().is_none(), "Can allocate more than 32 packets");

  // Test order
  assert_eq!(f.len(Ep::Tx3), 16);
  for i in 0..10 {
    let re = f.dequeue(Ep::Tx3).expect("Can't dequeue from EP3");
    assert_eq!(re.buf().len(), i);
    re.recycle(&pool);
  }
  for i in 16..30 {
    let re = f.dequeue(Ep::Tx6).expect("Cant dequeue from EP6");
    assert_eq!(re.index(), i);
    pool.free(re);
  }

  assert_eq!(pool.available(), 24);

  assert_eq!(f.len(Ep::Tx6), 2);
  assert_eq!(f.len(Ep::Tx3), 6);
  f.clear(Ep::Tx3, &pool);
  assert_eq!(f.len(Ep::Tx3), 0);
  f.clear_all(&pool);
  assert_eq!(f.len(Ep::Tx6), 0);
  assert_eq!(pool.available(), 32);

  // test priority
  for i in 0..32 {
    let mut p = pool.allocate().unwrap();
    p.set_index(i);
    f.enqueue(Ep::Tx15, p);
  }
  assert_eq!(f.len(Ep::Tx15), 32);
  assert_eq!(pool.available(), 0);

  assert!(pool.allocate().is_none(), "Can allocate more than 32 packets");
  unsafe { assert_eq!(SERVED_PRIO, 0);}
  pool.allocate_priority();
  pool.allocate_priority();
  unsafe { assert_eq!(SERVED_PRIO, 0);}
  f.dequeue(Ep::Tx15).unwrap().recycle(&pool);
  unsafe { assert_eq!(SERVED_PRIO, 1);}
  f.dequeue(Ep::Tx15).unwrap().recycle(&pool);
  unsafe { assert_eq!(SERVED_PRIO, 2);}
  f.dequeue(Ep::Tx15).unwrap().recycle(&pool);
  unsafe { assert_eq!(SERVED_PRIO, 2);}
  f.clear_all(&pool);
  assert_eq!(pool.available(), 30);

  info!(target: "usbmem", "FIFO test successfull");
}

static mut SERVED_PRIO : usize = 0;

#[link_name="handle_priority_allocation"]
#[no_mangle]
fn handle_priority_allocation(p : AllocatedUsbPacket) -> Option<AllocatedUsbPacket> {
    use usbmempool::BufferPointerMagic;
    unsafe {
      p.into_buf_ptr();
      SERVED_PRIO += 1;
    }
    None
}

