#![feature(plugin, start, core_intrinsics)]
#![feature(const_fn, drop_types_in_const, linkage)]
#![plugin(ioreg)]
#![no_std]

#[macro_use] pub extern crate zinc;
#[macro_use] #[no_link] extern crate ioreg;
#[macro_use] extern crate volatile_cell;
extern crate core_io;

pub mod generated;
//pub use generated2 as generated;
pub mod usb;
pub mod usbmempool;
#[macro_use]
pub mod usbmem;
pub mod usbenum;
pub mod usbdriver;
pub mod usbserial;
// mod test;

pub use core as c;

use core::option::Option::Some;
use core::mem;
use core::cmp;
use core::cell::UnsafeCell;

use zinc::hal::cortex_m4::systick;
use zinc::hal::k20::{pin, watchdog};
use zinc::hal::pin::Gpio;
use zinc::hal::k20::regs::*;
use zinc::hal::k20::uart_logger;
use zinc::hal::k20::uart::{Uart, UARTPeripheral};
use zinc::hal::uart::Parity;

use zinc::drivers::chario::CharIO;
use core::fmt::Write;

use usbmempool::{MemoryPool, AllocatedUsbPacket, UsbPacket, MemoryPoolTrait, HandlePriorityAllocation, BufferPointerMagic};
use usbmem::{Fifos};
use usbenum::{EndpointWithDir, Direction};

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

pub const ENDPOINTCONFIG_FOR_REGISTERS: &'static [Usb_endpt_endpt] = &[
         Usb_endpt_endpt::from_raw(0x00),
         Usb_endpt_endpt::from_raw(0x00),
         Usb_endpt_endpt::from_raw(0x19),
         Usb_endpt_endpt::from_raw(0x15),
         Usb_endpt_endpt::from_raw(0x19),
         Usb_endpt_endpt::from_raw(0x00),
         Usb_endpt_endpt::from_raw(0x00),
         Usb_endpt_endpt::from_raw(0x00),
         Usb_endpt_endpt::from_raw(0x00),
         Usb_endpt_endpt::from_raw(0x00),
         Usb_endpt_endpt::from_raw(0x00),
         Usb_endpt_endpt::from_raw(0x00),
         Usb_endpt_endpt::from_raw(0x00),
         Usb_endpt_endpt::from_raw(0x00),
         Usb_endpt_endpt::from_raw(0x00),
         Usb_endpt_endpt::from_raw(0x00),
              ];


static mut POOL : Option<MemoryPool<[UsbPacket; 32]>> = None;
pub fn pool_ref() -> &'static MemoryPool<[UsbPacket; 32]> {
    let r = unsafe { &mut POOL };
    if r.is_none() {
        *r = Some(MemoryPool::new());
    }
    &r.as_ref().unwrap()
}

struct PrioHandler(UnsafeCell<usize>);



impl HandlePriorityAllocation for PrioHandler {
    /// Just now there are some new free packets
    fn handle_priority_allocation(&self, packet : AllocatedUsbPacket) -> Option<AllocatedUsbPacket> {
        unsafe { packet.into_buf_ptr() };
        unsafe { *self.0.get() += 1 };
        None
    }
}

static mut PRIOHANDLER : Option<PrioHandler> = None;
fn prio_handler_ref() -> &'static PrioHandler {
    let r = unsafe { &mut PRIOHANDLER };
    if r.is_none() {
        *r = Some(PrioHandler(UnsafeCell::new(0)));
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
     let recovered = unsafe { AllocatedUsbPacket::from_raw_buf_ptr(ptr, 2) };
     pool.free(recovered, prio_handler_ref());
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
  let mut enq_helper_ep3 = pool.for_enqueuing(ep3tx).unwrap();
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
  let mut deq_helper_ep3 = pool.for_dequeuing(ep3tx).unwrap();
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
  let prio_handler = prio_handler_ref();

  let ep3tx = EndpointWithDir::new(3, Direction::Tx);
  let ep6tx = EndpointWithDir::new(6, Direction::Tx);
  let ep8tx = EndpointWithDir::new(8, Direction::Tx);
  let ep13tx = EndpointWithDir::new(13, Direction::Tx);
  let ep15tx = EndpointWithDir::new(15, Direction::Tx);

  let mut p = pool.allocate().unwrap();
  p.buf_mut(30).iter_mut().enumerate().map(|(i,b)| *b = i as u8).count();
  f.enqueue(ep8tx, p);

  let pr = f.dequeue(ep8tx).unwrap();
  assert!(pr.buf()[4] == 4);
  assert!(f.dequeue(ep8tx).is_none(), "Can dequeue from empty queue.");
  pool.free(pr, prio_handler);

  // fill
  for i in 0..16 {
    let mut p = pool.allocate().unwrap();
    assert_eq!(p.buf().len(), 0);
    assert_eq!(i, p.buf_mut(i as u16).iter().count());
    f.enqueue(ep3tx, p);
    assert_eq!(f.len(ep3tx), i + 1);
  }

  assert_eq!(pool.available(), 16);

  // fill more
  for i in 16..32 {
    let mut p = pool.allocate().unwrap();
    p.set_index(i);
    f.enqueue(ep6tx, p);
  }

  assert_eq!(pool.available(), 0);
  assert!(pool.allocate().is_none(), "Can allocate more than 32 packets");

  // Test order
  assert_eq!(f.len(ep3tx), 16);
  for i in 0..10 {
    let re = f.dequeue(ep3tx).expect("Can't dequeue from EP3");
    assert_eq!(re.buf().len(), i);
    re.recycle(&pool, prio_handler);
  }
  for i in 16..30 {
    let re = f.dequeue(ep6tx).expect("Cant dequeue from EP6");
    assert_eq!(re.index(), i);
    pool.free(re, prio_handler);
  }

  assert_eq!(pool.available(), 24);

  assert_eq!(f.len(ep6tx), 2);
  assert_eq!(f.len(ep3tx), 6);
  f.clear(ep3tx, &pool, prio_handler);
  assert_eq!(f.len(ep3tx), 0);
  f.clear_all(&pool, prio_handler);
  assert_eq!(f.len(ep6tx), 0);
  assert_eq!(pool.available(), 32);

  // test priority
  for i in 0..32 {
    let mut p = pool.allocate().unwrap();
    p.set_index(i);
    f.enqueue(ep15tx, p);
  }
  assert_eq!(f.len(ep15tx), 32);
  assert_eq!(pool.available(), 0);

  assert!(pool.allocate().is_none(), "Can allocate more than 32 packets");
  unsafe { assert_eq!(*prio_handler.0.get(), 0);}
  pool.allocate_priority();
  pool.allocate_priority();
  unsafe { assert_eq!(*prio_handler.0.get(), 0);}
  f.dequeue(ep15tx).unwrap().recycle(&pool, prio_handler);
  unsafe { assert_eq!(*prio_handler.0.get(), 1);}
  f.dequeue(ep15tx).unwrap().recycle(&pool, prio_handler);
  unsafe { assert_eq!(*prio_handler.0.get(), 2);}
  f.dequeue(ep15tx).unwrap().recycle(&pool, prio_handler);
  unsafe { assert_eq!(*prio_handler.0.get(), 2);}
  f.clear_all(&pool, prio_handler);
  assert_eq!(pool.available(), 30);

  info!(target: "usbmem", "FIFO test successfull");
}



