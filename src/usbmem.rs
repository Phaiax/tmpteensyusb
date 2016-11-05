//! An endpoint grouped FIFO memory pool for usb packets.
//!
//! There is one FIFO for each direction of each of the 15 endpoints.
//! Endpoint 0 will not use this memory pool.
//!
//! ## Create
//! Create static pool. Must not be dropped as long as usb is active.
//!
//! ```
//!     let pool = fifos!(); // = fifos!(32, packetbufsize: 64)
//! ```
//!
//! Change the memory pool size from 32 to 10 packets (Max 32 elements allowed):
//!
//! ```
//!     let pool = fifos!(10);
//! ```
//!
//! Change the buf size of a packet to 30 bytes.
//!
//! ```
//!     let pool = fifos!(10, packetbufsize: 30);
//! ```
//!
//! ## Enqueue into FIFO of endpoint 3
//! ```
//!  // max 1 helper for each endpoint and direction at a time
//!  let mut enq_helper_ep3 = pool.for_enqueuing(Ep::Tx3).unwrap();
//!  {
//!    let mut packet1 = enq_helper_ep3.enqueue().unwrap(); // allocate
//!    packet1.len = 1;
//!    // buf is uninitialized! Buf defaults to 64 bytes.
//!    packet1.buf[0] = b'a';
//!    // The DROP does the enqueuing
//!  }
//!  // enq_helper_ep3 can be reused as many times as needed.
//! ```
//!
//! ## Dequeue from FIFO of endpoint 3
//! ```
//!  // max 1 helper for each endpoint and direction at a time
//!  let mut deq_helper_ep3 = pool.for_dequeuing(Ep::Tx3).unwrap();
//!  {
//!    let packet1 = deq_helper_ep3.dequeue().unwrap(); // dequeue
//!    info!("Got {} bytes", packet1.len);
//!    // packet1.buf[packet1.len..] is garbage!
//!    work(packet1.buf());
//!    // The DROP does the deallocation
//!  }
//!  // deq_helper_ep3 can be reused as many times as needed.
//!
//!  fn work(data : &[u8]) {}
//! ```
//!
//! ## Memory usage
//!
//! The overhead for the FIFO pointers and the pool allocation is 276 bytes
//!
//! The default queue `fifos!()` uses 2580 bytes.
//!
//! A `fifos(aaa, packetbufsize: bbb)` uses `276+(bbb+8)*aaa` bytes. (Includes two alignment bytes).
//!
//! ```
//!   memory : MemoryPool,            | 4+(bbb+2+2+4)*aaa bytes
//!       available : u32,            |   4 bytes
//!       pool : [__; aaa],           |   (4+2+2+bbb)*aaa bytes
//!          next : *mut              |     4 bytes
//!          packet : UsbPacket       |
//!             index : u16           |     2 bytes
//!             len : u16             |     2 bytes
//!             buf : [u8; bbb]       |     bbb bytes
//!   fifos : [__; 30]                | (4+4+1)*30=270 bytes
//!       first : *mut T,             |     4 bytes
//!       last : *mut T,              |     4 bytes
//!       borrow_state : BorrowState, |     1 byte
//! ```
//!
//!
use core::cell::UnsafeCell;
use core::mem;
use core::ptr;

use zinc::hal::cortex_m4::irq::NoInterrupts;
use usbmempool::{UsbPacket, AllocatedUsbPacket, MemoryPoolTrait};

pub trait StoreNext {
    unsafe fn set_next(&mut self, next : AllocatedUsbPacket);
}

pub trait RetrieveNext {
    unsafe fn take_next(&mut self) -> Option<AllocatedUsbPacket>;
    unsafe fn count_queue(&self) -> usize;
    unsafe fn ptr_inner(&mut self) -> *mut UsbPacket;
}


//#[macro_export]
/// See `usbmem` module configuration for examples.
//macro_rules! fifos {
//    () => (
//        fifos!(32)
//    );
//    ($fifocapacity:tt) => (
//        {
//            use $crate::usbmem::UsbPacket;
//            fifos!($fifocapacity, UsbPacket)
//        }
//    );
//    ($fifocapacity:tt, $packettype:ty) => (
//        {
//            assert!($fifocapacity <= 32);
//            use $crate::usbmem::{Fifos, Array};
//            Fifos::<[$packettype; $fifocapacity]>::new()
//        }
//    );
//}

/// All endpoints
#[derive(Clone, Copy)]
pub enum Ep {
    Rx1 = 0,
    Tx1 = 1,
    Rx2 = 2,
    Tx2 = 3,
    Rx3 = 4,
    Tx3 = 5,
    Rx4 = 6,
    Tx4 = 7,
    Rx5 = 8,
    Tx5 = 9,
    Rx6 = 10,
    Tx6 = 11,
    Rx7 = 12,
    Tx7 = 13,
    Rx8 = 14,
    Tx8 = 15,
    Rx9 = 16,
    Tx9 = 17,
    Rx10 = 18,
    Tx10 = 19,
    Rx11 = 20,
    Tx11 = 21,
    Rx12 = 22,
    Tx12 = 23,
    Rx13 = 24,
    Tx13 = 25,
    Rx14 = 26,
    Tx14 = 27,
    Rx15 = 28,
    Tx15 = 29,
}


/// Pointers to the begin and end of the FIFO queue of an endpoint. Also tracks `BorrowState`.
#[repr(packed)]
struct FifoPtrs {
    first : Option<AllocatedUsbPacket>,
    last : *mut UsbPacket,
}

impl Default for FifoPtrs {
    /// Empty FIFO.
    fn default() -> Self {
        FifoPtrs {
            first : None,
            last : ptr::null_mut(),
        }
    }
}

impl FifoPtrs {
    fn enqueue(&mut self, mut packet : AllocatedUsbPacket) {
        let new_last = unsafe { packet.ptr_inner() };
        if self.first.is_none() {
            self.first = Some(packet);
        } else {
            unsafe { (*self.last).set_next(packet); }
        }
        self.last = new_last;
    }
    fn dequeue(&mut self) -> Option<AllocatedUsbPacket> {
        match self.first.take() {
            None => { return None },
            Some(mut first) => {
                self.first = unsafe { first.take_next() };
                if self.first.is_none() {
                    self.last = ptr::null_mut();
                }
                return Some(first);
            }
        }
    }
    fn len(&mut self) -> usize {
        match self.first.as_ref() {
            None => 0,
            Some(p) => unsafe { p.count_queue() },
        }
    }
}


/// Main fifo manager and memory pool.
///
/// ```
///    let f = Fifos::<[WithNP<UsbPacket>; 32]>::new();
/// ```
///
/// The safety garanties are valid for a single core cpu only.
pub struct Fifos {
    fifos : [UnsafeCell<FifoPtrs>; 30] // (NUM_ENDPOINTS-1)*2
}


impl Fifos {
    /// Creates a new Memory pool
    pub fn new() -> Self {
        unsafe {
            let mut new = Fifos {
                fifos : mem::uninitialized(),
            };
            for element in new.fifos.iter_mut() {
                ptr::write(element, UnsafeCell::new(FifoPtrs::default()));
            }
            new
        }
    }

    pub fn enqueue(&self, endpoint : Ep, packet : AllocatedUsbPacket) {
        let _guard = NoInterrupts::new();
        let fifo = unsafe { &mut *self.fifos[endpoint as usize].get() };
        fifo.enqueue(packet);
    }

    pub fn dequeue(&self, endpoint : Ep) -> Option<AllocatedUsbPacket> {
        let _guard = NoInterrupts::new();
        let fifo = unsafe { &mut *self.fifos[endpoint as usize].get() };
        fifo.dequeue()
    }

    pub fn len(&self, endpoint : Ep) -> usize {
        let _guard = NoInterrupts::new();
        let fifo = unsafe { &mut *self.fifos[endpoint as usize].get() };
        fifo.len()
    }

    pub fn clear<T:MemoryPoolTrait>(&self, endpoint : Ep, pool : &T) {
        loop {
            match self.dequeue(endpoint) {
                Some(p) => pool.free(p),
                None => return
            }
        }
    }

    pub fn clear_all<T:MemoryPoolTrait>(&self, pool : &T) {
        let _guard = NoInterrupts::new();
        for fifo in self.fifos.iter() {
            loop {
                match unsafe { &mut *fifo.get() }.dequeue() {
                    Some(p) => pool.free(p),
                    None => break
                }
            }
        }
    }
}


