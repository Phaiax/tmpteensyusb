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
use core::cell::UnsafeCell;
use core::mem::size_of;
use core::ops::{Deref, DerefMut};
use core::mem;
use core::ptr;
use core::intrinsics::abort;
use zinc::hal::cortex_m4::irq::NoInterrupts;

#[macro_export]
/// See `usbmem` module configuration for examples.
macro_rules! fifos {
    () => (
        fifos!(32)
    );
    ($fifocapacity:tt) => (
        {
            use $crate::usbmem::UsbPacket;
            fifos!($fifocapacity, UsbPacket)
        }
    );
    ($fifocapacity:tt, $packettype:ty) => (
        {
            assert!($fifocapacity <= 32);
            use $crate::usbmem::{Fifos, WithNP, Array};
            Fifos::<[WithNP<$packettype>; $fifocapacity]>::new()
        }
    );
    ($fifocapacity:tt, packetbufsize: $packetcapacity:tt) => (
        {
            usb_packet_def!($packetcapacity);
            fifos!($fifocapacity, UsbPacket)
        }
    );
}

#[macro_export]
macro_rules! usb_packet_def {
    ($capacity:tt) => (


        /// Packet that can be recieved or send via an usb endpoint.
        pub struct UsbPacket {
            /// Length of valid bytes in buffer.
            pub len : u16,
            /// ?
            pub index : u16,
            /// May be uninitialized or containing old data. Only `len` bytes
            /// are valid.
            pub buf : [u8;$capacity],
        }

        impl UsbPacket {
            /// Returns a slice of `len` bytes of `self.buf`.
            pub fn buf(&self) -> &[u8] {
                use $crate::usbmem::Index;
                &self.buf[0..self.len.to_usize()]
            }
        }

        impl Default for UsbPacket {
            /// Used for memory pool initialization.
            fn default() -> Self {
                UsbPacket {
                    len : 0,
                    index : 0,
                    buf : unsafe { mem::uninitialized() },
                }
            }
        }

        impl $crate::usbmem::Reset for UsbPacket {
            /// Fast reset. Does not zero the buffer.
            fn reset(&mut self) {
                self.len = 0;
                self.index = 0;
            }
        }
    )
}

/// Default UsbPacket with 64 byte buffer.
usb_packet_def!(64);

/// Implementers allow recycling of itself in a memory pool.
pub trait Reset {
    fn reset(&mut self);
}

/// Elements in the FIFO must be able to store a pointer. Use `WithNP` to add this trait via a wrapper type.
/// It is implemented in this way to allow defining the size of the memory pool. (Rust does not support integer type parameters).
pub trait StoreNextPointer {
    /// Must store `next` internally.
    fn set_next(&mut self, next : *const Self);
    /// Must return the unmodified stored pointer.
    fn next(&mut self) -> *const Self;
}

/// Adds the trait `StoreNextPointer` to the inner type. Implements DerefMut to the inner type.
pub struct WithNP<T : Default + Reset>(pub T, *const WithNP<T>);

impl<T : Default + Reset> Reset for WithNP<T> {
    fn reset(&mut self) {
        self.0.reset();
    }
}

impl<T : Default + Reset> Default for WithNP<T> {
    fn default() -> Self {
        WithNP(T::default(), 0 as *const _)
    }
}

impl<T : Default + Reset> StoreNextPointer for WithNP<T> {
    fn set_next(&mut self, next : *const WithNP<T>) {
        self.1 = next
    }
    fn next(&mut self) -> *const Self {
        self.1
    }
}

impl<T : Default + Reset> Deref for WithNP<T> {
    type Target = T;
    fn deref(&self) -> &T {
        &self.0
    }
}

impl<T : Default + Reset> DerefMut for WithNP<T> {
    fn deref_mut<'a>(&'a mut self) -> &'a mut T {
        &mut self.0
    }
}

/// All endpoints
pub enum Ep {
    Tx1 = 0,
    Rx1 = 1,
    Tx2 = 2,
    Rx2 = 3,
    Tx3 = 4,
    Rx3 = 5,
    Tx4 = 6,
    Rx4 = 7,
    Tx5 = 8,
    Rx5 = 9,
    Tx6 = 10,
    Rx6 = 11,
    Tx7 = 12,
    Rx7 = 13,
    Tx8 = 14,
    Rx8 = 15,
    Tx9 = 16,
    Rx9 = 17,
    Tx10 = 18,
    Rx10 = 19,
    Tx11 = 20,
    Rx11 = 21,
    Tx12 = 22,
    Rx12 = 23,
    Tx13 = 24,
    Rx13 = 25,
    Tx14 = 26,
    Rx14 = 27,
    Tx15 = 28,
    Rx15 = 29,
}

/// Main fifo manager and memory pool.
///
/// ```
///    let f = Fifos::<[WithNP<UsbPacket>; 32]>::new();
/// ```
///
/// The safety garanties are valid for a single core cpu only.
pub struct Fifos<A:Array> {
    memory : UnsafeCell<MemoryPool<A>>,
    fifos : [UnsafeCell<FifoPtrs<A::Item>>; 30] // NUM_ENDPOINTS
}


impl<A:Array> Fifos<A> {
    /// Creates a new Memory pool
    pub fn new() -> Self {
        unsafe {
            let mut s = WithNP::<UsbPacket>::default();
            let mut new = Fifos {
                memory : UnsafeCell::new(MemoryPool::new()),
                fifos : mem::uninitialized(),
            };
            for element in new.fifos.iter_mut() {
                ptr::write(element, UnsafeCell::new(FifoPtrs::default()));
            }
            new
        }
    }

    /// Returns a helper that allows appending new elements to the FIFO that belongs to `endpoint`
    ///
    /// There can always only be one such a helper for each endpoint at a time.
    pub fn for_enqueuing(&self, endpoint : Ep) -> Option<FifoEnqueuer<A>> {
        let endpoint = endpoint as usize;
        assert!(0 <= endpoint && endpoint <= 29, "Endpoint range: 1-15");
        let ptrs = &self.fifos[endpoint];
        if unsafe { &mut (*ptrs.get()) }.borrow_state.set_enqueuing() {
            Some(FifoEnqueuer{
                ptrs : ptrs,
                memory : &self.memory,
            })
        } else {
            None
        }
    }

    /// Returns a helper that allows receiving elements from the FIFO that belongs to `endpoint`
    ///
    /// There can always only be one such a helper for each endpoint at a time.
    pub fn for_dequeuing(&self, endpoint : Ep) -> Option<FifoDequeuer<A>> {
        let endpoint = endpoint as usize;
        assert!(0 <= endpoint && endpoint <= 29, "Endpoint range: 1-15");
        let ptrs = &self.fifos[endpoint];
        if unsafe { &mut (*ptrs.get()) }.borrow_state.set_dequeuing() {
            Some(FifoDequeuer {
                ptrs : ptrs,
                memory : &self.memory,
            })
        } else {
            None
        }
    }
}

/// State machine to track if a FifoEnqueuer or FifoDequeuer is alive for an endpoint.
///
/// Has helper methods to transition state.
enum BorrowState {
    IsEnqueuing,
    IsDequeuing,
    IsEnqueuingAndDequeuing,
    Free,
}

impl BorrowState {
    /// Try to transition into enqueuing-enabled mode. Returns true if successful.
    fn set_enqueuing(&mut self) -> bool {
        let _guard = NoInterrupts::new();
        match *self {
            BorrowState::IsEnqueuing => false,
            BorrowState::IsDequeuing => { *self = BorrowState::IsEnqueuingAndDequeuing; return true; },
            BorrowState::IsEnqueuingAndDequeuing => false,
            BorrowState::Free => { *self = BorrowState::IsEnqueuing; return true; } ,
        }
    }
    /// Try to transition into dequeuing-enabled mode. Returns true if successful.
    fn set_dequeuing(&mut self) -> bool {
        let _guard = NoInterrupts::new();
        match *self {
            BorrowState::IsEnqueuing => { *self = BorrowState::IsEnqueuingAndDequeuing; return true; },
            BorrowState::IsDequeuing => false,
            BorrowState::IsEnqueuingAndDequeuing => false,
            BorrowState::Free => { *self = BorrowState::IsDequeuing; return true; } ,
        }
    }
    /// Try to transition into enqueuing-disabled mode. Returns true if successful.
    fn clear_enqueuing(&mut self) {
        let _guard = NoInterrupts::new();
        match *self {
            BorrowState::IsEnqueuing => { *self = BorrowState::Free; },
            BorrowState::IsDequeuing => { unsafe { abort(); } },
            BorrowState::IsEnqueuingAndDequeuing => { *self = BorrowState::IsDequeuing; },
            BorrowState::Free => { unsafe { abort(); } } ,
        }
    }
    /// Try to transition into dequeuing-disabled mode. Returns true if successful.
    fn clear_dequeuing(&mut self) {
        let _guard = NoInterrupts::new();
        match *self {
            BorrowState::IsEnqueuing => { unsafe { abort(); } },
            BorrowState::IsDequeuing => { *self = BorrowState::Free; },
            BorrowState::IsEnqueuingAndDequeuing => { *self = BorrowState::IsEnqueuing; },
            BorrowState::Free => { unsafe { abort(); } } ,
        }
    }
}

/// Pointers to the begin and end of the FIFO queue of an endpoint. Also tracks `BorrowState`.
struct FifoPtrs<T : Default + Reset + StoreNextPointer> {
    first : *mut T,
    last : *mut T,
    /// There should be only one responsible part of code
    /// for enqueuing data to an endpoint. (Otherwise data may
    /// be unordered). Same goes for dequeuing.
    /// (If this restriction impedes usb driver
    /// development it can be removed.)
    /// But on the other side dequeuing and enqueuing should
    /// be allowed concurrently. (secured via NoInterrupt)
    /// (Interrupt routine may remove data from fifo while
    /// the user pushes new data in the main loop.)
    borrow_state : BorrowState,
}

impl<T : Default + Reset + StoreNextPointer> Default for FifoPtrs<T> {
    /// Empty FIFO.
    fn default() -> Self {
        FifoPtrs {
            first : 0 as *mut T,
            last : 0 as *mut T,
            borrow_state : BorrowState::Free,
        }
    }
}

/// Helper to push elements into a FIFO. Can be obtained with `Fifos::for_enqueuing(endpoint)`.
pub struct FifoEnqueuer<'a, A : Array + 'a> where A::Item : 'a {
    ptrs : &'a UnsafeCell<FifoPtrs<A::Item>>,
    memory : &'a UnsafeCell<MemoryPool<A>>,
}

impl<'a, A : Array + 'a> FifoEnqueuer<'a, A> where A::Item : 'a  {
    /// Allocates a packet from the memory pool when called.
    /// Enqueues the packet when the return value of this function is dropped.
    /// It is safe to do so even in an interrupt.
    pub fn enqueue(&mut self) -> Option<ForEnqueue<A::Item>> {
        // allocate
        if let Some(unenqueued) = self.memory().allocate() {
            Some(ForEnqueue {
                unenqueued : unenqueued,
                ptrs : self.ptrs,
            })
        } else {
            None
        }
    }
    #[inline]
    /// Return reference to corresponding FIFO ptrs.
    fn ptrs(&self) -> &mut FifoPtrs<A::Item> {
        unsafe { &mut (*self.ptrs.get()) }
    }
    #[inline]
    /// Return reference to memory pool.
    fn memory(&self) -> &mut MemoryPool<A> {
        unsafe { &mut (*self.memory.get()) }
    }
}

impl<'a, A : Array + 'a> Drop for FifoEnqueuer<'a, A> where A::Item : 'a {
    /// Drops the enqueuing helper.
    ///
    /// Afterwards `Fifos::for_enqueuing(same_ep)` will return `Some` again.
    fn drop(&mut self) {
        self.ptrs().borrow_state.clear_enqueuing();
    }
}

/// Newly allocated element that is not yet appended to FIFO. Will be appended
/// if this stuct is dropped.
pub struct ForEnqueue<'a, T : Default + Reset + StoreNextPointer + 'a> {
    /// Allocated memory. Must never be NULL!
    unenqueued : *mut T,
    /// FIFO for enqueuing on drop.
    ptrs : &'a UnsafeCell<FifoPtrs<T>>,
}

impl<'a, T:Default + Reset + StoreNextPointer + 'a> ForEnqueue<'a, T> {
    #[inline]
    /// Get a reference to the allocated packet. Write into the packet buffer
    /// and then drop this struct to trigger enqueuing into fifo.
    pub fn as_mut(&mut self) -> &mut T {
        unsafe { &mut *self.unenqueued }
    }
    #[inline]
    /// Return reference to corresponding FIFO ptrs.
    fn ptrs(&self) -> &mut FifoPtrs<T> {
        unsafe { &mut (*self.ptrs.get()) }
    }
}

impl<'a, T:Default + Reset + StoreNextPointer + 'a> Deref for ForEnqueue<'a, T> {
    type Target = T;
    fn deref(&self) -> &T {
        unsafe { &*self.unenqueued }
    }
}

impl<'a, T:Default + Reset + StoreNextPointer + 'a> DerefMut for ForEnqueue<'a, T> {
    fn deref_mut<'b>(&'b mut self) -> &'b mut T {
        unsafe { &mut *self.unenqueued }
    }
}

impl<'a, T:Default + Reset + StoreNextPointer + 'a> Drop for ForEnqueue<'a, T> {
    /// Enqueues the inner packet into the FIFO.
    fn drop(&mut self) {
        let _guard = NoInterrupts::new();
        if self.ptrs().first as usize == 0 {
            self.ptrs().first = self.unenqueued;
        } else {
            unsafe { (*self.ptrs().last).set_next(self.unenqueued); }
        }
        unsafe { (*self.unenqueued).set_next(0 as _); }
        self.ptrs().last = self.unenqueued;
    }
}

/// Helper to pull elements from a FIFO. Can be obtained with `Fifos::for_dequeuing(endpoint)`.
pub struct FifoDequeuer<'a, A : Array + 'a> where A::Item : 'a  {
    ptrs : &'a UnsafeCell<FifoPtrs<A::Item>>,
    memory : &'a UnsafeCell<MemoryPool<A>>,
}

impl<'a, A: Array + 'a> FifoDequeuer<'a, A> where A::Item : 'a {
    /// Dequeues an element from the fifo when called and allows inspection of
    /// the inner packet contents.
    /// Deallocates the packet when the return value of this function is dropped.
    /// It is safe to do so even in an interrupt.
    pub fn dequeue(&mut self) -> Option<Dequeued<'a, A>> {
        let _guard = NoInterrupts::new();
        if self.ptrs().first as usize == 0 {
            return None;
        } else {
            let ret = Dequeued {
                dequeued : self.ptrs().first,
                memory : self.memory,
            };
            self.ptrs().first = unsafe { (*ret.dequeued).next() } as *mut _;
            if self.ptrs().first as usize == 0 {
                self.ptrs().last = 0usize as _;
            }
            return Some(ret);
        }
    }
    #[inline]
    /// Return reference to corresponding FIFO ptrs.
    fn ptrs(&self) -> &mut FifoPtrs<A::Item> {
        unsafe { &mut (*self.ptrs.get()) }
    }
}

impl<'a, A : Array + 'a> Drop for FifoDequeuer<'a, A> where A::Item : 'a {
    /// Drops the dequeuing helper.
    ///
    /// Afterwards `Fifos::for_dequeuing(same_ep)` will return `Some` again.
    fn drop(&mut self) {
        self.ptrs().borrow_state.clear_dequeuing();
    }
}

/// Dequeued but still allocated element. Will be freed if this stuct is dropped.
pub struct Dequeued<'a, A : Array + 'a> where A::Item : 'a  {
    /// Allocated memory. Must never be NULL!
    dequeued : *mut A::Item,
    memory : &'a UnsafeCell<MemoryPool<A>>,
}

impl<'a, A : Array + 'a> Dequeued<'a, A> where A::Item : 'a  {
    /// Allows inspection of the received packet contents.
    pub fn as_ref(&self) -> &A::Item {
        unsafe { & *self.dequeued }
    }
    #[inline]
    /// Return reference to memory pool.
    fn memory(&self) -> &mut MemoryPool<A> {
        unsafe { &mut (*self.memory.get()) }
    }
}

impl<'a, A : Array + 'a> Deref for Dequeued<'a, A> where A::Item : 'a {
    type Target = A::Item;
    fn deref(&self) -> &Self::Target {
        self.as_ref()
    }
}

impl<'a, A : Array + 'a> Drop for Dequeued<'a, A> where A::Item : 'a  {
    /// Frees the packets memory.
    fn drop(&mut self) {
        self.memory().free(self.dequeued);
    }
}

/// Usb Packet memory is allocated from this memory pool.
/// Max 32 packets are used since the bits of an u32 are used to remember allocations.
struct MemoryPool<A : Array> {
    /// We assume all items in the pool are resetted.
    pool : A,
    /// Bitmask to track unallocated pool elements.
    /// 1: Unallocated, 2: InUse
    available : u32,
}

impl<A> MemoryPool<A> where A:Array {
    /// Create new pool
    fn new() -> Self {
        let mut new = MemoryPool {
            pool : A::default(),
            available : 0xFFFFFFFF,
        };
        new
    }
    /// Allocates the next free slot in the memory pool if not completely filled.
    fn allocate(&mut self) -> Option<*mut A::Item> {
        let _guard = NoInterrupts::new();
        let n = self.available.leading_zeros();
        if n >= A::capacity() as u32 {
            return None;
        }
        self.available = self.available & !(0x80000000 >> n);
        return unsafe { Some(self.pool.as_mut_ptr().offset(n as isize) as *mut A::Item) };
    }
    /// Allows reusing the slot behind pointer `item`. Calls `reset()` from trait `Reset` on `item`.
    fn free(&mut self, item : *mut A::Item) {
        let n = (item as usize - self.pool.as_ptr() as usize) / size_of::<A::Item>();
        if n >= A::capacity() {
            unsafe { abort(); }
        }
        let mask = 0x80000000 >> n;
        unsafe { (*item).reset(); }
        let _guard = NoInterrupts::new();
        self.available |= mask;
    }
}

// *****************************************************
// The following code is taken from the crate ArrayVec and slightly modified.
// MIT License conditions apply: https://github.com/bluss/arrayvec/blob/master/LICENSE

/// Trait for fixed size arrays.
/// Max 32 packets are used since the bits of an u32 are used to remember allocations.
pub unsafe trait Array : Default where Self::Item : Default + Reset + StoreNextPointer {
    /// The array's element type
    type Item;
    #[doc(hidden)]
    fn as_ptr(&self) -> *const Self::Item;
    #[doc(hidden)]
    fn as_mut_ptr(&mut self) -> *mut Self::Item;
    #[doc(hidden)]
    fn capacity() -> usize;
}

/// Trait that allows to use smaller-than-usize integers to be used for indexing.
pub trait Index : PartialEq + Copy {
    fn to_usize(self) -> usize;
    fn from(usize) -> Self;
}

impl Index for u8 {
    #[inline(always)]
    fn to_usize(self) -> usize { self as usize }
    #[inline(always)]
    fn from(ix: usize) ->  Self { ix as u8 }
}

impl Index for u16 {
    #[inline(always)]
    fn to_usize(self) -> usize { self as usize }
    #[inline(always)]
    fn from(ix: usize) ->  Self { ix as u16 }
}

impl Index for usize {
    #[inline(always)]
    fn to_usize(self) -> usize { self }
    #[inline(always)]
    fn from(ix: usize) ->  Self { ix }
}

macro_rules! fix_array_impl {
    ($len:expr ) => (
        unsafe impl<T> Array for [T; $len] where T : Default + Reset + StoreNextPointer {
            type Item = T;
            #[inline(always)]
            fn as_ptr(&self) -> *const T { self as *const _ as *const _ }
            #[inline(always)]
            fn as_mut_ptr(&mut self) -> *mut T { self as *mut _ as *mut _}
            #[inline(always)]
            fn capacity() -> usize { $len }
        }
    )
}

macro_rules! fix_array_impl_recursive {
    () => ();
    ($len:expr, $($more:expr,)*) => (
        fix_array_impl!($len);
        fix_array_impl_recursive!($($more,)*);
    );
}

// restrict to arrays of 32 because we use an u32 to remember allocations
fix_array_impl_recursive!(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
                          16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, );

