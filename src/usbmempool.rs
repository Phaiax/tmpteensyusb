//! Define like this:
//!
//! ```
//! use usbmempool::{MemoryPoolOption, AllocatedUsbPacket, UsbPacket};
//!
//! static mut POOL : MemoryPoolOption<[UsbPacket; 32]> = MemoryPoolOption::none();
//!
//! fn pool_ref() -> MemoryPoolRef<[UsbPacket; 32]> {
//!   unsafe { POOL.unwrap() }
//! }
//! ```
//!
//! POOL must be initialized with
//!
//! ```
//! unsafe { POOL.init(); };
//! ```
//!
//! This can only be done once. Try it twice and it will panic.
//!
//! Access the pool via
//!
//! ```
//! let pool = pool_ref();
//! let packet = pool.allocate().unwrap();
//! ```
//!
//! You cannot create a non-static MemoryPoolOption. If you do, you can't call unwrap().
//! (=> Error: borrowed value must be valid for the static lifetime...)
//!
//! You must never drop allocated packets because this will panic.
//! Instead move them back to the pool.
//!
//! ```
//! pool.free(packet);
//! ```
//!
//! For moving ownership to the BufferDescriptorTable and back, you can use these unsafe functions.
//!
//! ```
//! let ptr = unsafe { p.into_buf_ptr() };
//! let recovered = unsafe { AllocatedUsbPacket::from_raw_buf_ptr(ptr) };
//! ```
//!
//! Never forget to recover the packet, otherwise you run out of memory.



use core::mem;
use core::fmt;
use core::ptr;
use core::cell::UnsafeCell;
use core::ops::Drop;
use zinc::hal::cortex_m4::irq::NoInterrupts;
use usbmem::{StoreNext, RetrieveNext};


/// Implementers allow recycling of itself in a memory pool.
pub trait Reset {
    fn reset(&mut self);
}

/// This is Magic to allow a allocated buffer to be temporary owned by a buffer descriptor table entry
pub trait BufferPointerMagic {
    unsafe fn into_buf_ptr(self) -> *mut u8;
    unsafe fn from_raw_buf_ptr(ptr: *mut u8) -> AllocatedUsbPacket;
}


/// Packet that can be recieved or send via an usb endpoint.
pub struct UsbPacket {
    /// May be uninitialized or containing old data. Only `len` bytes
    /// are valid. Must come first because of MagicFromBufferPointer
    buf : [u8;64],
    /// Length of valid bytes in buffer.
    len : u16,
    /// ?
    index : u16,
    /// Ptr for fifo functionality. Must point to an allocated UsbPacket
    next : Option<AllocatedUsbPacket>,
}

impl fmt::Debug for UsbPacket {
    fn fmt(&self, f : &mut fmt::Formatter) -> fmt::Result {
        try!(writeln!(f, "[UsbPacket len: {}, index: {}, first byte: {}", self.len, self.index, self.buf[0]));
        writeln!(f, "next: {:?} ]", self.next)
    }
}

impl Default for UsbPacket {
    /// Used for memory pool initialization.
    #[doc(hide)]
    fn default() -> UsbPacket {
        UsbPacket {
            len : 0,
            index : 0,
            buf : unsafe { mem::uninitialized() },
            next : None,
        }
    }
}

impl Reset for UsbPacket {
    /// Fast reset. Does not zero the buffer.
    fn reset(&mut self) {
        self.len = 0;
        self.index = 0;
    }
}


/// This can only be created via the memory pool
/// or some magic. Only the usb driver must do magic.
#[derive(Debug)]
pub struct AllocatedUsbPacket {
    inner : &'static mut UsbPacket,
}

impl AllocatedUsbPacket {
    /// Returns a slice of `len` bytes of `self.buf`.
    pub fn buf(&self) -> &[u8] {
        //use $crate::usbmem::Index;
        &self.inner.buf[0..self.inner.len.to_usize()]
    }
    pub fn buf_mut(&mut self, newlen : u16) -> &mut[u8] {
        assert!(newlen < 64);
        self.inner.len = newlen;
        &mut self.inner.buf[0..self.inner.len.to_usize()]
    }
    pub fn index(&self) -> u16 {
        self.inner.index
    }
    pub fn set_index(&mut self, index : u16) {
        self.inner.index = index;
    }
    pub fn recycle<T:MemoryPoolTrait>(self, pool : &T)
    {
        pool.free(self);
    }
}


impl StoreNext for UsbPacket {
    unsafe fn set_next(&mut self, next : AllocatedUsbPacket) {
        self.next = Some(next);
    }
}


impl RetrieveNext for AllocatedUsbPacket {
    unsafe fn take_next(&mut self) -> Option<AllocatedUsbPacket> {
        self.inner.next.take()
    }
    unsafe fn ptr_inner(&mut self) -> *mut UsbPacket {
        self.inner as *mut UsbPacket
    }
    unsafe fn count_queue(&self) -> usize {
        let mut cnt = 0;
        let mut next : Option<&AllocatedUsbPacket> = Some(self); // count Self
        while next.is_some() {
            cnt += 1;
            next = next.unwrap().inner.next.as_ref();
        }
        cnt
    }
}

impl BufferPointerMagic for AllocatedUsbPacket {
    unsafe fn into_buf_ptr(self) -> *mut u8 {
        let ptr = &mut (self.inner.buf[0]) as *mut u8;
        mem::forget(self);
        ptr
    }
    unsafe fn from_raw_buf_ptr(ptr : *mut u8) -> AllocatedUsbPacket {
        // offset between buffer and self must be zero
        let ptr : *mut UsbPacket = ptr as usize as *mut UsbPacket;
        AllocatedUsbPacket { inner : (&mut *ptr) }
    }
}


impl Drop for AllocatedUsbPacket {
    fn drop(&mut self) {
        panic!("Must never drop AllocatedUsbPacket!");
    }
}

/// This struct helps with the lazy initialization of a `'static` memory pool.
/// It also prevents that memory pool from being deleted by panicing on drop.
pub struct MemoryPoolOption<A : Array<Item=UsbPacket>>( Option< MemoryPool<A>> );

impl<A : Array<Item=UsbPacket>> MemoryPoolOption<A> {
    /// Const function to allow definition of `static mut POOL`. The POOL needs to be initialized.
    pub const fn none() -> MemoryPoolOption<A> {
        MemoryPoolOption(None)
    }
    /// Initialize self, that means transition from `None` to `Some`. Afterwards calls to `unwrap` are possible.
    pub fn init(&mut self) {
       match self.0 {
           None => {*self = MemoryPoolOption(Some(MemoryPool::new()));}
           Some(_) => { panic!(); }
       }
    }
    /// Get a reference to the MemoryPool. Panics if not initialized.
    pub fn unwrap(&'static mut self) -> MemoryPoolRef<A> {
        match self.0 {
            Some(ref mut p) => MemoryPoolRef(p),
            None => { panic!("Call MemoryPoolOption::init() first"); },
        }
    }
}


/// Usb Packet memory is allocated from this memory pool.
/// Max 32 packets are used since the bits of an u32 are used to remember allocations.
pub struct MemoryPool<A : Array> {
    /// We assume all items in the pool are resetted.
    pool : UnsafeCell<A>,
    /// Bitmask to track unallocated pool elements.
    /// 1: Unallocated, 0: InUse
    available : UnsafeCell<u32>,
    /// number of requested priority allocations
    priority_requests : UnsafeCell<usize>,
}




impl<A : Array> MemoryPool<A> {
    /// Create new pool
    pub fn new() -> Self {
        MemoryPool {
            pool : UnsafeCell::new(A::default()),
            available : UnsafeCell::new(0xFFFFFFFF),
            priority_requests : UnsafeCell::new(0),
        }
    }

    /// not interrupt safe
    pub fn available(&self) -> usize {
        let available : u32 = unsafe { *self.available.get() };
        available.count_ones() as usize
    }
}

impl<A:Array> Drop for MemoryPool<A> {
    fn drop(&mut self) {
       panic!("Must never drop MemoryPool");
    }
}

/// This type wraps a `'static` memory pool. We must only allocate `'static`
/// memory because we want the returned `AllocatedUsbPacket` to practically own the memory.
///
/// Also the only way to get this type is by using a `MemoryPoolOption`. That type
/// can not be destroyed.
pub struct MemoryPoolRef<A:'static + Array<Item=UsbPacket>>(&'static MemoryPool<A>);

pub trait MemoryPoolTrait {
    /// Allocates the next free slot in the memory pool if not completely filled.
    fn allocate(&self) -> Option<AllocatedUsbPacket>;
    /// Allows reusing the slot behind pointer `item`. Calls `reset()` from trait `Reset` on `item`.
    fn free(&self, item : AllocatedUsbPacket);
    /// It is assumed that a call to allocate() has failed and there are no free packets
    /// at the moment. Call this to register prioritized interest in the next packet.
    /// Implement
    ///
    /// ```
    /// #[link_name="handle_priority_allocation"]
    /// fn my_priority_allocation(p : AllocatedUsbPacket) -> Option<AllocatedUsbPacket> {}
    /// ```
    ///
    /// Return the packet if you don't need it.
    fn allocate_priority(&self);
    fn clear_priority_allocation_requests(&self);
}

impl<A> MemoryPoolRef<A> where A:Array<Item = UsbPacket> {
}

impl<A> MemoryPoolTrait for &'static MemoryPool<A> where A:Array<Item = UsbPacket> {


    /// Allocates the next free slot in the memory pool if not completely filled.
    fn allocate(&self) -> Option<AllocatedUsbPacket> {
        let _guard = NoInterrupts::new();
        let available : &mut u32 = unsafe { &mut *self.available.get() };
        let pool : &mut A = unsafe { &mut *self.pool.get() };
        let n = available.leading_zeros();
        if n >= A::capacity() as u32 {
            return None;
        }
        *available = *available & !(0x80000000 >> n);
        //info!("Allocate n {}, addr 0x{:x}", n, unsafe { pool.as_mut_ptr().offset(n as isize) as usize } );
        Some(AllocatedUsbPacket {
            inner : unsafe { &mut * pool.as_mut_ptr().offset(n as isize) }
        })
    }
    /// Allows reusing the slot behind pointer `item`. Calls `reset()` from trait `Reset` on `item`.
    fn free(&self, mut item : AllocatedUsbPacket) {
        {
            let guard = NoInterrupts::new();
            let priority_requests : &mut usize = unsafe { &mut *self.priority_requests.get() };
            if *priority_requests > 0 {
                *priority_requests -= 1;
                mem::drop(guard);
                match unsafe { handle_priority_allocation(item) } {
                    None => return,
                    Some(packet) => {
                        info!("Prio alloc did not need the packet.");
                        item = packet;
                    }
                }
            }
        }
        let pool : &mut A = unsafe { &mut *self.pool.get() };
        let offset = item.inner as *const UsbPacket as usize - pool.as_ptr() as usize;
        let n = offset / mem::size_of::<A::Item>();
        //info!("{} free: packet addr 0x{:x}, pool 0x{:x}", n, item.inner as *const UsbPacket as usize, pool.as_ptr() as usize, n);
        if n >= A::capacity() { // if the AllocatedPackets address is < Array baseaddr, offset has overflowed
            panic!();
        }
        let mask = 0x80000000 >> n;
        item.inner.reset();
        let _guard = NoInterrupts::new();
        mem::forget(item);
        let available : &mut u32 = unsafe { &mut *self.available.get() };
        *available |= mask;
    }
    ///
    fn allocate_priority(&self) {
        let _guard = NoInterrupts::new();
        let priority_requests : &mut usize = unsafe { &mut *self.priority_requests.get() };
        *priority_requests += 1;
    }
    ///
    fn clear_priority_allocation_requests(&self) {
        let _guard = NoInterrupts::new();
        unsafe { *self.priority_requests.get() = 0; };
    }
}

extern {
    fn handle_priority_allocation(p : AllocatedUsbPacket) -> Option<AllocatedUsbPacket>;
}

// #[link_name="handle_priority_allocation"]
// #[linkage = "weak"]
// fn handle_priority_allocation(p : AllocatedUsbPacket) -> Option<AllocatedUsbPacket> {
//     Some(p)
// }

// *****************************************************
// The following code is taken from the crate ArrayVec and slightly modified.
// MIT License conditions apply: https://github.com/bluss/arrayvec/blob/master/LICENSE

/// Trait for fixed size arrays.
/// Max 32 packets are used since the bits of an u32 are used to remember allocations.
pub unsafe trait Array : Default where Self::Item : Default + Reset {
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
        unsafe impl<T> Array for [T; $len] where T : Default + Reset {
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