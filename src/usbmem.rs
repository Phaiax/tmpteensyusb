        use ::wait;
use core::cell::UnsafeCell;
use core::mem::size_of;
use core::mem;
use core::ptr;
use core::intrinsics::abort;
use zinc::hal::cortex_m4::irq::NoInterrupts;
//typedef struct usb_packet_struct {
//    uint16_t len;
//    uint16_t index;
//    struct usb_packet_struct *next;
//    uint8_t buf[64];
//} usb_packet_t;



// we need a fifo for each endpoint
// all fifos must use a common memory pool

pub struct UsbPacket {
    pub len : u16,
    pub index : u16,
    pub buf : [u8;64],
    next : *const UsbPacket,
}

impl Default for UsbPacket {
    fn default() -> Self {
        let mut s = 3u8;
        s += 314;
        let u = UsbPacket {
            len : 0,
            index : 0,
            buf : [0; 64],
            next : 0 as *const _
        };
        u
    }
}

pub trait Reset {
    fn reset(&mut self);
}

impl Reset for UsbPacket {
    fn reset(&mut self) {
        self.len = 0;
        self.index = 0;
    }
}

pub trait StoreNextPointer {
    fn set_next(&mut self, next : *const Self);
    fn next(&mut self) -> *const Self;
}

impl StoreNextPointer for UsbPacket {
    fn set_next(&mut self, next : *const UsbPacket) {
        self.next = next
    }
    fn next(&mut self) -> *const Self {
        self.next
    }
}

pub struct Fifos<A:Array> {
    memory : UnsafeCell<MemoryPool<A>>,
    fifos : [UnsafeCell<FifoPtrs<A::Item>>; 15] // NUM_ENDPOINTS
}


impl<A:Array> Fifos<A> {
    pub fn new() -> Self {
        unsafe {
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
    pub fn for_enqueuing(&self, endpoint : u8) -> Option<FifoEnqueuer<A>> {
        info!(target: "mem", "ask ptrs");
        let ptrs = &self.fifos[endpoint as usize];
        info!(target: "mem", "ptrs {:p}", ptrs);
        return None;
        if unsafe { &mut (*ptrs.get()) }.borrow_state.set_enqueuing() {
            Some(FifoEnqueuer{
                ptrs : ptrs,
                memory : &self.memory,
            })
        } else {
            None
        }
    }
    pub fn for_dequeuing(&self, endpoint : u8) -> Option<FifoDequeuer<A>> {
        let ptrs = &self.fifos[endpoint as usize];
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

enum BorrowState {
    IsEnqueuing,
    IsDequeuing,
    IsEnqueuingAndDequeuing,
    Free,
}

impl BorrowState {
    /// returns true if allowed and setted
    fn set_enqueuing(&mut self) -> bool {
        let _guard = NoInterrupts::new();
        match *self {
            BorrowState::IsEnqueuing => false,
            BorrowState::IsDequeuing => { *self = BorrowState::IsEnqueuingAndDequeuing; return true; },
            BorrowState::IsEnqueuingAndDequeuing => false,
            BorrowState::Free => { *self = BorrowState::IsEnqueuing; return true; } ,
        }
    }
    fn set_dequeuing(&mut self) -> bool {
        let _guard = NoInterrupts::new();
        match *self {
            BorrowState::IsEnqueuing => { *self = BorrowState::IsEnqueuingAndDequeuing; return true; },
            BorrowState::IsDequeuing => false,
            BorrowState::IsEnqueuingAndDequeuing => false,
            BorrowState::Free => { *self = BorrowState::IsDequeuing; return true; } ,
        }
    }
    fn clear_enqueuing(&mut self) {
        let _guard = NoInterrupts::new();
        match *self {
            BorrowState::IsEnqueuing => { *self = BorrowState::Free; },
            BorrowState::IsDequeuing => { unsafe { abort(); } },
            BorrowState::IsEnqueuingAndDequeuing => { *self = BorrowState::IsDequeuing; },
            BorrowState::Free => { unsafe { abort(); } } ,
        }
    }
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
    fn default() -> Self {
        FifoPtrs {
            first : 0 as *mut T,
            last : 0 as *mut T,
            borrow_state : BorrowState::Free,
        }
    }
}

pub struct FifoEnqueuer<'a, A : Array + 'a> where A::Item : 'a {
    ptrs : &'a UnsafeCell<FifoPtrs<A::Item>>,
    memory : &'a UnsafeCell<MemoryPool<A>>,
}

impl<'a, A : Array + 'a> FifoEnqueuer<'a, A> where A::Item : 'a  {
    /// Allocates a packet when called.
    /// Enqueues the packet when the return value
    /// of this function is dropped.
    pub fn enqueue(&mut self) -> Option<ForEnqueue<A::Item>> {
        // allocate
        if let Some(unenqueued) = unsafe { self.memory().allocate() } {
            Some(ForEnqueue {
                unenqueued : unenqueued,
                ptrs : self.ptrs,
            })
        } else {
            None
        }
    }
    #[inline]
    fn ptrs(&self) -> &mut FifoPtrs<A::Item> {
        unsafe { &mut (*self.ptrs.get()) }
    }
    #[inline]
    fn memory(&self) -> &mut MemoryPool<A> {
        unsafe { &mut (*self.memory.get()) }
    }
}

impl<'a, A : Array + 'a> Drop for FifoEnqueuer<'a, A> where A::Item : 'a {
    fn drop(&mut self) {
        self.ptrs().borrow_state.clear_enqueuing();
    }
}

pub struct ForEnqueue<'a, T : Default + Reset + StoreNextPointer + 'a> {
    /// Must never be NULL
    unenqueued : *mut T,
    ptrs : &'a UnsafeCell<FifoPtrs<T>>,
}

impl<'a, T:Default + Reset + StoreNextPointer + 'a> ForEnqueue<'a, T> {
    #[inline]
    pub fn as_mut(&mut self) -> &mut T {
        unsafe { &mut *self.unenqueued }
    }
    #[inline]
    fn ptrs(&self) -> &mut FifoPtrs<T> {
        unsafe { &mut (*self.ptrs.get()) }
    }
}

impl<'a, T:Default + Reset + StoreNextPointer + 'a> Drop for ForEnqueue<'a, T> {
    fn drop(&mut self) {
        // enqueue! (after last)
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

pub struct FifoDequeuer<'a, A : Array + 'a> where A::Item : 'a  {
    ptrs : &'a UnsafeCell<FifoPtrs<A::Item>>,
    memory : &'a UnsafeCell<MemoryPool<A>>,
}

impl<'a, A: Array + 'a> FifoDequeuer<'a, A> where A::Item : 'a {
    /// Dequeues the packet when called.
    /// Deallocates the packet when the return
    /// value of this function is dropped.
    pub fn dequeue(&mut self) -> Option<Dequeued<'a, A>> {
        // dequeue!
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
    fn ptrs(&self) -> &mut FifoPtrs<A::Item> {
        unsafe { &mut (*self.ptrs.get()) }
    }
}

impl<'a, A : Array + 'a> Drop for FifoDequeuer<'a, A> where A::Item : 'a {
    fn drop(&mut self) {
        self.ptrs().borrow_state.clear_dequeuing();
    }
}

pub struct Dequeued<'a, A : Array + 'a> where A::Item : 'a  {
    /// Must never be NULL
    dequeued : *mut A::Item,
    memory : &'a UnsafeCell<MemoryPool<A>>,
}

impl<'a, A : Array + 'a> Dequeued<'a, A> where A::Item : 'a  {
    pub fn as_ref(&self) -> &A::Item {
        unsafe { & *self.dequeued }
    }
    #[inline]
    fn memory(&self) -> &mut MemoryPool<A> {
        unsafe { &mut (*self.memory.get()) }
    }
}

impl<'a, A : Array + 'a> Drop for Dequeued<'a, A> where A::Item : 'a  {
    fn drop(&mut self) {
        unsafe { self.memory().free(self.dequeued); }
    }
}


/// Usb Packet memory is allocated from this memory pool since
/// we do not want to be flooded with memory.
/// Max 32 packets since the bits of an u32 are used to remember allocations
/// This struct uses 2304 bytes of heap memory via Rc<>.
/// Maybe use a more efficient but unsafe implementation
struct MemoryPool<A : Array> {
    /// We assume all items in the pool are resetted.
    pool : A,
    available : u32,
}

impl<A> MemoryPool<A> where A:Array {
    pub fn new() -> Self {
        info!(target: "mem", "new mem");

        let mut new = MemoryPool {
            pool : unsafe { mem::uninitialized() },
            available : 0xFFFFFFFF,
        };
        unsafe {
            let mut a : &mut A = &mut new.pool; // fix 'type of value must be known'
            let start : *mut A::Item = a.as_mut_ptr();
            for i in 0..A::capacity() {
                ptr::write(start.offset(i as isize), A::Item::default());
            }
        }
        new
    }
    unsafe fn allocate(&mut self) -> Option<*mut A::Item> {
        let _guard = NoInterrupts::new();
        let n = self.available.leading_zeros();
        if n >= A::capacity() as u32 {
            return None;
        }
        self.available = self.available & !(0x80000000 >> n);
        return Some(self.pool.as_mut_ptr().offset(n as isize) as *mut A::Item);
    }
    unsafe fn free(&mut self, item : *mut A::Item) {
        let n = (item as usize - self.pool.as_ptr() as usize) / size_of::<A::Item>();
        if n >= A::capacity() {
            abort();
        }
        let mask = 0x80000000 >> n;
        (*item).reset();
        let _guard = NoInterrupts::new();
        self.available |= mask;
    }
}

// *****************************************************
// The following code is taken from the crate ArrayVec and slightly modified.
// MIT License conditions apply: https://github.com/bluss/arrayvec/blob/master/LICENSE

/// Trait for fixed size arrays.
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
fix_array_impl_recursive!(2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
                          16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, );

