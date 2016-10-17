// Zinc, the bare metal stack for rust.
// Copyright 2016 Geoff Cant 'archaelus' <nem@erlang.geek.nz>
// Copyright 2016 Daniel Seemer 'phaiax' <phaiax-zinc@invisibletower.de>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*!
USB Tranceiver functions and utilities.

Sooo... how is all this going to work.

Firstly memory. We need a buffer descriptor table sufficient for 2-16 endpoints allocated with 512byte alignment. The calling code is going to have to do that and pass it in to us, for us to manage.

Then we need some transceiver buffers. We can statically allocate some buffers for known message sizes ourselves (8 byte setup message buffer?), but whenever the calling code wants to receive from USB, they're going to have to give us some memory for us to write into.

 */

//use hal::k20::regs;
//use hal::isr::isr_k20;

//use util::support::nop;

use core::slice;
use core::mem;
use core::ptr;

pub use zinc::hal::k20::usb::*;

pub enum OddEven {
    Even = 0b0000_0000,
    Odd = 0b0000_0001,
}

pub enum TxRx {
    Rx = 0b0000_0000,
    Tx = 0b0000_0010,
}

pub enum Ep {
    Ep0 = 0b00_0000_00,
    Ep1 = 0b00_0001_00,
    Ep2 = 0b00_0010_00,
    Ep3 = 0b00_0011_00,
    Ep4 = 0b00_0100_00,
    Ep5 = 0b00_0101_00,
    Ep6 = 0b00_0110_00,
    Ep7 = 0b00_0111_00,
    Ep8 = 0b00_1000_00,
    Ep9 = 0b00_1001_00,
    Ep10 = 0b00_1010_00,
    Ep11 = 0b00_1011_00,
    Ep12 = 0b00_1100_00,
    Ep13 = 0b00_1101_00,
    Ep14 = 0b00_1110_00,
    Ep15 = 0b00_1111_00,
}
/*
#[repr(C)]
#[derive(Copy, Clone, Debug)]
/// A K20 USB-FS Buffer Descriptor
pub struct BufferDescriptor {
    control: usize,
    addr: *const u8
}

impl Default for BufferDescriptor {
    fn default() -> BufferDescriptor {
        BufferDescriptor {
            control: 0,
            addr: ptr::null()
        }
    }
}

#[repr(C)]
/// Buffer Descriptor Ownership (Processor or USB-FS Controller)
pub enum BufferDescriptor_own {
    /// The CPU (and thus rust-code) owns and can modify this descriptor
    Processor = 0,
    /// The USB-FS controller owns this buffer descriptor - do not modify it.
    Controller = 1
}



impl BufferDescriptor {

    /// Create a new buffer descriptor backed by byte buffer (you supply a pointer to the buffer that must remain valid until the USB Controler doesn't need it any more
    pub fn new(buf: *const u8) -> BufferDescriptor {
        let bd = BufferDescriptor {
            control: 0,
            addr: buf
        };
        bd
    }

    /// Buffer Byte Count (up to 1024 bytes)
    pub fn byte_count(&self) -> usize {
        // Bits 25..16
        (self.control & 0x03_FF_00_00) >> 16
    }

    /// Buffer Descriptor ownership
    pub fn ownership(&self) -> BufferDescriptor_own {
        // Bit 7
        match self.control & 0x00_00_00_40 {
            0 => BufferDescriptor_own::Processor,
            _ => BufferDescriptor_own::Controller
        }
    }

    /// USB Packet token for this buffer (tok_pid)
    pub fn token(&self) -> u32 {
        // Bits 5..2
        ((self.control & 0b11_1100) >> 2) as u32
    }

    /// If true, the USB-FS keeps ownership of this BDT
    pub fn keep(&self) -> bool {
        // Bit 5
        (self.control & 0b10_0000) != 0
    }

    /// True if the USB-FS shouldn't increment the address it writes to. Usually only used to point the BDT into a fifo.
    pub fn ninc(&self) -> bool {
        // Bit 4
        (self.control & 0b1_0000) != 0
    }

    /// If true, 'Data Tottle Synchronization' is used. No idea - look it up in the manual
    pub fn dts(&self) -> bool {
        // Bit 3
        (self.control & 0b1000) != 0
    }

    /// Trigger a USB Stall if this BDT would be used.
    pub fn bdt_stall(&self) -> bool {
        // Bit 2
        (self.control & 0b100) != 0
    }

    /// Obtain a slice from the address in the BDT plus the number of bytes written into it.
    pub fn buffer(&self) -> &[u8] {
        assert!(!self.addr.is_null());
        unsafe { slice::from_raw_parts(self.addr, self.byte_count()) }
    }

    pub unsafe fn interpret_as_setup_packet(&self) -> SetupPacket {
        assert!(!self.addr.is_null());
        assert!(self.byte_count() == 8);
        mem::transmute_copy::<[u8; 8], SetupPacket>(&*(self.addr as *const [u8; 8]))
    }
}
*/

