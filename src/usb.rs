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

use volatile_cell::VolatileCell;

use core::slice;
use core::mem;
use core::ptr;

use usbmempool::{AllocatedUsbPacket, BufferPointerMagic};



ioregs!(BufferDescriptor = { //! An individual K20 Buffer Descriptor
    0 => reg32 control { //! Control attributes
        25..16 => bc, //= Byte Count
        7 => own { //! Determines whether the processor or the USB-FS currently owns the buffer.
            0 => Processor, //= this buffer descriptor can be modified by code/the CPU
            1 => Controller //= this buffer descriptor can only be modifed by the USB controller
        },
        6 => data01 { //! Defines whether a DATA0 field (DATA0/1=0) or a DATA1 (DATA0/1=1) field was transmitted or received. It is unchanged by the USB-FS.
            0 => Data0,
            1 => Data1
        },
        5 => keep, //= Tok[3] _or_ 'Keep'. Typically, this bit is 1 with ISO endpoints feeding a FIFO. The microprocessor is not informed that a token has been processed, the data is simply transferred to or from the FIFO. When KEEP is set, normally the NINC bit is also set to prevent address increment.
        4 => ninc, //= Tok[2] _or_ 'No Increment'. Disables the DMA engine address increment. This forces the DMA engine to read or write from the same address. This is useful for endpoints when data needs to be read from or written to a single location such as a FIFO. Typically this bit is set with the KEEP bit for ISO endpoints that are interfacing to a FIFO.
        3 => dts, //= Tok[1] _or_ 'Data Toggle Synchronization'. Setting this bit enables the USB-FS to perform Data Toggle Synchronization.
        2 => bdt_stall, //= Tok[0] _or_ trigger STALL handshake if this BDT is used. Setting this bit causes the USB-FS to issue a STALL handshake if a token is received by the SIE that would use the BDT in this location.
    },
    4 => reg32 addr { //! Buffer Address
        31..0 => addr //= The 32bit address of the buffer in memory.
    }
});

// Add an override field for pid_tok over 5..2 of a BufferDescriptor. (Hand implement what ioregs would)
impl BufferDescriptor_control {
    /// Returns raw == 0
    pub fn is_zero(&self) -> bool {
        self.value.get() == 0x00
    }
    /// Return the token for this buffer descriptor (overloaded with keep/ninc/dts/bdt_stall
    pub fn pid_tok(&self) -> u32 {
        BufferDescriptor_control_Get::new(self)
            .pid_tok()
    }
    /// Set the value of the pid_tok field
    pub fn set_pid_tok<'a>(&'a self, new_value: u32) -> BufferDescriptor_control_Update<'a> {
        let mut setter: BufferDescriptor_control_Update = BufferDescriptor_control_Update::new(self);
        setter.set_pid_tok(new_value);
        setter
    }
}

impl BufferDescriptor_control_Get {
    /// Return the token for this buffer descriptor (overloaded with keep/ninc/dts/bdt_stall
    pub fn pid_tok(&self) -> u32 {
        ((self.value >> 2) & 0b1111)
    }
}

impl<'a> BufferDescriptor_control_Update<'a> {
    /// Set the value of the pid_tok field
    #[inline(always)]
    pub fn set_pid_tok<'b>(&'b mut self,
                           new_value: u32)
                           -> &'b mut BufferDescriptor_control_Update<'a> {
        self.value = (self.value & !(0b1111 << 2)) | ((new_value as u32) & 0b1111) << 2;
        self.mask |= 0b1111 << 2;
        self
    }
    /// Clear all
    #[inline(always)]
    pub fn zero_all<'b>(&'b mut self) -> &'b mut BufferDescriptor_control_Update<'a> {
        self.value = 0x00;
        self.mask = 0xFF;
        self
    }
    /// Give back to controller
    #[inline(always)]
    pub fn give_back<'b>(&'b mut self,
                         buffersize: usize,
                         data01: BufferDescriptor_control_data01)
                         -> &'b mut BufferDescriptor_control_Update<'a> {
        self.set_own(BufferDescriptor_control_own::Controller)
            .set_dts(true)
            .set_data01(data01)
            .set_bc(buffersize as u32);
        self
    }
}

impl BufferDescriptor_addr {
}

impl BufferDescriptor {
    pub fn swap_usb_packet(&self, i: Option<AllocatedUsbPacket>) -> Option<AllocatedUsbPacket> {
        let a = self.addr.addr();
        self.addr.ignoring_state().set_addr(match i {
            Some(p) => unsafe { p.into_buf_ptr() as u32 },
            None => 0,
        });
        if a != 0 {
            Some(unsafe { AllocatedUsbPacket::from_raw_buf_ptr(a as *mut u8, self.control.bc() as u16) })
        } else {
            None
        }
    }

    /// _
    pub unsafe fn interpret_buf_as_setup_packet(&self) -> SetupPacket {
        assert!(self.addr.addr() != 0);
        assert!(self.control.bc() == 8);
        let setuppacket: &SetupPacket = mem::transmute(self.addr.addr());
        *setuppacket
    }
}

impl Copy for BufferDescriptor_control_data01{}

impl Clone for BufferDescriptor_control_data01 {
    fn clone(&self) -> BufferDescriptor_control_data01 {
        *self
    }
}

impl BufferDescriptor_control_data01 {
    pub fn toggle(&mut self) {
        use self::BufferDescriptor_control_data01::*;
        *self = match *self {
            Data1 => Data0,
            Data0 => Data1,
        };
    }
    pub fn next(&self) -> BufferDescriptor_control_data01 {
        use self::BufferDescriptor_control_data01::*;
        match *self {
            Data1 => Data0,
            Data0 => Data1,
        }
    }
}


#[repr(C)]
#[derive(Copy, Clone, Debug, Default)]
/// _
pub struct SetupPacket {
    /// _
    bmRequestType : u8,
    /// Specific Request
    bRequest : u8,
    /// Use varies according to request
    wValue : u16,
    /// Use varies according to request
    wIndex : u16,
    /// Number of bytes to transfer if there is a data stage
    wLength : u16,
}

/// _
pub enum DataDirection {
    /// _
    HostToDevice,
    /// _
    DeviceToHost,
}

/// _
pub enum RequestType {
    /// _
    Standard,
    /// _
    Class,
    /// _
    Vendor,
    /// _
    Reserved
}

/// _
pub enum RequestRecipient {
    /// _
    Device,
    /// _
    Interface,
    /// _
    Endpoint,
    /// _
    Other,
    /// _
    Reserved,
}


/*pub enum RequestAndType {
    SetAddress,
    SetConfiguration,
    GetConfiguration,
    GetDeviceStatus,
    GetEndpointStatus,
    ClearEndpointFeature,
    SetEndpointFeature,
    GetDescriptor,
    GetDescriptor2,
    CdcSetControlLineState,
    CdcSendBreak,
    CdcSetLineCoding,
    SetInterfaceAlternate,
    GetInterfaceAlternate,
}
*/
impl SetupPacket {

    /// _
    pub fn data_direction(&self) -> DataDirection {
        match (self.bmRequestType & 0b1000_0000) >> 7 {
            0 => DataDirection::HostToDevice,
            _ => DataDirection::DeviceToHost,
        }
    }

    /// _
    pub fn request_type(&self) -> RequestType {
        match (self.bmRequestType & 0b0110_0000) >> 5 {
            0 => RequestType::Standard,
            1 => RequestType::Class,
            2 => RequestType::Vendor,
            _ => RequestType::Reserved,
        }
    }

    /// _
    pub fn recipient(&self) -> RequestRecipient {
        match self.bmRequestType & 0b0001_1111 {
            0 => RequestRecipient::Device,
            1 => RequestRecipient::Interface,
            2 => RequestRecipient::Endpoint,
            3 => RequestRecipient::Other,
            _ => RequestRecipient::Reserved,
        }
    }

    /// _
    pub fn request(&self) -> u8 {
        self.bRequest
    }

    /// _
    pub fn clear_request(&mut self) {
        self.bRequest = 0;
    }

    #[inline(always)]
    /// _
    pub fn request_and_type(&self) -> u16 {
        let as_arr = unsafe { mem::transmute::<&SetupPacket, &[u16; 4]>(&self) };
        as_arr[0]
    }

    /// _
    pub fn wValue(&self) -> u16 {
        self.wValue
    }

    /// _
    pub fn wIndex(&self) -> u16 {
        self.wIndex
    }

    /// _
    pub fn wLength(&self) -> u16 {
        self.wLength
    }
}