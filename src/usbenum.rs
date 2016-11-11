
use zinc::hal::k20::regs::Usb_stat_Get;

pub use zinc::hal::k20::regs::Usb_stat_odd as Bank;
pub use zinc::hal::k20::regs::Usb_stat_tx as Direction;



/// Endpoint, direction (rx/tx) and bank (even/odd)
///
/// Internal Format: `0b00ee_eedb`
#[derive(Clone, Copy)]
pub struct EndpointWithDirAndBank(u8);

impl From<Usb_stat_Get> for EndpointWithDirAndBank {
    fn from(stat : Usb_stat_Get) -> EndpointWithDirAndBank {
        EndpointWithDirAndBank(stat.raw() >> 2)
    }
}

impl EndpointWithDirAndBank {
    /// new from zero based endpoint nr
    pub fn new(ep : u8, d : Direction, b : Bank) -> EndpointWithDirAndBank {
        let ep : Endpoint = ep.into();
        ep.with_dir_bank(d, b)
    }
    /// Return the index used in the buffer descriptor array
    pub fn as_bufferdescriptorarray_index(self) -> usize {
        self.0 as usize
    }
}

/// Endpoint, direction (rx/tx)
///
/// Internal Format: 0b000e_eeed
#[derive(Clone, Copy)]
pub struct EndpointWithDir(u8);

impl From<EndpointWithDirAndBank> for EndpointWithDir {
    fn from(ep_d_b : EndpointWithDirAndBank) -> EndpointWithDir {
        EndpointWithDir(ep_d_b.0 >> 1)
    }
}

impl EndpointWithDir {
    /// new from zero based endpoint nr
    pub fn new(ep : u8, d : Direction) -> EndpointWithDir {
        let ep : Endpoint = ep.into();
        ep.with_dir(d)
    }
    /// Convert to EndpointWithDirAndBank
    pub fn with_bank(self, b : Bank) -> EndpointWithDirAndBank {
        EndpointWithDirAndBank( (self.0 << 1) | (b as u8) )
    }
    /// Return the Ep1Rx=0, Ep1Tx=1, ... Ep15Tx=29 based index used in the fifos
    pub fn as_fifo_1_index(self) -> usize {
        assert!(self.0 >= 2);
        (self.0 - 2) as usize
    }
}

/// Endpoint
///
/// Internal Format: 0b0000_eeee
#[derive(Clone, Copy)]
pub struct Endpoint(u8);

impl From<EndpointWithDirAndBank> for Endpoint {
    fn from(ep_d_b : EndpointWithDirAndBank) -> Endpoint {
        Endpoint(ep_d_b.0 >> 2)
    }
}

impl From<EndpointWithDir> for Endpoint {
    fn from(ep_d_b : EndpointWithDir) -> Endpoint {
        Endpoint(ep_d_b.0 >> 1)
    }
}

impl From<u8> for Endpoint {
    fn from(ep : u8) -> Endpoint {
        assert!(ep <= 15);
        Endpoint(ep)
    }
}

impl Endpoint {
    pub fn with_dir_bank(self, d : Direction, b : Bank) -> EndpointWithDirAndBank {
        EndpointWithDirAndBank( (self.0 << 2) | ((d as u8) << 1) | (b as u8) )
    }
    pub fn with_dir(self, d : Direction) -> EndpointWithDir {
        EndpointWithDir( (self.0 << 1) | (d as u8) )
    }
}
