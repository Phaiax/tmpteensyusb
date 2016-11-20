#![allow(dead_code)]
// init for EEPROM
    pub const DEVICEDESCRIPTOR: &'static [u8] = &[
        18,      // bLength
        0x1,      // bDescriptorType
        1, 1,// bcdUSB
        0x2,      // bDeviceClass
        0x0,      // bDeviceSubClass
        0x0,      // bDeviceProtocol
        0x40,      // bMaxPacketSize0
        0xc0, 0x16,// idVendor
        0x83, 0x4,// idProduct
        0x0, 0x1,// bcdDevice
        0x1,      // iManufacturer
        0x2,      // iProduct
        0x3,      // iSerialNumber
        0x1,      // bNumConfigurations
    ];
             
    pub const CONFIGDESCRIPTORTREE: &'static [u8] = &[
        // CONFIGURATION DESCRIPTOR
            9,      // bLength
            0x2,      // bDescriptorType
            0x43, 0x0,// wTotalLength
            0x2,      // bNumInterfaces
            0x1,      // bConfigurationValue
            0x4,      // iConfiguration
            0xc0,      // bmAttributes
            0x32,      // bMaxPower
            
        // INTERFACE
            9,      // bLength
            0x4,      // bDescriptorType
            0x0,      // bInterfaceNumber
            0x0,      // bAlternateSetting
            0x1,      // bNumEndpoints
            0x2,      // bInterfaceClass
            0x2,      // bInterfaceSubClass
            0x1,      // bInterfaceProtocol
            0x5,      // iInterface
            
        // CDC Interface Functional Descriptor : Header
            5,        // bLength
            0x24,     // bDescriptorType
            0x0,      // bDescriptorSubType
            16, 1,    // bcdCDC
        // CDC Interface Functional Descriptor : CallManagement
            5,        // bLength
            0x24,     // bDescriptorType
            0x1,      // bDescriptorSubType
            0b00000000, // bmCapabilities
            1, // bDataInterface
        // CDC Interface Functional Descriptor : AbstractControlManagement
            4,        // bLength
            0x24,     // bDescriptorType
            0x2,      // bDescriptorSubType
            0b00000110, // bmCapabilities
        // CDC Interface Functional Descriptor : Union
            5,        // bLength
            0x24,     // bDescriptorType
            0x6,      // bDescriptorSubType
            0,        // bControlInterface
			1,        // bSubordinateInterface0
        // ENDPOINT
            7,      // bLength
            0x5,      // bDescriptorType
            0x82,      // bEndpointAddress
            0x3,      // bmAttributes
            0x10, 0x0,// wMaxPacketSize
            0x40,      // bInterval
            
        // INTERFACE
            9,      // bLength
            0x4,      // bDescriptorType
            0x1,      // bInterfaceNumber
            0x0,      // bAlternateSetting
            0x2,      // bNumEndpoints
            0xa,      // bInterfaceClass
            0x0,      // bInterfaceSubClass
            0x0,      // bInterfaceProtocol
            0x6,      // iInterface
            
        // ENDPOINT
            7,      // bLength
            0x5,      // bDescriptorType
            0x3,      // bEndpointAddress
            0x2,      // bmAttributes
            0x40, 0x0,// wMaxPacketSize
            0x0,      // bInterval
            
        // ENDPOINT
            7,      // bLength
            0x5,      // bDescriptorType
            0x84,      // bEndpointAddress
            0x2,      // bmAttributes
            0x40, 0x0,// wMaxPacketSize
            0x0,      // bInterval
            
    ];
    const STRINGZERODESCRIPTOR: &'static [u8] = &[
        4,      // bLength
        0x3,      // bDescriptorType
        0x9, 0x4
    ];
    pub const STRING_1_DESCRIPTOR: &'static [u8] = &[
        14,      // bLength
        0x3,      // bDescriptorType
        // Daniel
        0x44, 0x0, 0x61, 0x0, 0x6e, 0x0, 0x69, 0x0, 0x65, 0x0, 0x6c, 0x0, 
    ];
    pub const STRING_2_DESCRIPTOR: &'static [u8] = &[
        22,      // bLength
        0x3,      // bDescriptorType
        // THE Profud
        0x54, 0x0, 0x48, 0x0, 0x45, 0x0, 0x20, 0x0, 0x50, 0x0, 0x72, 0x0, 0x6f, 0x0, 0x66, 0x0, 0x75, 0x0, 0x64, 0x0, 
    ];
    pub const STRING_3_DESCRIPTOR: &'static [u8] = &[
        12,      // bLength
        0x3,      // bDescriptorType
        // 12345
        0x31, 0x0, 0x32, 0x0, 0x33, 0x0, 0x34, 0x0, 0x35, 0x0, 
    ];
    pub const STRING_4_DESCRIPTOR: &'static [u8] = &[
        10,      // bLength
        0x3,      // bDescriptorType
        // Blub
        0x42, 0x0, 0x6c, 0x0, 0x75, 0x0, 0x62, 0x0, 
    ];
    pub const STRING_5_DESCRIPTOR: &'static [u8] = &[
        10,      // bLength
        0x3,      // bDescriptorType
        // Int1
        0x49, 0x0, 0x6e, 0x0, 0x74, 0x0, 0x31, 0x0, 
    ];
    pub const STRING_6_DESCRIPTOR: &'static [u8] = &[
        10,      // bLength
        0x3,      // bDescriptorType
        // Int2
        0x49, 0x0, 0x6e, 0x0, 0x74, 0x0, 0x32, 0x0, 
    ];
    pub const STRING_7_DESCRIPTOR: &'static [u8] = &[
        24,      // bLength
        0x3,      // bDescriptorType
        // No str fnd.
        0x4e, 0x0, 0x6f, 0x0, 0x20, 0x0, 0x73, 0x0, 0x74, 0x0, 0x72, 0x0, 0x20, 0x0, 0x66, 0x0, 0x6e, 0x0, 0x64, 0x0, 0x2e, 0x0, 
    ];
    pub fn get_str(strdescr_id : u8) -> &'static [u8] {
        match strdescr_id { 
            0 => STRINGZERODESCRIPTOR,
            1 => STRING_1_DESCRIPTOR,
            2 => STRING_2_DESCRIPTOR,
            3 => STRING_3_DESCRIPTOR,
            4 => STRING_4_DESCRIPTOR,
            5 => STRING_5_DESCRIPTOR,
            6 => STRING_6_DESCRIPTOR,
            7 => STRING_7_DESCRIPTOR,
            _ => STRING_7_DESCRIPTOR,
        }
    }

    extern {
        #[no_mangle]
        #[link_name="_usbbufferdescriptors"]
        static mut usbbufferdescriptors : [::usb::BufferDescriptor; 20];
    }

    #[allow(non_snake_case, dead_code)]
    #[inline(always)]
    pub fn BufferDescriptors() -> &'static mut [::usb::BufferDescriptor] {
        unsafe { &mut usbbufferdescriptors[..] }
    }

        

    pub const ENDPOINTCONFIG_FOR_REGISTERS: &'static [Usb_endpt_endpt] = &[
		Usb_endpt_endpt::from_raw(0b00000000),
		Usb_endpt_endpt::from_raw(0b00000000),
		Usb_endpt_endpt::from_raw(0b00010101),
		Usb_endpt_endpt::from_raw(0b00011001),
		Usb_endpt_endpt::from_raw(0b00010101),
		Usb_endpt_endpt::from_raw(0b00000000),
		Usb_endpt_endpt::from_raw(0b00000000),
		Usb_endpt_endpt::from_raw(0b00000000),
		Usb_endpt_endpt::from_raw(0b00000000),
		Usb_endpt_endpt::from_raw(0b00000000),
		Usb_endpt_endpt::from_raw(0b00000000),
		Usb_endpt_endpt::from_raw(0b00000000),
		Usb_endpt_endpt::from_raw(0b00000000),
		Usb_endpt_endpt::from_raw(0b00000000),
		Usb_endpt_endpt::from_raw(0b00000000),
		Usb_endpt_endpt::from_raw(0b00000000),
    ];
    #[allow(non_snake_case, dead_code)]
    #[inline(always)]
    pub fn EndpointconfigForRegisters() -> &'static [Usb_endpt_endpt] {
        unsafe { &mut ENDPOINTCONFIG_FOR_REGISTERS }
    }

    use usbmempool::{MemoryPool, UsbPacket};

    static mut POOL : Option<MemoryPool<[UsbPacket; 32]>> = None;
    pub fn pool_ref() -> &'static MemoryPool<[UsbPacket; 32]> {
        let r = unsafe { &mut POOL };
        if r.is_none() {
            *r = Some(MemoryPool::new());
        }
        &r.as_ref().unwrap()
    }

    use usbserial::UsbSerial;
    use usbdriver::UsbDriver;

    static mut USBDRIVER : Option<UsbSerial> = None;

    pub fn usb_ref() -> &'static mut UsbSerial {
        let r = unsafe { &mut USBDRIVER };
        if r.is_none() {
            *r = Some(UsbSerial::new(
                UsbDriver::new(pool_ref(),
                               DEVICEDESCRIPTOR,
                               CONFIGDESCRIPTORTREE,
                               get_str,
                               BufferDescriptors(),
                               ENDPOINTCONFIG_FOR_REGISTERS
                                )));
        }
        r.as_mut().unwrap()
    }

    #[allow(dead_code)]
    #[no_mangle]
    pub unsafe extern "C" fn isr_usb() {
        usb_ref().isr();
    }



	use main as user_entry_function;

	#[start]
	fn generated_start(_: isize, _: *const *const u8) -> isize {
		startup();
		user_entry_function();
		0
	}

use zinc::hal::mem_init;
use zinc::hal::k20::watchdog;
use zinc::hal::k20::regs::*;
use zinc::hal::k20::clocks;
use zinc::hal::k20::rtc;
use zinc::hal::cortex_m4::systick;

#[inline(always)]
pub fn startup() {

        watchdog::init(watchdog::State::Disabled);

    
        mem_init::init_stack();
        mem_init::init_data();
        

        SIM().scgc6.ignoring_state()
            .set_rtc(Sim_scgc6_rtc::Enabled)           // Allow access to RTC module
            .set_ftfl(Sim_scgc6_ftfl::ClockEnabled);   // Enable clock for flash memory

        // if the RTC oscillator isn't enabled, get it started early
        rtc::init();

        // release I/O pins hold, if we woke up from VLLS mode
        // if (PMC_REGSC & PMC_REGSC_ACKISO) PMC_REGSC |= PMC_REGSC_ACKISO;
        if PMC().regsc.ackiso() == Pmc_regsc_ackiso::Latched {
            PMC().regsc.set_ackiso(Pmc_regsc_ackiso::Latched);
        }

        // since this is a write once register, make it visible to all F_CPU's
        // so we can into other sleep modes in the future at any speed
        // SMC_PMPROT = SMC_PMPROT_AVLP | SMC_PMPROT_ALLS | SMC_PMPROT_AVLLS;
        SMC().pmprot.ignoring_state()
            .set_avlp(Smc_pmprot_avlp::Allowed)
            .set_alls(Smc_pmprot_alls::Allowed)
            .set_avlls(Smc_pmprot_avlls::Allowed);

        // // enable capacitors for crystal
        // OSC0_CR = OSC_SC8P | OSC_SC2P | OSC_ERCLKEN;
        OSC().cr
            .ignoring_state()
            .set_sc8p(true)
            .set_sc2p(true)
            .set_er(Osc_cr_er::Enabled);

        // enable osc, 8-32 MHz range, low power mode
        // MCG_C2 = MCG_C2_RANGE0(2) | MCG_C2_EREFS;
        MCG().c2
            .ignoring_state()
            .set_range0(Mcg_c2_range0::VeryHigh)
            .set_erefs0(Mcg_c2_erefs0::Oscillator);
        // switch to crystal as clock source, FLL input = 16 MHz / 512
        // MCG_C1 =  MCG_C1_CLKS(2) | MCG_C1_FRDIV(4);
        MCG().c1
            .ignoring_state()
            .set_clks(Mcg_c1_clks::External)
            .set_frdiv(4);

        // wait for crystal oscillator to begin
        wait_for!(MCG().status.oscinit0() == Mcg_status_oscinit0::Initialized);
        // wait for FLL to use oscillator
        wait_for!(MCG().status.irefst() == Mcg_status_irefst::External);
        // wait for MCGOUT to use oscillator
        wait_for!(MCG().status.clkst() == Mcg_status_clkst::External);


        // config PLL input for 16 MHz Crystal / 6 = 2.667 Hz
        MCG().c5.ignoring_state().set_prdiv0(5);

        // config PLL for 72 MHz output
        MCG().c6.ignoring_state()
            .set_plls(Mcg_c6_plls::PLL)
            .set_vdiv0(3);

        // wait for PLL to start using xtal as its input
        wait_for!(MCG().status.pllst() == Mcg_status_pllst::PLL);
        // wait for PLL to lock
        wait_for!(MCG().status.lock0() == Mcg_status_lock0::Locked);
        // now we're in PBE mode

        // config divisors: 72 MHz core, 36 MHz bus, 24 MHz flash, USB = 72 * 2 / 3
        // SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(1) | SIM_CLKDIV1_OUTDIV4(2);
        SIM().clkdiv1.ignoring_state()
            .set_outdiv1(0)
            .set_outdiv2(1)
            .set_outdiv4(2);
        // SIM_CLKDIV2 = SIM_CLKDIV2_USBDIV(2) | SIM_CLKDIV2_USBFRAC;
        SIM().clkdiv2.ignoring_state()
            .set_usbdiv(2)
            .set_usbfrac(true);

        // switch to PLL as clock source, FLL input = 16 MHz / 512
        // MCG_C1 = MCG_C1_CLKS(0) | MCG_C1_FRDIV(4);
        MCG().c1.ignoring_state()
            .set_clks(Mcg_c1_clks::PLLS)
            .set_frdiv(4);
        // wait for PLL clock to be used
        wait_for!(MCG().status.clkst() == Mcg_status_clkst::PLL);
        // now we're in PEE mode

        // USB uses PLL clock, trace is CPU clock, CLKOUT=OSCERCLK0
        // SIM_SOPT2 = SIM_SOPT2_USBSRC | SIM_SOPT2_PLLFLLSEL | SIM_SOPT2_TRACECLKSEL
        //      | SIM_SOPT2_CLKOUTSEL(6);
        SIM().sopt2.ignoring_state()
            .set_usbsrc(Sim_sopt2_usbsrc::PllFll)
            .set_pllfllsel(Sim_sopt2_pllfllsel::Pll)
            .set_traceclksel(Sim_sopt2_traceclksel::SystemClock)
            .set_clkoutsel(Sim_sopt2_clkoutsel::OscERClk0);

        // Record the clock frequencies we've just set up.
        clocks::set_system_clock(72_000_000);
        clocks::set_bus_clock(36_000_000);
        clocks::set_flash_clock(24_000_000);
        // USB is at 48_000_000



        // The CLKSOURCE bit in SysTick Control and Status register is
        // always set to select the core clock.
        // Because the timing reference (FCLK) is a variable frequency, the TENMS bit in the
        // SysTick Calibration Value Register is always zero.
        // Set tick freq to 1 ms
        systick::setup(72_000_000/1000); //
        systick::enable();

}

