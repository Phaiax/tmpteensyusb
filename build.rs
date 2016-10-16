
extern crate platformtreeb;
use platformtreeb::{start, McuType, Bytes, k20};


fn main() {
    start(McuType::Mk20dx256vlh7)
        .call_user_startup_function("main")
        .mcu()
        .with(k20::predefined::teensy::teensy_configurator)
        .set_eeprom(Bytes::b(32), Bytes::k(32), k20::EEESplit::Unsupported)
        .enable_usb(k20::usb::UsbConfig::new(k20::usbserial::make_device_tree_for_teensy_serial()))
        .execute();
}



