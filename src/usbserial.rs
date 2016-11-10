
use usbdriver::UsbDriver;

pub struct UsbSerial {
    base : UsbDriver,
  //  #ifdef CDC_DATA_INTERFACE
  //extern uint32_t usb_cdc_line_coding[2];
  //extern volatile uint32_t usb_cdc_line_rtsdtr_millis;
  //extern volatile uint32_t systick_millis_count;
  //extern volatile uint8_t usb_cdc_line_rtsdtr;
  //extern volatile uint8_t usb_cdc_transmit_flush_timer;
  //extern void usb_serial_flush_callback(void);
  //#endif
}


impl UsbSerial {
    pub fn new(base : UsbDriver) -> UsbSerial {
        UsbSerial{
            base : base
        }
    }

    pub fn isr(&'static self) {
        self.base.isr();
    }


}