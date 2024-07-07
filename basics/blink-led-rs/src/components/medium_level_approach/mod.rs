use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    peripherals::{Peripherals, GPIO},
    system::SystemControl,
};
use esp_println::println;

const LED_PIN: u32 = 2;

pub fn medium_level_approach() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    println!("Medium-Level Approach");

    // Set LED_PIN as output
    let gpio = unsafe { &*GPIO::ptr() };
    unsafe {
        gpio.enable_w1ts().write(|w| w.bits(1 << LED_PIN));
        // Use the appropriate method to configure the pin
        let pin = gpio.pin(LED_PIN as usize);
        pin.write(|w| w.pad_driver().clear_bit()); // Set push-pull mode
        gpio.func_out_sel_cfg(LED_PIN as usize)
            .write(|w| w.bits(256)); // GPIO_FUNC_OUT_SEL
    }

    let delay = Delay::new(&clocks);

    loop {
        // Set pin high
        unsafe {
            gpio.out_w1ts().write(|w| w.bits(1 << LED_PIN));
        }
        delay.delay_millis(1000u32);
        // Set pin low
        unsafe {
            gpio.out_w1tc().write(|w| w.bits(1 << LED_PIN));
        }
        delay.delay_millis(1000u32);
    }
}
