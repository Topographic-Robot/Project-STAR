use esp_hal::{clock::ClockControl, delay::Delay, peripherals::Peripherals, system::SystemControl};
use esp_println::println;

const DR_REG_GPIO_BASE: u32 = 0x3FF44000;
const GPIO_ENABLE_W1TS_REG: *mut u32 = (DR_REG_GPIO_BASE + 0x0020) as *mut u32;
const GPIO_OUT_W1TS_REG: *mut u32 = (DR_REG_GPIO_BASE + 0x0008) as *mut u32;
const GPIO_OUT_W1TC_REG: *mut u32 = (DR_REG_GPIO_BASE + 0x000C) as *mut u32;
const LED_PIN: u32 = 2;

pub fn low_level_approach() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let delay = Delay::new(&clocks);

    println!("Low-Level Approach");

    unsafe {
        GPIO_ENABLE_W1TS_REG.write_volatile(1 << LED_PIN); // Set LED_PIN as output
    }

    loop {
        unsafe {
            println!("led on");
            GPIO_OUT_W1TS_REG.write_volatile(1 << LED_PIN); // Turn LED on
        }
        delay.delay_millis(1000u32);

        unsafe {
            println!("led off");
            GPIO_OUT_W1TC_REG.write_volatile(1 << LED_PIN); // Turn LED off
        }
        delay.delay_millis(1000u32);
    }
}
