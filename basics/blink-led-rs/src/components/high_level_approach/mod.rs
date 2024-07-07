use esp_hal::clock::ClockControl;
use esp_hal::delay::Delay;
use esp_hal::gpio::Io;
use esp_hal::gpio::Level;
use esp_hal::gpio::Output;
use esp_hal::peripherals::Peripherals;
use esp_hal::system::SystemControl;
use esp_println::println;

pub fn high_level_approach() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    println!("High-Level Approach");

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut led = Output::new(io.pins.gpio2, Level::Low);

    let delay = Delay::new(&clocks);

    loop {
        led.set_high();
        delay.delay_millis(1000u32);
        led.set_low();
        delay.delay_millis(1000u32);
    }
}
