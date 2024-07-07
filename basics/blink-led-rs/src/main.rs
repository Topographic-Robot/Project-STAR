#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::entry;

mod components;
use components::high_level_approach::high_level_approach;
use components::low_level_approach::low_level_approach;
use components::medium_level_approach::medium_level_approach;

#[entry]
fn main() -> ! {
    // high_level_approach();
    // medium_level_approach();
    low_level_approach();
}
