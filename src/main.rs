#![no_std]      // Don't use the Rust standard library
#![no_main]     // Don't use the normal main entry point provided by std

use rp_pico::entry; // Entry point macro for bare-metal programs
use rp_pico::hal as hal;   // Import the HAL (Hardware Abstraction Layer)
use hal::gpio::{FunctionPio0, Pin};
use hal::pac;
use hal::Sio;
use hal::pio::PIOExt;

// Import PIO crates
use hal::pio::PIOBuilder;
use pio::{Assembler, SetDestination};

use cortex_m;
use panic_halt as _; // Panic handler that simply halts the CPU

#[entry] // Marks the function that will run at startup
fn main() -> ! {

    // Peripherals::take() gives access to all hardware blocks (GPIO, clocks)
    let mut pac = pac::Peripherals::take().unwrap();
    let sio = Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,              // Actual GPIO hardware block
        pac.PADS_BANK0,            // GPIO pad configuration (pull ups,...)
        sio.gpio_bank0,            // SIO control of GPIO
        &mut pac.RESETS            // Reset controller
    );

    // Configure LED pin for Pio0
    let led: Pin <_, FunctionPio0, _> = pins.gpio25.into_function();
    // Get PIN id for use inside PIO
    let led_pin_id = led.id().num;

    const MAX_DELAY: u8 = 31;
    let mut ass = Assembler::<32>::new();
    let mut wrap_target = ass.label();
    let mut wrap_source = ass.label();
    // Set pin as output
    ass.set(SetDestination::PINDIRS, 1);
    // Define begin of program loop
    ass.bind(&mut wrap_target);
    // Set pin low
    ass.set_with_delay(SetDestination::PINS,0,MAX_DELAY);
    // Set pin high
    ass.set_with_delay(SetDestination::PINS,1,MAX_DELAY);
    // Define end of program loop
    ass.bind(&mut wrap_source);

    let program = ass.assemble_with_wrap(wrap_source, wrap_target);

    // Initialize and start PIO
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio.install(&program).unwrap();
    let (int, frac) = (0, 0);
    let (sm, _, _) = PIOBuilder::from_installed_program(installed)
        .set_pins(led_pin_id, 1)
        .clock_divisor_fixed_point(int, frac)
        .build(sm0);
    sm.start();
 
    loop {
        cortex_m::asm::wfi();
    }
}
