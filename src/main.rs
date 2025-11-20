#![no_std]      // Don't use the Rust standard library
#![no_main]     // Don't use the normal main entry point provided by std

use rp_pico::entry; // Entry point macro for bare-metal programs
use rp_pico::hal;   // Import the HAL (Hardware Abstraction Layer)
use rp_pico::hal::clocks::Clock;
use embedded_hal::digital::OutputPin; // Trait for controlling digital output pins
use cortex_m::Peripherals as CorePeripherals;
use panic_halt as _; // Panic handler that simply halts the CPU

#[entry] // Marks the function that will run at startup
fn main() -> ! {

    // Peripherals::take() gives access to all hardware blocks (GPIO, clocks)
    let mut pac = rp_pico::pac::Peripherals::take().unwrap();

    let core = CorePeripherals::take().unwrap();

    // Initialize system clocks
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Initialize PLLs and clock tree using Pico's 12 MHz crystal oscillator
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ, // 12 MHz external crystal
        pac.XOSC,                   // Crystal oscillator block
        pac.CLOCKS,                 // Clock control block
        pac.PLL_SYS,                // System PLL
        pac.PLL_USB,                // USB PLL
        &mut pac.RESETS,            // Reset controller
        &mut watchdog               // Watchdog
    )
    .ok()
    .unwrap(); // If any part fails, panic

    // Configure GPIO pins
    // SIO (Single-Cycle IO) gives fast access to each pin
    let sio = hal::Sio::new(pac.SIO);

    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,              // Actual GPIO hardware block
        pac.PADS_BANK0,            // GPIO pad configuration (pull ups,...)
        sio.gpio_bank0,           // SIO control of GPIO
        &mut pac.RESETS            // Reset controller
    );

    // Configure LED pin as output
    // On Raspberry Pi Pico, the onboard LED is connected to GPIO25.
    let mut led_pin = pins.led.into_push_pull_output();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    // Main loop
    loop {
        // Turn LED ON
        led_pin.set_high().unwrap();

        // Busy-wait delay. The RR2040 executes ~1 instruction per cycle
        // at default clock speeds
        delay.delay_ms(1000);

        // Turn LED OFF
        led_pin.set_low().unwrap();

        // Delay again
        delay.delay_ms(500);
    }
}
