#![no_std]      // Don't use the Rust standard library
#![no_main]     // Don't use the normal main entry point provided by std

use rp_pico::entry; 
use rp_pico::hal::clocks::Clock;
// Entry point macro for bare-metal programs
use rp_pico::hal as hal;   // Import the HAL (Hardware Abstraction Layer)
use hal::gpio::{FunctionPio0, Pin};
use hal::pac;
use hal::Sio;
use hal::pio::PIOExt;

// Import PIO crates
use hal::pio::PIOBuilder;
use pio::{Assembler, SetDestination};

use panic_probe as _; // Panic handler that simply halts the CPU
use cortex_m;
use defmt::*;
use defmt_rtt as _;

fn config_pio_clock_divisor_for(pulse_freq: f32, pulse_cycles: u8, fsys: f32) -> (u16, u8){
    let divisor: f32 = fsys / ((pulse_cycles as f32) * pulse_freq);
    let int= divisor as u16;
    let frac: u8 = ((divisor - int as f32) * 256.0) as u8;
    debug!("System freq={}", fsys);
    debug!("Divisor={}, ({} + {}/256)", divisor, int, frac);
    debug!("Configured pulse freq={}", fsys / ((int as f32) + (frac as f32) /256.0) / (pulse_cycles as f32));
    return (int, frac)
}
#[entry] // Marks the function that will run at startup
fn main() -> ! {

    // Peripherals::take() gives access to all hardware blocks (GPIO, clocks)
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure system clocks
    // The internal crystal oscillator oscillates at 12 MHz.
    // This call configures internal PLLs to generate:
    // - The system clock ~ 125MHz
    // - The USB clock ~ 48MHz

    let clocks = hal::clocks::
    init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog
    )
    .ok()
    .unwrap();

    let sio = Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,              // Actual GPIO hardware block
        pac.PADS_BANK0,            // GPIO pad configuration (pull ups,...)
        sio.gpio_bank0,            // SIO control of GPIO
        &mut pac.RESETS            // Reset controller
    );

    // Configure LED pin for Pio0
    let led: Pin <_, FunctionPio0, _> = pins.gpio15.into_function();
    // Get PIN id for use inside PIO
    let led_pin_id = led.id().num;

    const MAX_DELAY: u8 = 31; // Total instruction time 1 + delay cycles
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
    
    // PIO Clock divisor
    // FPIO = FSYS (125MHz) / (int + frac / 256)
    let fsys = clocks.system_clock.freq().to_Hz() as f32;
    let (int, frac) = config_pio_clock_divisor_for(1_000.0,MAX_DELAY + 1,fsys);

    let (sm, _, _) = PIOBuilder::from_installed_program(installed)
        .set_pins(led_pin_id, 1)
        .clock_divisor_fixed_point(int, frac)
        .build(sm0);
    sm.start();
 
    loop {
        cortex_m::asm::wfi();
    }
}
