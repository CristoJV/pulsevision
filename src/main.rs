#![no_std]      // Don't use the Rust standard library
#![no_main]     // Don't use the normal main entry point provided by std

use rp_pico::entry;

// Entry point macro for bare-metal programs
use rp_pico::hal as hal;   // Import the HAL (Hardware Abstraction Layer)
use hal::clocks::Clock;
use hal::gpio::{FunctionPio0, Pin};
use hal::pac;
use hal::Sio;
use hal::pio::{PIOExt, PIOBuilder};
use pio::{Instruction, MovDestination, MovSource, SideSet};
use hal::timer::Timer;

use panic_probe as _; // Panic handler that simply halts the CPU
use defmt::*;
use defmt_rtt as _;
use libm::sinf;
use pio_proc::{pio_file, pio_asm};
use embedded_hal::delay::DelayNs;


fn config_pio_clock_divisor_for(pulse_freq: f32, pulse_cycles: u8, fsys: f32) -> (u16, u8){
    let divisor: f32 = fsys / ((pulse_cycles as f32) * pulse_freq);
    let int= divisor as u16;
    let frac: u8 = ((divisor - int as f32) * 256.0) as u8;
    debug!("System freq={}", fsys);
    debug!("Divisor={}, ({} + {}/256)", divisor, int, frac);
    debug!("Configured pulse freq={}", fsys / ((int as f32) + (frac as f32) /256.0) / (pulse_cycles as f32));
    return (int, frac)
}


fn generate_sine_table<const N: usize>() -> [u32; N] {
    let mut table = [0u32; N];
    for i in 0..N {
        let angle = (i as f32) * core::f32::consts::TAU / N as f32;
        let s = sinf(angle);
        table[i] = ((s * 0.5 + 0.5) * (N as f32 - 1.0)) as u32;
    }
    table
}

const TABLE_SIZE: usize = 256;

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

    // Initialize and start PIO
    let program = pio_file!("src/pwm.pio");
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let installed = pio.install(&program.program).unwrap();
    
    // PIO Clock divisor
    // FPIO = FSYS (125MHz) / (int + frac / 256)
    let fsys = clocks.system_clock.freq().to_Hz() as f32;
    let (int, frac) = config_pio_clock_divisor_for(10_000.0, 32,fsys);

    let (mut sm, _, mut tx) = PIOBuilder::from_installed_program(installed)
        .set_pins(led_pin_id, 1)
        .side_set_pin_base(led_pin_id)
        .clock_divisor_fixed_point(int, frac)
        .build(sm0);

    let period: u32 = (TABLE_SIZE as u32) -1;
    tx.write(period);     // TX FIFO ← PERIOD
    let pull = pio_asm!("pull noblock","out isr, 32").program;
    let pull_instr = Instruction::decode(pull.code[0], SideSet::new(false, 0, false)).unwrap();
    let mov_isr_osr = Instruction::decode(pull.code[1], SideSet::new(false, 0, false)).unwrap();
    sm.exec_instruction(pull_instr);
    sm.exec_instruction(mov_isr_osr);
    
    sm.start();

    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let mut sine_table = generate_sine_table::<TABLE_SIZE>();
    loop {
        // for duty in 0..31 {
        for &duty in sine_table.iter() {
            tx.write(duty as u32);
            timer.delay_ns(39000_000);
        }
    }
}
