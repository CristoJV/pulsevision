#![no_std] // Don't use the Rust standard library
#![no_main] // Don't use the normal main entry point provided by std

use rp_pico::entry;

use rp_pico::hal::pio::ValidStateMachine;
// Entry point macro for bare-metal programs
use defmt::*;
use defmt_rtt as _;
use hal::clocks::Clock;
use hal::gpio::{FunctionPio0, Pin};
use hal::pac;
use hal::pio::{PIOBuilder, PIOExt, Running, StateMachine, Stopped, Tx};
use hal::Sio;
use libm::sinf;
use panic_probe as _; // Panic handler that simply halts the CPU
use pio::{Instruction, SideSet};
use pio_proc::{pio_asm, pio_file};
use rp_pico::hal; // Import the HAL (Hardware Abstraction Layer)

fn config_pio_clock_divisor_for(
    pulse_freq: f32,
    pulse_cycles: u16,
    fsys: f32,
) -> (u16, u8) {
    let divisor: f32 = fsys / ((pulse_cycles as f32) * pulse_freq);
    let int = divisor as u16;
    let frac: u8 = ((divisor - int as f32) * 256.0) as u8;
    debug!("System freq={}", fsys);
    debug!("Divisor={}, ({} + {}/256)", divisor, int, frac);
    debug!(
        "Configured pulse freq={}",
        fsys / ((int as f32) + (frac as f32) / 256.0) / (pulse_cycles as f32)
    );
    (int, frac)
}

fn generate_sine_table<const N: usize>(max_value: u32) -> [u32; N] {
    let mut table = [0u32; N];
    for i in 0..N {
        let angle = (i as f32) * core::f32::consts::TAU / N as f32;
        let s = sinf(angle);
        table[i] = ((s * 0.5 + 0.5) * (max_value as f32)) as u32;
    }
    table
}

fn pio_set_pindirs<SM: ValidStateMachine>(sm: &mut StateMachine<SM, Stopped>) {
    let instructions = pio_asm!("set pindirs, 1").program;
    sm.exec_instruction(
        Instruction::decode(instructions.code[0], SideSet::new(false, 0, false))
            .unwrap(),
    );
}

fn pio_load_pwm_cycles<SM: ValidStateMachine>(
    sm: &mut StateMachine<SM, Stopped>,
    tx: &mut Tx<SM>,
    n_cycles: u32,
) {
    let instructions = pio_asm!("pull noblock", "out isr, 32").program;
    tx.write(n_cycles);
    for &code in instructions.code.iter() {
        let instr: Instruction =
            Instruction::decode(code, SideSet::new(false, 0, false)).unwrap();
        sm.exec_instruction(instr);
    }
}

fn pio_set_wait_for_irq_state<SM: ValidStateMachine>(
    sm: &mut StateMachine<SM, Stopped>,
) {
    let instructions = pio_asm!("wait 1 irq 0").program;
    sm.exec_instruction(
        Instruction::decode(instructions.code[0], SideSet::new(false, 0, false))
            .unwrap(),
    );
}

fn pio_set_irq_flag<SM: ValidStateMachine>(sm: &mut StateMachine<SM, Running>) {
    let instructions = pio_asm!("irq set 0").program;
    sm.exec_instruction(
        Instruction::decode(instructions.code[0], SideSet::new(false, 0, false))
            .unwrap(),
    );
}

const PWM_FREQ: u32 = 150_000;
const DAC_FREQ: u32 = 10000;

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

    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let sio = Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,    // Actual GPIO hardware block
        pac.PADS_BANK0,  // GPIO pad configuration (pull ups,...)
        sio.gpio_bank0,  // SIO control of GPIO
        &mut pac.RESETS, // Reset controller
    );

    // Configure LED pin for Pio0 (Using GPIO15)
    let led: Pin<_, FunctionPio0, _> = pins.gpio15.into_function();
    let led_pin_id = led.id().num;
    // Configure SYNC pin for Pio0 (Using GPIO16)
    let pin_sync: Pin<_, FunctionPio0, _> = pins.gpio16.into_function();
    let pin_sync_id = pin_sync.id().num;

    // Initialize and start PIO
    let (mut pio, sm0, sm1, _, _) = pac.PIO0.split(&mut pac.RESETS);

    // ------------------------- PWM generator --------------------------------
    // Generates a PWM signal with dynamic duty
    let pwm_program = pio_file!("src/pwm.pio");
    let pwm_installed = pio.install(&pwm_program.program).unwrap();

    // PIO Clock divisor
    // FPIO = FSYS (125MHz) / (int + frac / 256) / n_cycles
    let fsys = clocks.system_clock.freq().to_Hz() as f32;
    let pwm_total_cycles: u16 = 3 * 256; // 256 cycles for PWM (only 254 available)
    let (pwm_int, pwm_frac) =
        config_pio_clock_divisor_for(PWM_FREQ as f32, pwm_total_cycles, fsys);
    let (mut sm, _, mut tx) = PIOBuilder::from_installed_program(pwm_installed)
        .set_pins(led_pin_id, 2)
        .jmp_pin(pin_sync_id)
        .side_set_pin_base(led_pin_id)
        .clock_divisor_fixed_point(pwm_int, pwm_frac)
        .build(sm0);
    pio_set_pindirs(&mut sm);
    pio_load_pwm_cycles(&mut sm, &mut tx, pwm_total_cycles as u32);
    pio_set_wait_for_irq_state(&mut sm);

    let mut sm = sm.start();

    // ------------------------- DAC sampler --------------------------------
    // Updates the pwm duty cycle at a predefined sampling frequency

    let dac_program = pio_asm!(
        "
        wait_clear:
        wait 0 pin 0
        send_pulse:
        set pins, 1 [31]
        jmp wait_clear [31]
    "
    );
    let dac_total_cycles = 65;
    let dac_installed = pio.install(&dac_program.program).unwrap();
    let (int1, frac1) =
        config_pio_clock_divisor_for(DAC_FREQ as f32, dac_total_cycles, fsys);
    let (mut sm1, _, _) = PIOBuilder::from_installed_program(dac_installed)
        .set_pins(pin_sync_id, 1)
        .clock_divisor_fixed_point(int1, frac1)
        .build(sm1);

    pio_set_pindirs(&mut sm1);
    pio_set_wait_for_irq_state(&mut sm1);

    sm1.start();

    // Generates a 1Hz sinusoid signal at 10000 samples per second
    let sine_table = generate_sine_table::<10000>(254);

    // Launch synchronization signal between SMs
    pio_set_irq_flag(&mut sm);
    loop {
        //TODO: Writes as fast as possible -> Use DMA instead
        for &duty in sine_table.iter() {
            while tx.is_full() {
                cortex_m::asm::nop();
            }
            tx.write(duty);
        }
    }
}
