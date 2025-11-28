fn main() {
    // Indica a Cargo que debe recompilar si cambia el archivo PIO
    println!("cargo:rerun-if-changed=src/pwm.pio");
}