# Install rust
```shell
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```
> Output
> 
> Welcome to Rust!
> 
> This will download and install the official compiler for the Rust
> programming language, and its package manager, Cargo.
> 
> Rustup metadata and toolchains will be installed into the Rustup
> home directory, located at:
> 
>   /home/undefined/.rustup
> 
> This can be modified with the RUSTUP_HOME environment variable.
> 
> The Cargo home directory is located at:
> 
>   /home/undefined/.cargo
> 
> This can be modified with the CARGO_HOME environment variable.
> 
> The cargo, rustc, rustup and other commands will be added to
> Cargo's bin directory, located at:
> 
>   /home/undefined/.cargo/bin
> 
> This path will then be added to your PATH environment variable by
> modifying the profile files located at:
> 
>   /home/undefined/.profile
>   /home/undefined/.bashrc
>   /home/undefined/.config/zsh/.zshenv
> 
> You can uninstall at any time with rustup self uninstall and
> these changes will be reverted.
> 
> Current installation options:
> 
> 
>    default host triple: x86_64-unknown-linux-gnu
>      default toolchain: stable (default)
>                profile: default
>   modify PATH variable: yes
> 
> 1) Proceed with standard installation (default - just press enter)
> 2) Customize installation
> 3) Cancel installation

# Update Rust
```shell
source ~/.cargo/env
rustup update
```

# Install toolchain for ARM Cortex-M0+
Un target de rust siempre describe la arquitectura, el nivel del procesasor, el sistema operativo y el ABI.
- thumb - Indica las instrucciones ARM Thumb. La familia ARM Cortex-M0+ sólo soporta Thumb, que son instrucciones más compactas.
- v6m - Indica la versión del ISA ARM
- none - No hay sistema operativo
- eabi - ABI (Application Binary Interface) para ARM. Define cómo se pasan los argumentos en funciones, como se alinean las estrucutras, como se organiza la memoria, como funcionan las interrupciones.

```shell
rustup target add thumbv6m-none-eabi
```

# Init cargo
```shell
cargo init .
```

# Modify the Cargo.toml

# Create .cargo dir
```shell
mkdir .cargo
```

# Create a .cargo/config.toml
Con los siguientes contenidos
```toml
[buid]
target = "thumbv6m-none-eabi"

[target.thumbv6m-none-eabi]
rustflags = [
    "-C", "link-arg=-Tmemory.x",
]
```

# Create a memory.x
Con los siguientes contenidos
```text
MEMORY {
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
    FLASH : ORIGIN = 0x10000100, LENGTH = 2048K - 0x100
    RAM   : ORIGIN = 0x20000000, LENGTH = 256K
}

EXTERN(BOOT2_FIRMWARE)

SECTIONS {
    /* ### Boot loader */
    .boot2 ORIGIN(BOOT2) :
    {
        KEEP(*(.boot2));
    } > BOOT2
} INSERT BEFORE .text;
```

# Install tool to generate ELF 2 UF2
ELF (Executable and Linkable Format): formato estándar que generan compiladores como GCC, Clang o Rust cuando producen ejecutables. Contiene: código máquina, tablas de símbolos, dirección de memoria donde debe ir cada sección, tablas de depuración (DWARF), información sobre los segmentos y secciones y metadatos para el linker.

> La RP2040 no sabe interpretar EFL. No se carga el ELF directamente en el microcontrolador. En cambio se usa para que el depurador entienda el programa, el linker lo convierta en un binario plano. El bootloader de las Raspberry Pi Pico necesita un formato más simple para cargar el firmware mediante USB.

UF2 (USB Flashing Format 2): formato creado por Microsoft para programar microcontroladores fácilmente usando drag&drop como si fueran una memoria USB.

```shell
cargo install elf2uf2-rs
```

# Compilar
```shell
cargo build --release
```

# Convert to UF2
```shell
elf2uf2-rs target/thumbv6m-none-eabi/release/pulsevision pulsevision.uf2
```

# Debugging with Rasbperry Pi Debug Probe

Connect 3-pin to 3-pin debug cable
Connect UART cables
Install Probe-rs. This command will download, compile and install probe-rs, cargo-flash and cargo-embed.
```shell
cargo install probe-rs-tools --locked
```
Install flip-link linker wrapper. It detects subtle errors at linking.

Install vscode extensions.
- Rust Analyzer
- probe-rs Debugger