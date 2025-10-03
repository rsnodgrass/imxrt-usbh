use std::env;

fn main() {
    // Only apply linker script for thumbv7em-none-eabihf target (Teensy 4.x)
    let target = env::var("TARGET").unwrap();
    if target.starts_with("thumbv7em-none-eabihf") {
        // Tell cargo to look for linker script in teensy4-bsp package
        // This is provided by teensy4-bsp when the "rt" feature is enabled
        println!("cargo:rerun-if-changed=build.rs");

        // Memory layout for i.MX RT1062 (Teensy 4.1)
        // - ITCM: 512KB (instruction tightly-coupled memory)
        // - DTCM: 512KB (data tightly-coupled memory)
        // - OCRAM: 512KB (on-chip RAM)
        // - External RAM: 8MB PSRAM on Teensy 4.1
        // - Flash: 8MB on Teensy 4.1

        // teensy4-bsp provides the memory.x and link.x scripts
        // These are automatically included when using teensy4-bsp with rt feature
    }
}
