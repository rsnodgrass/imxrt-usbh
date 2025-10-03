use std::env;

fn main() {
    // Only apply linker script for thumbv7em-none-eabihf target (Teensy 4.x)
    let target = env::var("TARGET").unwrap();
    if target.starts_with("thumbv7em-none-eabihf") {
        println!("cargo:rerun-if-changed=build.rs");

        // teensy4-bsp provides t4link.x which includes the memory layout
        // and boot configuration for i.MX RT1062 (Teensy 4.x)
        println!("cargo:rustc-link-arg=-Tt4link.x");
    }
}
