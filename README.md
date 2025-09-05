imxrt-usbh
=========

**Work in progress - not ready for use yet**

A USB host driver for i.MX RT processors. `imxrt-usbh` provides EHCI USB host functionality
for the i.MX RT1062 microcontroller family, targeting Teensy 4.0/4.1 boards with RTIC support.

## Development Requirements

- **LLD Linker**: Install LLD for faster compilation times:
  ```bash
  brew install lld
  ```

- **Rust Target**: The embedded ARM Cortex-M7 target:
  ```bash
  rustup target add thumbv7em-none-eabihf
  ```

## Architecture

- **EHCI USB Host**: Full EHCI 1.0 compliant implementation
- **RTIC Integration**: Real-time interrupt handling with <10μs ISR latency
- **Cache Coherent DMA**: ARM Cortex-M7 cache management for USB transfers
- **Safety First**: `#![deny(unsafe_op_in_unsafe_fn)]` with comprehensive bounds checking


License
-------

Licensed under either of

- [Apache License, Version 2.0](http://www.apache.org/licenses/LICENSE-2.0) ([LICENSE-APACHE](./LICENSE-APACHE))
- [MIT License](http://opensource.org/licenses/MIT) ([LICENSE-MIT](./LICENSE-MIT))

at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.
