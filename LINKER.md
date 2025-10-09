# Linker Configuration for imxrt-usbh

This document explains the linker script requirements for the imxrt-usbh USB host driver library.

## Overview

The USB host driver requires DMA buffers to be placed in **on-chip RAM (OCRAM)** that is accessible by the USB DMA engine and properly configured for cache coherency.

## Using teensy4-bsp (Recommended)

If you're using the `teensy4-bsp` crate (as shown in the examples), **no custom linker script is required**. The BSP provides a complete linker script that defines all necessary memory regions including OCRAM.

Add to your `Cargo.toml`:
```toml
[dependencies]
teensy4-bsp = { version = "0.4", features = ["rt"] }
```

The `rt` feature provides the linker script automatically through the build system.

## Memory Layout

The i.MX RT1062 (Teensy 4.x) has the following memory regions:

| Region | Base Address | Size | Usage |
|--------|-------------|------|-------|
| ITCM | 0x00000000 | 128 KB | Instruction Tightly-Coupled Memory (fast code) |
| DTCM | 0x20000000 | 128 KB | Data Tightly-Coupled Memory (fast data, **non-cacheable**) |
| OCRAM | 0x20200000 | 512 KB | On-Chip RAM (cacheable, DMA-accessible) |
| Flash | 0x60000000 | 1.9 MB | External QSPI Flash (via FlexSPI) |

## DMA Region Placement

The library uses `#[link_section = ".ocram"]` to place the DMA region:

```rust
#[link_section = ".ocram"]
static mut DMA_REGION: DmaRegion = ...
```

This section **MUST** be defined in the linker script to place buffers in OCRAM at 0x20200000.

### Option 1: teensy4-bsp (Recommended)

The `teensy4-bsp` crate automatically provides the `.ocram` section definition. Just use:

```toml
[dependencies]
teensy4-bsp = { version = "0.4", features = ["rt"] }
```

### Option 2: Custom Linker Script

If you're NOT using `teensy4-bsp`, you need a custom linker script.

Create `memory.x` in your project root:

```ld
/* i.MX RT1062 Memory Layout for Teensy 4.x */
MEMORY
{
    ITCM  (rwx) : ORIGIN = 0x00000000, LENGTH = 128K
    DTCM  (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
    OCRAM (rwx) : ORIGIN = 0x20200000, LENGTH = 512K
    FLASH (rx)  : ORIGIN = 0x60000000, LENGTH = 1920K
}

REGION_ALIAS("REGION_TEXT", FLASH);
REGION_ALIAS("REGION_RODATA", FLASH);
REGION_ALIAS("REGION_DATA", DTCM);
REGION_ALIAS("REGION_BSS", DTCM);
REGION_ALIAS("REGION_HEAP", DTCM);
REGION_ALIAS("REGION_STACK", DTCM);

SECTIONS
{
    /* USB DMA buffers in OCRAM */
    .ocram (NOLOAD) : ALIGN(4096)
    {
        *(.ocram .ocram.*)
    } > OCRAM

    /* Ensure .ocram starts at proper alignment for MPU */
    . = ALIGN(4096);
} INSERT AFTER .bss;
```

**Key Points:**
- `.ocram` section uses `(NOLOAD)` to avoid initialization overhead
- 4096-byte (4KB) alignment required for MPU region configuration
- Section placed in OCRAM region starting at 0x20200000

### Verification

After building, verify the DMA region is in OCRAM:

```bash
arm-none-eabi-objdump -t target/thumbv7em-none-eabihf/release/examples/02_device_enumeration \
    | grep DMA_REGION
```

Expected output:
```
20200000 l     O .ocram 00004000 _ZN9imxrt_usbh3dma10DMA_REGION17h...
```

The address should be in the range `0x20200000 - 0x2027FFFF` (OCRAM).

## Cache Coherency

The DMA region requires proper cache configuration:

### Automatic (Recommended)

Call `init_dma_region()` at startup:

```rust
// MUST be called before any buffer allocations
unsafe {
    imxrt_usbh::dma::init_dma_region()?;
}
```

This configures MPU region 7 as non-cacheable for the OCRAM DMA region.

### Manual MPU Configuration

If you need manual control, configure MPU region for the OCRAM DMA region:

```rust
use cortex_m::peripheral::MPU;

unsafe {
    let mpu = &*MPU::PTR;

    // Select MPU region 7
    mpu.rnr.write(7);

    // Set base address (must be aligned to region size)
    mpu.rbar.write(0x20200000 | 0x10); // VALID bit

    // Configure region attributes
    mpu.rasr.write(
        (1 << 0) |      // Enable region
        (18 << 1) |     // Size = 2^(18+1) = 512KB
        (3 << 24) |     // Full access (AP)
        (1 << 16) |     // TEX=1 (non-cacheable)
        (0 << 17) |     // C=0 (non-cacheable)
        (1 << 18)       // B=1 (bufferable)
    );

    // Enable MPU
    mpu.ctrl.write(0x01);
}
```

## Common Issues

### Issue 1: DMA Buffers in Wrong Memory Region

**Symptom**: Silent data corruption, intermittent transfer failures

**Cause**: `.ocram` section not defined, buffers placed in FLASH or DTCM

**Fix**: Verify linker script includes `.ocram` section definition

### Issue 2: Misaligned DMA Region

**Symptom**: MPU configuration fails, cache coherency issues

**Cause**: `.ocram` section not aligned to 4096 bytes

**Fix**: Add `ALIGN(4096)` to section definition in linker script

### Issue 3: DMA Region Too Small

**Symptom**: `NoResources` errors when allocating buffers

**Cause**: OCRAM region too small or overlapping with other data

**Fix**: Ensure OCRAM region is at least 16KB (default: 16KB)

## Build System Integration

### Cargo Configuration

No special Cargo configuration needed if using `teensy4-bsp`.

For custom linker scripts, create `.cargo/config.toml`:

```toml
[target.thumbv7em-none-eabihf]
rustflags = [
    "-C", "link-arg=-Tmemory.x",
    "-C", "link-arg=-Tlink.x",
]
```

### Build Script

If using custom linker configuration, create `build.rs`:

```rust
use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    // Put `memory.x` in our output directory and ensure it's
    // on the linker search path.
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());

    // Only re-run if memory.x changes
    println!("cargo:rerun-if-changed=memory.x");
}
```

## References

- [i.MX RT1060 Reference Manual](https://www.nxp.com/docs/en/reference-manual/IMXRT1060RM.pdf) - Chapter 2 (Memory Map)
- [ARM Cortex-M7 Technical Reference Manual](https://developer.arm.com/documentation/ddi0489) - MPU configuration
- [teensy4-bsp documentation](https://docs.rs/teensy4-bsp/) - BSP linker script details
- [Rust Embedded Book - Memory Layout](https://docs.rust-embedded.org/book/start/memory.html)

## Quick Start Checklist

For new projects:

- [ ] Add `teensy4-bsp = { version = "0.4", features = ["rt"] }` to Cargo.toml
- [ ] Call `unsafe { imxrt_usbh::dma::init_dma_region()? }` at startup
- [ ] Verify examples compile and run
- [ ] If issues, verify `.ocram` section with `objdump` command above

For existing projects without `teensy4-bsp`:

- [ ] Create `memory.x` with `.ocram` section definition
- [ ] Create `.cargo/config.toml` with linker flags
- [ ] Create `build.rs` to provide memory.x to linker
- [ ] Call `unsafe { imxrt_usbh::dma::init_dma_region()? }` at startup
- [ ] Verify DMA region placement with `objdump`
