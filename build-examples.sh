#!/bin/bash
set -e

echo "Building all examples..."
cargo build --release --examples

echo ""
echo "Converting examples to hex format..."

# List of all examples
examples=(
    "01_basic_host_init"
    "02_device_enumeration"
    "03_qwerty_keyboard"
    "04_midi_keyboard"
    "05_multi_device_manager"
    "hid_gamepad"
    "mass_storage"
)

for example in "${examples[@]}"; do
    echo "  Converting $example..."
    cargo objcopy --release --example "$example" -- -O ihex "$example.hex"
done

echo ""
echo "Build Summary:"
echo "=============================================="
ls -lah *.hex
echo ""
echo "Total size: $(du -sh *.hex | awk '{s+=$1} END {print s}')"
echo "Ready to flash with: teensy_loader_cli --mcu=TEENSY41 -w <filename>.hex"
