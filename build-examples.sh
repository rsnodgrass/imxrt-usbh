#!/bin/bash
set -e

echo "Building all examples for Teensy 4.1..."
cargo build --release --target thumbv7em-none-eabihf --examples

echo ""
echo "Converting examples to hex format..."

# Basic examples (Simple API)
basic_examples=(
    "01_hello_keyboard"
    "02_device_info"
    "03_hid_mouse"
    "04_hid_gamepad"
    "05_typing_game"
)

# Advanced examples (Low-level API)
advanced_examples=(
    "a01_phy_initialization"
    "a02_manual_enumeration"
    "a03_midi_keyboard"
    "a04_multi_device_manager"
    "a05_mass_storage"
    "a06_hid_report_protocol"
    "a07_qwerty_keyboard"
)

echo "Basic examples:"
for example in "${basic_examples[@]}"; do
    echo "  Converting $example..."
    cargo objcopy --release --target thumbv7em-none-eabihf --example "$example" -- -O ihex "$example.hex"
done

echo ""
echo "Advanced examples:"
for example in "${advanced_examples[@]}"; do
    echo "  Converting $example..."
    cargo objcopy --release --target thumbv7em-none-eabihf --example "$example" -- -O ihex "advanced_$example.hex"
done

echo ""
echo "Build Summary:"
echo "=============================================="
ls -lah *.hex
echo ""
echo "Basic examples (Simple API):"
echo "  01_hello_keyboard  - Simple keyboard input (no logging)"
echo "  02_device_info     - Device enumeration with USB CDC logging"
echo "  03_hid_mouse       - Mouse input (no logging)"
echo "  04_hid_gamepad     - Gamepad input with USB CDC logging"
echo "  05_typing_game     - Interactive typing test with USB CDC logging"
echo ""
echo "Advanced examples (Low-level API):"
echo "  a01_phy_initialization    - Low-level PHY setup"
echo "  a02_manual_enumeration    - Manual device enumeration"
echo "  a03_midi_keyboard         - MIDI device support"
echo "  a04_multi_device_manager  - Multiple device handling"
echo "  a05_mass_storage          - Mass storage class"
echo "  a06_hid_report_protocol   - HID report descriptor parsing"
echo "  a07_qwerty_keyboard       - Full QWERTY keyboard mapping"
echo ""
echo "Total size: $(du -sh *.hex | tail -1 | awk '{print $1}')"
echo ""
echo "Flash with: teensy_loader_cli --mcu=TEENSY41 -w <filename>.hex"
echo ""
echo "Note: Examples 02, 04, and 05 output to USB1 (micro USB) for logging."
echo "      All examples use USB2 (pins 30/31) for USB host functionality."
