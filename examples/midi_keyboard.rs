//! RTIC example: USB MIDI Keyboard enumeration
//! 
//! This example demonstrates connecting a USB MIDI keyboard to the Teensy 4.x
//! and performing basic enumeration to identify the device.

#![no_std]
#![no_main]

use panic_halt as _;
use teensy4_bsp as bsp;
use teensy4_panic as _;

use imxrt_hal as hal;
use imxrt_ral as ral;

use imxrt_usbh::{
    ehci::controller::EhciController,
    phy::UsbPhy,
    enumeration::{DeviceEnumerator, EnumeratedDevice},
    dma::memory::USB_MEMORY_POOL,
    safety,
    vbus::TeensyVbus,
};

// MIDI support will come from separate imxrt-usbh-midi crate
// use imxrt_usbh_midi::{MidiDevice, MidiManager};

#[rtic::app(device = teensy4_bsp, peripherals = true)]
mod app {
    use super::*;
    use systick_monotonic::{Systick, fugit::Duration};
    
    // 1ms tick rate
    #[monotonic(binds = SysTick, default = true)]
    type Mono = Systick<1000>;
    
    #[shared]
    struct Shared {
        /// Currently enumerated device
        midi_device: Option<EnumeratedDevice>,
    }
    
    #[local]
    struct Local {
        /// USB host controller
        usb_host: EhciController<8>,
        /// USB PHY
        usb_phy: UsbPhy,
        /// LED for status indication
        led: bsp::Led,
        /// VBUS power control
        vbus: TeensyVbus,
    }
    
    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Initialize Teensy 4 board support
        let board = bsp::t40(ctx.device);
        let pins = board.pins;
        
        // Configure LED for status
        let led = bsp::configure_led(pins.p13);
        
        // Initialize USB clocks
        let ccm = unsafe { ral::ccm::CCM::instance() };
        configure_usb_clocks(&ccm);
        
        // Initialize safety monitoring (stack protection)
        unsafe {
            let stack_top = 0x2000_8000; // Adjust for your memory layout
            let stack_size = 0x4000;      // 16KB stack
            safety::init_safety_monitoring(stack_top, stack_size);
        }
        
        // Initialize USB memory pool
        unsafe {
            USB_MEMORY_POOL = UsbMemoryPool::new();
        }
        
        // Initialize USB PHY
        let mut usb_phy = UsbPhy::new(unsafe { ral::usbphy::USBPHY1::instance() });
        usb_phy.init().expect("Failed to initialize USB PHY");
        
        // Initialize USB host controller
        let usb_host = EhciController::new(
            unsafe { ral::usb::USB1::instance() },
            0x402E_0100, // USB1 base address for i.MX RT1062
        );
        usb_host.init().expect("Failed to initialize USB host");
        
        // Initialize VBUS control
        let vbus = TeensyVbus::new();
        vbus.enable_vbus().expect("Failed to enable VBUS");
        
        // Setup monotonic timer
        let mono = Systick::new(ctx.core.SYST, 600_000_000);
        
        // Schedule device detection polling
        detect_device::spawn_after(Duration::<u64, 1, 1000>::from_ticks(100))
            .expect("Failed to spawn detect_device");
        
        defmt::info!("USB MIDI host initialized, waiting for keyboard...");
        
        (
            Shared {
                midi_device: None,
            },
            Local {
                usb_host,
                usb_phy,
                led,
                vbus,
            },
            init::Monotonics(mono),
        )
    }
    
    #[task(local = [usb_host, led], shared = [midi_device])]
    fn detect_device(ctx: detect_device::Context) {
        let host = ctx.local.usb_host;
        let led = ctx.local.led;
        
        // Check for device connection
        if host.is_device_connected() {
            // Turn on LED to indicate device detected
            led.set();
            
            // Check if already enumerated
            let mut midi = ctx.shared.midi_device;
            midi.lock(|device| {
                if device.is_none() {
                    // Spawn enumeration task
                    enumerate_device::spawn()
                        .expect("Failed to spawn enumeration");
                }
            });
        } else {
            // Turn off LED
            led.clear();
            
            // Clear device if disconnected
            let mut midi = ctx.shared.midi_device;
            midi.lock(|device| {
                if device.is_some() {
                    defmt::info!("MIDI keyboard disconnected");
                    *device = None;
                }
            });
        }
        
        // Schedule next detection
        detect_device::spawn_after(Duration::<u64, 1, 1000>::from_ticks(100))
            .expect("Failed to reschedule detection");
    }
    
    #[task(local = [usb_host], shared = [midi_device])]
    fn enumerate_device(ctx: enumerate_device::Context) {
        let host = ctx.local.usb_host;
        
        // Create enumerator
        let mut memory_pool = unsafe { &mut USB_MEMORY_POOL };
        let mut enumerator = DeviceEnumerator::new(host, &mut memory_pool);
        
        // Attempt enumeration
        match enumerator.enumerate_device() {
            Ok(device) => {
                if device.is_midi {
                    defmt::info!("MIDI keyboard enumerated successfully!");
                    defmt::info!("  Address: {}", device.address);
                    defmt::info!("  VID: {:04x}", device.device_desc.id_vendor);
                    defmt::info!("  PID: {:04x}", device.device_desc.id_product);
                    defmt::info!("  Class: {:?}", device.class);
                    
                    // Store device info
                    let mut midi = ctx.shared.midi_device;
                    midi.lock(|d| *d = Some(device));
                    
                    // Start MIDI data handling
                    handle_midi_data::spawn()
                        .expect("Failed to spawn MIDI handler");
                } else {
                    defmt::warn!("Non-MIDI device connected: {:?}", device.class);
                }
            }
            Err(e) => {
                defmt::error!("Enumeration failed: {:?}", e);
                
                // Retry after delay if retryable
                if e.is_retryable() {
                    if let Some(delay_ms) = e.retry_delay_ms() {
                        let delay = Duration::<u64, 1, 1000>::from_ticks(delay_ms as u64);
                        enumerate_device::spawn_after(delay)
                            .expect("Failed to reschedule enumeration");
                    }
                }
            }
        }
    }
    
    #[task(shared = [midi_device])]
    fn handle_midi_data(ctx: handle_midi_data::Context) {
        let mut midi = ctx.shared.midi_device;
        
        midi.lock(|device| {
            if let Some(dev) = device {
                // TODO: Setup bulk endpoints for MIDI data transfer
                // TODO: Read MIDI events from IN endpoint
                // TODO: Process MIDI messages (note on/off, CC, etc)
                
                defmt::debug!("Ready to handle MIDI data from device {}", dev.address);
            }
        });
        
        // Schedule next MIDI data check
        handle_midi_data::spawn_after(Duration::<u64, 1, 1000>::from_ticks(10))
            .expect("Failed to reschedule MIDI handler");
    }
    
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }
}

/// Configure USB clocks for i.MX RT1062
fn configure_usb_clocks(ccm: &ral::ccm::CCM) {
    use ral::{modify_reg, read_reg, write_reg};
    
    unsafe {
        // Enable USB clocks
        modify_reg!(ral::ccm, ccm, CCGR6, 
            CG0: 0b11,  // usb_ctrl1_clk
            CG1: 0b11,  // usb_ctrl2_clk  
            CG2: 0b11,  // usb_phy1_clk
            CG3: 0b11   // usb_phy2_clk
        );
        
        // Configure USB PHY PLL (480MHz)
        let analog = ral::ccm_analog::CCM_ANALOG::instance();
        
        // Power up PLL
        modify_reg!(ral::ccm_analog, analog, PLL_USB1,
            POWER: 1,
            ENABLE: 1,
            EN_USB_CLKS: 1
        );
        
        // Wait for PLL lock
        while read_reg!(ral::ccm_analog, analog, PLL_USB1, LOCK) == 0 {}
    }
}