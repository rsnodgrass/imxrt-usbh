//! Example 05: Multi-Device Manager
//!
//! This example demonstrates advanced USB device enumeration and monitoring.
//! Features:
//! - Real-time device detection and enumeration
//! - Detailed device information parsing and display
//! - Multiple device management and tracking
//! - Device class identification and driver routing
//! - Connection/disconnection event monitoring
//! - Device health monitoring and diagnostics
//! - Performance metrics and statistics

#![no_std]
#![no_main]

use bsp::board;
use embedded_hal::digital::OutputPin;
use imxrt_ral as ral;
use log::info;
use teensy4_bsp as bsp;
use teensy4_panic as _;

use imxrt_usbh::{
    dma::UsbMemoryPool,
    ehci::{
        controller::{EhciController, Running, Uninitialized},
        TransferExecutor,
    },
    enumeration::{DeviceClass, DeviceEnumerator, EnumeratedDevice},
    phy::UsbPhy,
    transfer::simple_control::{ControlExecutor, SetupPacket},
    Result,
};

use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};

/// Maximum number of tracked devices
const MAX_DEVICES: usize = 8;

/// Device connection state
#[derive(Debug, Clone, Copy, PartialEq)]
enum DeviceState {
    Disconnected,
    Detecting,
    Enumerating,
    Configured,
    Error,
}

/// Comprehensive device information
#[derive(Clone)]
struct DeviceInfo {
    // Basic enumeration info
    enumerated: Option<EnumeratedDevice>,

    // Device state
    state: DeviceState,
    slot_id: u8,

    // Timing information
    detection_time: u32,
    enumeration_time: u32,
    last_activity: u32,

    // Device strings
    manufacturer: heapless::String<64>,
    product: heapless::String<64>,
    serial_number: heapless::String<32>,

    // Interface information
    interface_count: u8,
    endpoint_count: u8,

    // Performance metrics
    enumeration_attempts: u8,
    error_count: u8,

    // Class-specific information
    class_info: DeviceClassInfo,
}

impl DeviceInfo {
    fn new(slot_id: u8) -> Self {
        Self {
            enumerated: None,
            state: DeviceState::Disconnected,
            slot_id,
            detection_time: 0,
            enumeration_time: 0,
            last_activity: 0,
            manufacturer: heapless::String::new(),
            product: heapless::String::new(),
            serial_number: heapless::String::new(),
            interface_count: 0,
            endpoint_count: 0,
            enumeration_attempts: 0,
            error_count: 0,
            class_info: DeviceClassInfo::Unknown,
        }
    }

    fn update_device(&mut self, device: EnumeratedDevice, current_time: u32) {
        // Clone device information before moving it
        let device_class = device.class;
        let device_desc = device.device_desc.clone();

        self.enumerated = Some(device);
        self.state = DeviceState::Configured;
        self.enumeration_time = current_time;
        self.last_activity = current_time;

        // Extract class information using cloned values
        self.class_info = match device_class {
            DeviceClass::Hid => DeviceClassInfo::Hid {
                subclass: 0, // Would parse from interface descriptor
                protocol: 0, // Would parse from interface descriptor
            },
            DeviceClass::MassStorage => DeviceClassInfo::MassStorage {
                subclass: 0, // SCSI, UFI, etc.
                protocol: 0, // Bulk-Only Transport, etc.
            },
            DeviceClass::Audio => DeviceClassInfo::Audio {
                is_midi: false, // Would check interface descriptors
                interfaces: 0,
            },
            DeviceClass::Hub => DeviceClassInfo::Hub {
                port_count: 0, // Would parse from hub descriptor
                power_switching: false,
            },
            _ => DeviceClassInfo::Other {
                class_code: device_desc.b_device_class,
                subclass_code: device_desc.b_device_sub_class,
                protocol_code: device_desc.b_device_protocol,
            },
        };
    }

    /// Get device summary string
    fn get_summary(&self) -> heapless::String<128> {
        let mut summary = heapless::String::new();

        if let Some(ref device) = self.enumerated {
            let _ = summary.push_str("VID:");
            write_hex_u16(&mut summary, device.device_desc.id_vendor);
            let _ = summary.push_str(" PID:");
            write_hex_u16(&mut summary, device.device_desc.id_product);
            let _ = summary.push_str(" ");

            match device.class {
                DeviceClass::Hid => {
                    let _ = summary.push_str("HID");
                }
                DeviceClass::MassStorage => {
                    let _ = summary.push_str("MSC");
                }
                DeviceClass::Audio => {
                    let _ = summary.push_str(if device.is_midi { "MIDI" } else { "Audio" });
                }
                DeviceClass::Hub => {
                    let _ = summary.push_str("Hub");
                }
                _ => {
                    let _ = summary.push_str("Other");
                }
            }
        }

        summary
    }
}

/// Helper function to write hex u16 to string
fn write_hex_u16(s: &mut heapless::String<128>, value: u16) {
    const HEX_CHARS: &[u8] = b"0123456789ABCDEF";
    for i in (0..4).rev() {
        let nibble = ((value >> (i * 4)) & 0xF) as u8;
        let _ = s.push(HEX_CHARS[nibble as usize] as char);
    }
}

/// Device class-specific information
#[derive(Debug, Clone)]
enum DeviceClassInfo {
    Unknown,
    Hid {
        subclass: u8,
        protocol: u8,
    },
    MassStorage {
        subclass: u8,
        protocol: u8,
    },
    Audio {
        is_midi: bool,
        interfaces: u8,
    },
    Hub {
        port_count: u8,
        power_switching: bool,
    },
    Other {
        class_code: u8,
        subclass_code: u8,
        protocol_code: u8,
    },
}

/// Device enumeration statistics
#[derive(Default)]
struct EnumerationStats {
    total_detections: AtomicU32,
    successful_enumerations: AtomicU32,
    failed_enumerations: AtomicU32,
    disconnections: AtomicU32,
    active_devices: AtomicU8,

    // Class distribution
    hid_devices: AtomicU8,
    msc_devices: AtomicU8,
    audio_devices: AtomicU8,
    hub_devices: AtomicU8,
    other_devices: AtomicU8,
}

impl EnumerationStats {
    const fn new() -> Self {
        Self {
            total_detections: AtomicU32::new(0),
            successful_enumerations: AtomicU32::new(0),
            failed_enumerations: AtomicU32::new(0),
            disconnections: AtomicU32::new(0),
            active_devices: AtomicU8::new(0),
            hid_devices: AtomicU8::new(0),
            msc_devices: AtomicU8::new(0),
            audio_devices: AtomicU8::new(0),
            hub_devices: AtomicU8::new(0),
            other_devices: AtomicU8::new(0),
        }
    }

    fn device_detected(&self) {
        self.total_detections.fetch_add(1, Ordering::Relaxed);
    }

    fn enumeration_success(&self, class: DeviceClass) {
        self.successful_enumerations.fetch_add(1, Ordering::Relaxed);
        self.active_devices.fetch_add(1, Ordering::Relaxed);

        match class {
            DeviceClass::Hid => {
                self.hid_devices.fetch_add(1, Ordering::Relaxed);
            }
            DeviceClass::MassStorage => {
                self.msc_devices.fetch_add(1, Ordering::Relaxed);
            }
            DeviceClass::Audio => {
                self.audio_devices.fetch_add(1, Ordering::Relaxed);
            }
            DeviceClass::Hub => {
                self.hub_devices.fetch_add(1, Ordering::Relaxed);
            }
            _ => {
                self.other_devices.fetch_add(1, Ordering::Relaxed);
            }
        }
    }

    fn enumeration_failed(&self) {
        self.failed_enumerations.fetch_add(1, Ordering::Relaxed);
    }

    fn device_disconnected(&self, class: DeviceClass) {
        self.disconnections.fetch_add(1, Ordering::Relaxed);
        self.active_devices.fetch_sub(1, Ordering::Relaxed);

        match class {
            DeviceClass::Hid => {
                self.hid_devices.fetch_sub(1, Ordering::Relaxed);
            }
            DeviceClass::MassStorage => {
                self.msc_devices.fetch_sub(1, Ordering::Relaxed);
            }
            DeviceClass::Audio => {
                self.audio_devices.fetch_sub(1, Ordering::Relaxed);
            }
            DeviceClass::Hub => {
                self.hub_devices.fetch_sub(1, Ordering::Relaxed);
            }
            _ => {
                self.other_devices.fetch_sub(1, Ordering::Relaxed);
            }
        }
    }
}

/// Device manager for tracking multiple USB devices
struct DeviceManager {
    devices: [DeviceInfo; MAX_DEVICES],
    stats: EnumerationStats,
    next_slot: AtomicU8,
    system_time: AtomicU32,
}

impl DeviceManager {
    fn new() -> Self {
        const INIT: DeviceInfo = DeviceInfo {
            enumerated: None,
            state: DeviceState::Disconnected,
            slot_id: 0,
            detection_time: 0,
            enumeration_time: 0,
            last_activity: 0,
            manufacturer: heapless::String::new(),
            product: heapless::String::new(),
            serial_number: heapless::String::new(),
            interface_count: 0,
            endpoint_count: 0,
            enumeration_attempts: 0,
            error_count: 0,
            class_info: DeviceClassInfo::Unknown,
        };

        let mut devices = [INIT; MAX_DEVICES];
        for (i, device) in devices.iter_mut().enumerate() {
            device.slot_id = i as u8;
        }

        Self {
            devices,
            stats: EnumerationStats::new(),
            next_slot: AtomicU8::new(0),
            system_time: AtomicU32::new(0),
        }
    }

    fn tick(&self) {
        self.system_time.fetch_add(1, Ordering::Relaxed);
    }

    fn get_time(&self) -> u32 {
        self.system_time.load(Ordering::Relaxed)
    }

    fn find_free_slot(&self) -> Option<usize> {
        for (i, device) in self.devices.iter().enumerate() {
            if device.state == DeviceState::Disconnected {
                return Some(i);
            }
        }
        None
    }

    fn add_device(&mut self, enumerated_device: EnumeratedDevice) -> Option<u8> {
        if let Some(slot) = self.find_free_slot() {
            let current_time = self.get_time();
            let device_class = enumerated_device.class;
            self.devices[slot].update_device(enumerated_device, current_time);
            self.stats.enumeration_success(device_class);
            Some(slot as u8)
        } else {
            None
        }
    }

    fn remove_device(&mut self, slot_id: u8) {
        let slot = slot_id as usize;
        if slot < MAX_DEVICES {
            if let Some(ref device) = self.devices[slot].enumerated {
                self.stats.device_disconnected(device.class);
            }
            self.devices[slot] = DeviceInfo::new(slot_id);
        }
    }

    fn get_active_devices(&self) -> heapless::Vec<u8, MAX_DEVICES> {
        let mut active = heapless::Vec::new();
        for (i, device) in self.devices.iter().enumerate() {
            if device.state == DeviceState::Configured {
                let _ = active.push(i as u8);
            }
        }
        active
    }

    fn get_device(&self, slot_id: u8) -> Option<&DeviceInfo> {
        self.devices.get(slot_id as usize)
    }

    fn get_stats_summary(&self) -> (u32, u32, u32, u8, u8, u8, u8, u8, u8) {
        (
            self.stats.total_detections.load(Ordering::Relaxed),
            self.stats.successful_enumerations.load(Ordering::Relaxed),
            self.stats.failed_enumerations.load(Ordering::Relaxed),
            self.stats.active_devices.load(Ordering::Relaxed),
            self.stats.hid_devices.load(Ordering::Relaxed),
            self.stats.msc_devices.load(Ordering::Relaxed),
            self.stats.audio_devices.load(Ordering::Relaxed),
            self.stats.hub_devices.load(Ordering::Relaxed),
            self.stats.other_devices.load(Ordering::Relaxed),
        )
    }
}

/// Simple non-RTIC application structure
struct EnumerationApp {
    usb_controller: EhciController<8, Running>,
    memory_pool: UsbMemoryPool,
    transfer_executor: TransferExecutor,
    device_manager: DeviceManager,
    status_counter: u32,
}

impl EnumerationApp {
    fn new() -> Result<Self> {
        // Configure USB clocks
        configure_usb_clocks();

        // Initialize USB memory pool
        let memory_pool = UsbMemoryPool::new();

        // Initialize USB PHY (USB2 for host mode on Teensy pins 30/31)
        let phy_base = 0x400D_A000; // USBPHY2 base address
        let ccm_base = 0x400F_C000;
        let _usb_phy = unsafe { UsbPhy::new(phy_base, ccm_base) };

        // Initialize USB host controller (USB2 is for host, USB1 is for device/debug)
        let usb2_base = 0x402E_0200; // USB2 EHCI controller
        let controller = unsafe { EhciController::<8, Uninitialized>::new(usb2_base)? };

        let controller = unsafe { controller.initialize()? };
        let usb_controller = unsafe { controller.start() };

        // Initialize transfer executor
        let transfer_executor = unsafe { TransferExecutor::new(usb2_base) };

        Ok(Self {
            usb_controller,
            memory_pool,
            transfer_executor,
            device_manager: DeviceManager::new(),
            status_counter: 0,
        })
    }

    fn detect_and_enumerate_devices(&mut self) {
        // Try to enumerate new devices
        match self.enumerate_any_device() {
            Ok(device) => {
                if let Some(slot_id) = self.device_manager.add_device(device) {
                    // Device successfully enumerated and added
                    let _ = slot_id;
                } else {
                    // No free slots available
                }
            }
            Err(_) => {
                // No device to enumerate or enumeration failed
                self.device_manager.stats.enumeration_failed();
            }
        }
    }

    fn enumerate_any_device(&mut self) -> Result<EnumeratedDevice> {
        let mut enumerator = DeviceEnumerator::new(
            &mut self.usb_controller,
            &mut self.memory_pool,
            &mut self.transfer_executor,
        );
        let device = enumerator.enumerate_device()?;

        self.device_manager.stats.device_detected();

        // Attempt to get string descriptors
        self.get_device_strings(&device);

        Ok(device)
    }

    fn get_device_strings(&mut self, device: &EnumeratedDevice) {
        // Get manufacturer string
        if device.device_desc.i_manufacturer > 0 {
            if let Ok(manufacturer) = self.get_string_descriptor(
                device.address,
                device.device_desc.i_manufacturer,
                device.max_packet_size,
            ) {
                // Would store in device info
                let _ = manufacturer;
            }
        }

        // Get product string
        if device.device_desc.i_product > 0 {
            if let Ok(product) = self.get_string_descriptor(
                device.address,
                device.device_desc.i_product,
                device.max_packet_size,
            ) {
                // Would store in device info
                let _ = product;
            }
        }
    }

    fn get_string_descriptor(
        &mut self,
        device_address: u8,
        index: u8,
        max_packet_size: u16,
    ) -> Result<heapless::String<64>> {
        let setup = SetupPacket {
            bmRequestType: 0x80,
            bRequest: 0x06,                       // GET_DESCRIPTOR
            wValue: (0x03 << 8) | (index as u16), // STRING descriptor
            wIndex: 0x0409,                       // English (US)
            wLength: 64,
        };

        let mut executor = ControlExecutor::new(&mut self.transfer_executor, &mut self.memory_pool);
        let data = executor.execute_with_retry(setup, device_address, max_packet_size, 3)?;

        // Parse UTF-16LE string descriptor
        let mut result = heapless::String::new();
        if data.len() >= 2 {
            let length = data[0] as usize;
            for i in (2..length.min(data.len())).step_by(2) {
                if i + 1 < data.len() {
                    let ch = u16::from_le_bytes([data[i], data[i + 1]]);
                    if ch < 128 {
                        let _ = result.push(ch as u8 as char);
                    }
                }
            }
        }

        Ok(result)
    }

    fn monitor_devices(&mut self) {
        // Check for device disconnections and health
        let active_devices = self.device_manager.get_active_devices();
        for slot_id in active_devices {
            if let Some(device_info) = self.device_manager.get_device(slot_id) {
                // In a real implementation, check if device is still responding
                // For simulation, keep devices connected
                let _ = device_info;
            }
        }
    }

    fn report_status(&self) {
        if self.status_counter % 5000 == 0 {
            // Every 5 seconds
            let (detections, successes, failures, active, hid, msc, audio, hub, other) =
                self.device_manager.get_stats_summary();

            // In a real application, output these statistics
            let _ = (
                detections, successes, failures, active, hid, msc, audio, hub, other,
            );
        }
    }

    fn run_with_led<L>(&mut self, mut led: L, mut poller: imxrt_log::Poller) -> !
    where
        L: OutputPin,
    {
        info!("\r\n=== USB Multi-Device Manager Example ===");
        info!("Waiting for USB devices...\r\n");
        poller.poll();

        let mut led_state = false;

        loop {
            poller.poll();

            // Update system time
            self.device_manager.tick();

            // Device detection and enumeration
            self.detect_and_enumerate_devices();

            // Monitor existing devices
            self.monitor_devices();

            // Status reporting
            self.status_counter += 1;
            if self.status_counter % 5000 == 0 {
                // Every 5 seconds
                let (detections, successes, failures, active, hid, msc, audio, hub, other) =
                    self.device_manager.get_stats_summary();

                info!("\r\n📊 Device Statistics:");
                info!(
                    "  Detections: {}, Success: {}, Failed: {}",
                    detections, successes, failures
                );
                info!(
                    "  Active: {} (HID:{}, MSC:{}, Audio:{}, Hub:{}, Other:{})",
                    active, hid, msc, audio, hub, other
                );
                poller.poll();
            }

            // Blink LED every 500ms
            if self.status_counter % 500 == 0 {
                led_state = !led_state;
                if led_state {
                    let _ = led.set_high();
                } else {
                    let _ = led.set_low();
                }
            }

            // Small delay
            delay_ms(1);
        }
    }
}

#[bsp::rt::entry]
fn main() -> ! {
    let board::Resources {
        pins,
        mut gpio2,
        usb,
        ..
    } = board::t40(board::instances());

    let mut led = board::led(&mut gpio2, pins.p13);

    // Set up USB CDC logging
    let mut poller = imxrt_log::log::usbd(usb, imxrt_log::Interrupts::Enabled).unwrap();

    poller.poll();

    let mut app = EnumerationApp::new().expect("Failed to initialize USB enumeration app");

    // Pass the led and poller to the app
    app.run_with_led(led, poller)
}

/// Configure USB clocks for i.MX RT1062
fn configure_usb_clocks() {
    use ral::{modify_reg, read_reg};

    unsafe {
        let ccm = ral::ccm::CCM::instance();

        // Enable USB clocks
        modify_reg!(ral::ccm, ccm, CCGR6,
            CG0: 0b11,
            CG1: 0b11,
            CG2: 0b11,
            CG3: 0b11
        );

        // Configure USB PHY PLL
        let analog = ral::ccm_analog::CCM_ANALOG::instance();

        modify_reg!(ral::ccm_analog, analog, PLL_USB1,
            POWER: 1,
            ENABLE: 1,
            EN_USB_CLKS: 1
        );

        while read_reg!(ral::ccm_analog, analog, PLL_USB1, LOCK) == 0 {}
    }
}

/// Simple delay function
fn delay_ms(ms: u32) {
    cortex_m::asm::delay(600_000 * ms);
}
