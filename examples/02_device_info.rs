#![no_std]
#![no_main]

use teensy4_bsp as bsp;
use bsp::board;
use teensy4_panic as _;

use imxrt_usbh::simple::SimpleUsbHost;
use imxrt_log as logging;
use imxrt_ral as ral;
use imxrt_hal as hal;

// Hardware addresses for Teensy 4.1
const USB2_BASE: usize = 0x402E_0200; // USB2 (host mode on pins 30/31)
const USBPHY2_BASE: usize = 0x400DA000;
const CCM_BASE: usize = 0x400F_C000;


#[bsp::rt::entry]
fn main() -> ! {
    let _board = board::t41(board::instances());

    // Initialize USB CDC logging (USB1 - micro USB port)
    let usb_instances = hal::usbd::Instances {
        usb: unsafe { ral::usb::USB1::instance() },
        usbnc: unsafe { ral::usbnc::USBNC1::instance() },
        usbphy: unsafe { ral::usbphy::USBPHY1::instance() },
    };

    let mut poller = logging::log::usbd(usb_instances, logging::Interrupts::Disabled)
        .expect("failed to initialize USB logging");

    // Small delay for USB CDC to enumerate
    cortex_m::asm::delay(60_000_000);

    log::info!("");
    log::info!("========================================");
    log::info!("    USB Device Information Scanner");
    log::info!("========================================");
    log::info!("");
    log::info!("Waiting for device on USB2 (pins 30/31)...");
    poller.poll();

    // Initialize USB host (USB2 - pins 30/31)
    let mut usb = SimpleUsbHost::new(USB2_BASE, USBPHY2_BASE, CCM_BASE)
        .expect("failed to initialize USB host");

    log::info!("USB host initialized");
    log::info!("");
    poller.poll();

    loop {
        // Wait for device connection
        let device = usb.wait_for_device()
            .expect("failed to enumerate device");

        log::info!("========================================");
        log::info!("Device Connected!");
        log::info!("========================================");
        poller.poll();

        // Display basic device information
        log::info!("Vendor ID:  0x{:04X}", device.vendor_id());
        log::info!("Product ID: 0x{:04X}", device.product_id());
        log::info!("Address:    {}", device.address());

        let usb_version = device.usb_release();
        let major = (usb_version >> 8) & 0xFF;
        let minor = (usb_version >> 4) & 0x0F;
        log::info!("USB Version: {}.{}", major, minor);
        poller.poll();

        // Display device class
        log::info!("");
        log::info!("Device Type:");
        if device.is_hid() {
            log::info!("  - Human Interface Device (HID)");
        }
        if device.is_mass_storage() {
            log::info!("  - Mass Storage");
        }
        if device.is_audio() {
            log::info!("  - Audio Device");
        }
        if device.is_cdc() {
            log::info!("  - Communications Device (CDC)");
        }
        if device.is_hub() {
            log::info!("  - USB Hub");
        }
        poller.poll();

        // Display endpoint information
        log::info!("");
        log::info!("Endpoints:");
        for (i, ep) in device.endpoints().iter().enumerate() {
            let dir = if ep.is_in() { "IN " } else { "OUT" };
            let ep_type = match ep.transfer_type() {
                imxrt_usbh::simple::TransferType::Control => "Control",
                imxrt_usbh::simple::TransferType::Isochronous => "Isochronous",
                imxrt_usbh::simple::TransferType::Bulk => "Bulk",
                imxrt_usbh::simple::TransferType::Interrupt => "Interrupt",
            };
            log::info!("  EP{}: {} {} (max packet: {} bytes)",
                       i, dir, ep_type, ep.max_packet_size);
        }
        poller.poll();

        log::info!("");
        log::info!("Waiting for device disconnect...");
        log::info!("");
        poller.poll();

        // Wait for device disconnect (simple polling)
        cortex_m::asm::delay(600_000_000); // Wait 1 second before next scan
    }
}
