//! Shared test utilities for imxrt-usbh tests
//!
//! This module provides common helpers, mocks, and utilities
//! used across multiple test files.

#![no_std]

pub mod mock_hardware;

// Re-export commonly used items
pub use mock_hardware::{
    create_mock_buffer,
    create_test_device_descriptor,
    create_test_config_descriptor,
    assert_buffer_aligned,
};
