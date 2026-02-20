#![no_std]
#![no_main]

use esp_idf_sys as sys;
use esp_idf_svc::log::EspLogger;
use log::{info, error};
use esp_idf_hal::i2c::*;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::units::Hertz;
use heapless::Vec;
use bme68x_rust::{Device, Sample, Filter, Odr, OperationMode, DeviceConfig, GasHeaterConfig, CommInterface, Error, Interface, I2C_ADDR_LOW, I2C_ADDR_HIGH};

#[inline(always)]
fn ms_to_ticks(ms: u32) -> u32 {
    (ms as u64 * sys::configTICK_RATE_HZ as u64 / 1000) as u32
}

fn calculate_gas_voltage(gas_res_ohm: f32) -> f32 {
    let v_ref = 3.3;        // Tegangan referensi ESP32/BME
    let r_fixed = 10000.0;  // Estimasi resistor beban internal (10k Ohm)
    
    // Rumus Voltage Divider: Vout = (Vref * Rgas) / (R_fixed + Rgas)
    // Return value in mV (millivolts)
    if gas_res_ohm > 0.0 {
        ((v_ref * gas_res_ohm) / (r_fixed + gas_res_ohm)) * 1000.0
    } else {
        0.0
    }
}

// Custom I2cDriver to implement Interface trait from your new code
struct I2cDriver<'a> {
    i2c: esp_idf_hal::i2c::I2cDriver<'a>,
    addr: u8,
}

impl<'a> Interface for I2cDriver<'a> {
    fn interface_type(&self) -> CommInterface {
        CommInterface::I2C
    }

    fn delay(&self, period: u32) {
        unsafe {
            sys::vTaskDelay(ms_to_ticks(period));
        }
    }

    fn read(&mut self, reg: u8, data: &mut [u8]) -> Result<(), Error> {
        match self.i2c.write(self.addr, &[reg], u32::MAX) {
            Ok(()) => match self.i2c.read(self.addr, data, u32::MAX) {
                Ok(()) => Ok(()),
                Err(e) => {
                    error!("I2C read failed at address {:#04x}, reg {:#04x}: {:?}", self.addr, reg, e);
                    Err(Error::CommunicationFailure)
                }
            },
            Err(e) => {
                error!("I2C write failed at address {:#04x}, reg {:#04x}: {:?}", self.addr, reg, e);
                Err(Error::CommunicationFailure)
            }
        }
    }

    fn write(&mut self, reg: u8, data: &[u8]) -> Result<(), Error> {
        let mut bytes: Vec<u8, 32> = Vec::new();
        if bytes.push(reg).is_err() {
            error!("Failed to push register to buffer");
            return Err(Error::CommunicationFailure);
        }
        if bytes.extend_from_slice(data).is_err() {
            error!("Failed to extend buffer with data");
            return Err(Error::CommunicationFailure);
        }
        match self.i2c.write(self.addr, &bytes, u32::MAX) {
            Ok(()) => Ok(()),
            Err(e) => {
                error!("I2C write failed at address {:#04x}, reg {:#04x}: {:?}", self.addr, reg, e);
                Err(Error::CommunicationFailure)
            }
        }
    }

    unsafe fn read_raw(&mut self, reg: u8, data: *mut u8, len: u32) -> i8 {
        let data_slice = core::slice::from_raw_parts_mut(data, len as usize);
        match self.i2c.write(self.addr, &[reg], u32::MAX) {
            Ok(()) => match self.i2c.read(self.addr, data_slice, u32::MAX) {
                Ok(()) => 0,
                Err(_) => -2,
            },
            Err(_) => -2,
        }
    }

    unsafe fn write_raw(&mut self, reg: u8, data: *const u8, len: u32) -> i8 {
        let data_slice = core::slice::from_raw_parts(data, len as usize);
        let mut bytes: Vec<u8, 32> = Vec::new();
        if bytes.push(reg).is_err() {
            return -1;
        }
        if bytes.extend_from_slice(data_slice).is_err() {
            return -1;
        }
        match self.i2c.write(self.addr, &bytes, u32::MAX) {
            Ok(()) => 0,
            Err(_) => -2,
        }
    }
}

// Function to scan I2C bus for devices
fn scan_i2c(i2c: &mut I2cDriver<'_>) -> Vec<u8, 128> {
    let mut found: Vec<u8, 128> = Vec::new();
    info!("Scanning I2C bus...");
    for addr in 0x08..=0x77 {
        let mut buffer = [0u8; 1];
        i2c.addr = addr;
        // A simple read attempt to check for an ACK
        if i2c.i2c.read(addr, &mut buffer, ms_to_ticks(10)).is_ok() {
            info!("Found device at address {:#04x}", addr);
            found.push(addr).unwrap_or(());
        }
    }
    if found.is_empty() {
        info!("No I2C devices found");
    }
    found
}

// Function to read AHT25 temperature and humidity using esp-idf I2C directly
fn read_aht25_esp(i2c_port: u32, addr: u8) -> Option<(f32, f32)> {
    unsafe {
        // Send measurement command: 0xAC 0x33 0x00
        let cmd = [0xACu8, 0x33, 0x00];
        if sys::i2c_master_write_to_device(i2c_port, addr, cmd.as_ptr() as *mut u8, 3, ms_to_ticks(10) as u32) != 0 {
            return None;
        }
        
        // Wait for measurement to complete (80ms)
        sys::vTaskDelay(ms_to_ticks(80));
        
        // Read 6 bytes
        let mut data = [0u8; 6];
        if sys::i2c_master_read_from_device(i2c_port, addr, data.as_mut_ptr(), 6, ms_to_ticks(10) as u32) != 0 {
            return None;
        }
        
        // Check if data is ready (bit 7 of status byte should be 0)
        if (data[0] & 0x80) != 0 {
            return None;
        }
        
        // Parse humidity (20-bit value from bytes 1-3)
        let humidity_raw = ((data[1] as u32) << 12) | ((data[2] as u32) << 4) | ((data[3] as u32) >> 4);
        let humidity = (humidity_raw as f32 / 1048576.0) * 100.0;
        
        // Parse temperature (20-bit value from bytes 3-5)
        let temp_raw = (((data[3] as u32) & 0x0F) << 16) | ((data[4] as u32) << 8) | (data[5] as u32);
        let temperature = ((temp_raw as f32 / 1048576.0) * 200.0) - 50.0;
        
        Some((temperature, humidity))
    }
}

#[no_mangle]
fn main() {
    sys::link_patches();
    EspLogger::initialize_default();
    unsafe {
        sys::xTaskCreatePinnedToCore(
            Some(sensor_task),
            b"sensor\0".as_ptr() as *const u8,
            32768,
            core::ptr::null_mut(),
            5,
            core::ptr::null_mut(),
            0,
        );
        sys::vTaskDelay(u32::MAX);
    }
}

extern "C" fn sensor_task(_params: *mut core::ffi::c_void) {
    info!("E-Nose System Start");

    // --- BME I2C Initialization ---
    let peripherals = Peripherals::take().unwrap();
    let sda = peripherals.pins.gpio8;
    let scl = peripherals.pins.gpio9;
    let i2c_config = I2cConfig::new()
        .baudrate(Hertz(100_000))
        .sda_enable_pullup(true)
        .scl_enable_pullup(true);
    let i2c = match esp_idf_hal::i2c::I2cDriver::new(peripherals.i2c0, sda, scl, &i2c_config) {
        Ok(i2c) => i2c,
        Err(e) => {
            error!("I2C init failed: {:?}. Halting.", e);
            loop {}
        }
    };

    // Scan I2C bus to find BME688 and AHT25 addresses
    let mut i2c_driver = I2cDriver { i2c, addr: 0 };
    let found_addresses = scan_i2c(&mut i2c_driver);
    let possible_addresses = [I2C_ADDR_LOW, I2C_ADDR_HIGH];
    let bme_address = match possible_addresses.iter().find(|&&addr| found_addresses.contains(&addr)) {
        Some(&addr) => {
            info!("Using BME688 I2C address: {:#04x}", addr);
            addr
        }
        None => {
            error!("BME688 not found at addresses 0x76 or 0x77. Available addresses: {:?}. Halting.", found_addresses);
            loop {}
        }
    };

    // Check for AHT25 at address 0x38
    let aht25_address = 0x38u8;
    let has_aht25 = found_addresses.contains(&aht25_address);
    if has_aht25 {
        info!("Found AHT25 sensor at address {:#04x}", aht25_address);
    } else {
        info!("AHT25 sensor not found at address {:#04x}", aht25_address);
    }

    // Initialize BME688
    i2c_driver.addr = bme_address;
    let mut bme = match Device::initialize(i2c_driver) {
        Ok(bme) => bme,
        Err(e) => {
            error!("BME688 init failed at address {:#04x}: {:?}. Halting.", bme_address, e);
            loop {}
        }
    };

    // Recreate I2C driver for AHT25 if present
    let aht25_i2c_available = if has_aht25 {
        // We need a second reference to I2C - for now, we'll create direct I2C calls in the loop
        // The original i2c from Peripherals::take() is now owned by bme
        true
    } else {
        false
    };

    // BME688 Configuration
    let config = DeviceConfig::default()
        .oversample_temperature(Sample::Once)
        .oversample_pressure(Sample::Once)
        .oversample_humidity(Sample::Once)
        .filter(Filter::Size3)
        .odr(Odr::Standby0_59Ms);
    if let Err(e) = bme.set_config(config) {
        error!("Set BME config failed: {:?}. Halting.", e);
        loop {}
    }

    let gas_config = GasHeaterConfig::default()
        .enable()
        .heater_temp(300)
        .heater_duration(100);
    if let Err(e) = bme.set_gas_heater_conf(OperationMode::Forced, gas_config) {
        error!("Set BME gas heater config failed: {:?}. Halting.", e);
        loop {}
    }
    info!("BME688 initialized.");
    
    // For AHT25, we'll use direct I2C calls since bme owns the i2c_driver
    // We'll create temporary I2C calls when needed

    // --- ADC and GPIO Initialization ---
    unsafe {
        info!("Initializing GPIOs and ADCs via SYS...");
        sys::gpio_config(&sys::gpio_config_t {
            pin_bit_mask: (1 << 35) | (1 << 36) | (1 << 37) | (1 << 38),
            mode: sys::gpio_mode_t_GPIO_MODE_OUTPUT,
            ..core::mem::zeroed()
        });

        let mut adc1_handle: sys::adc_oneshot_unit_handle_t = core::ptr::null_mut();
        sys::adc_oneshot_new_unit(&sys::adc_oneshot_unit_init_cfg_t {
            unit_id: sys::adc_unit_t_ADC_UNIT_1, ..core::mem::zeroed()
        }, &mut adc1_handle);

        let mut adc2_handle: sys::adc_oneshot_unit_handle_t = core::ptr::null_mut();
        sys::adc_oneshot_new_unit(&sys::adc_oneshot_unit_init_cfg_t {
            unit_id: sys::adc_unit_t_ADC_UNIT_2, ..core::mem::zeroed()
        }, &mut adc2_handle);

        let chan_cfg = sys::adc_oneshot_chan_cfg_t {
            atten: sys::adc_atten_t_ADC_ATTEN_DB_11,
            bitwidth: sys::adc_bitwidth_t_ADC_BITWIDTH_DEFAULT,
        };
        sys::adc_oneshot_config_channel(adc1_handle, sys::adc_channel_t_ADC_CHANNEL_3, &chan_cfg);
        sys::adc_oneshot_config_channel(adc1_handle, sys::adc_channel_t_ADC_CHANNEL_4, &chan_cfg);
        sys::adc_oneshot_config_channel(adc1_handle, sys::adc_channel_t_ADC_CHANNEL_5, &chan_cfg);
        sys::adc_oneshot_config_channel(adc1_handle, sys::adc_channel_t_ADC_CHANNEL_6, &chan_cfg);
        sys::adc_oneshot_config_channel(adc2_handle, sys::adc_channel_t_ADC_CHANNEL_4, &chan_cfg);
        sys::adc_oneshot_config_channel(adc2_handle, sys::adc_channel_t_ADC_CHANNEL_5, &chan_cfg);
        sys::adc_oneshot_config_channel(adc2_handle, sys::adc_channel_t_ADC_CHANNEL_6, &chan_cfg);
        info!("GPIOs and ADCs initialized.");

        let start_time = sys::xTaskGetTickCount();
        info!("Starting main loop...");
        let mut loop_counter = 0;
        let mut temp_val = 0.0;
        let mut press_val = 0.0;
        let mut hum_val = 0.0;
        let mut gas_res_val = 0.0;
        let mut gas_volt_val = 0.0; // Variabel tegangan gas
        let mut aht25_temp = 0.0;
        let mut aht25_hum = 0.0;

        loop {
            // ADC reading
            let mut h2s_d4 = 0;  sys::adc_oneshot_read(adc1_handle, sys::adc_channel_t_ADC_CHANNEL_3, &mut h2s_d4);
            let mut ethanol = 0; sys::adc_oneshot_read(adc1_handle, sys::adc_channel_t_ADC_CHANNEL_4, &mut ethanol);
            let mut no2 = 0;     sys::adc_oneshot_read(adc1_handle, sys::adc_channel_t_ADC_CHANNEL_5, &mut no2);
            let mut smoke = 0;   sys::adc_oneshot_read(adc1_handle, sys::adc_channel_t_ADC_CHANNEL_6, &mut smoke);
            let mut h2s_d15 = 0; sys::adc_oneshot_read(adc2_handle, sys::adc_channel_t_ADC_CHANNEL_4, &mut h2s_d15);
            let mut ch4 = 0;     sys::adc_oneshot_read(adc2_handle, sys::adc_channel_t_ADC_CHANNEL_5, &mut ch4);
            let mut nh3 = 0;     sys::adc_oneshot_read(adc2_handle, sys::adc_channel_t_ADC_CHANNEL_6, &mut nh3);

            if loop_counter % 12 == 0 {
                // BME reading
                if let Err(e) = bme.set_op_mode(OperationMode::Forced) {
                    error!("BME set op mode failed: {:?}", e);
                }
                
                match bme.get_data(OperationMode::Forced) {
                    Ok(data_array) => {
                        if !data_array.is_empty() && data_array[0].status & 0x80 != 0 {
                            temp_val     = data_array[0].temperature;
                            press_val    = data_array[0].pressure;
                            hum_val      = data_array[0].humidity;
                            gas_res_val  = data_array[0].gas_resistance;

                            // Hitung tegangan dari resistansi gas
                            gas_volt_val = calculate_gas_voltage(gas_res_val);
                        } else {
                            info!("No new BME data available");
                        }
                    }
                    Err(e) => {
                        error!("BME688 read failed: {:?}", e);
                    }
                };
                
                // AHT25 reading if available
                if aht25_i2c_available {
                    if let Some((temp, hum)) = read_aht25_esp(0, 0x38) {
                        aht25_temp = temp;
                        aht25_hum = hum;
                    }
                }
            }

            // Time-based state machine
            let current_time = sys::xTaskGetTickCount();
            let elapsed_ticks = current_time.wrapping_sub(start_time);
            let elapsed_millis = (elapsed_ticks as u64 * 1000 / sys::configTICK_RATE_HZ as u64) as u32;

            let cycle_second = (elapsed_millis / 1000) % 1200;

            let (v_in, p_in, v_out, p_out, current_status) = match cycle_second {
                0..=299   => (1, 0, 1, 0, "IDLE"),       // 0-299s:    idle       (5 min)
                300..=599 => (0, 1, 1, 0, "INJECTING"),  // 300-599s:  injecting  (5 min)
                600..=899 => (1, 0, 1, 0, "SENSING"),    // 600-899s:  sensing    (5 min)
                _         => (1, 0, 0, 1, "PURGING"),    // 900-1199s: purging    (5 min)
            };
            sys::gpio_set_level(35, v_in);
            sys::gpio_set_level(37, p_in);
            sys::gpio_set_level(36, v_out);
            sys::gpio_set_level(38, p_out);

            // Split logging into two calls to reduce per-call stack usage
            info!(
                "STATE:STATUS={},V_IN={},P_IN={},V_OUT={},P_OUT={} | SENSORS:H2S (D4): raw={}, Ethanol (D5): raw={}, NO2 (D6): raw={}, Smoke (D7): raw={}",
                current_status, v_in, p_in, v_out, p_out,
                h2s_d4, ethanol, no2, smoke
            );
            
            if aht25_i2c_available {
                info!(
                    "SENSORS:H2S (D15): raw={}, CH4 (D16): raw={}, NH3 (D17): raw={}, sample_temp: val={:.2}, sample_humid: val={:.2}, chamber_temp: val={:.2}, chamber_humid: val={:.2}, Press: val={:.2}, Gas_Ohm: val={:.0}, Gas_Volt_mV: val={:.0}",
                    h2s_d15, ch4, nh3,
                    temp_val, hum_val, aht25_temp, aht25_hum, press_val / 100.0, gas_res_val, gas_volt_val
                );
            } else {
                info!(
                    "SENSORS:H2S (D15): raw={}, CH4 (D16): raw={}, NH3 (D17): raw={}, sample_temp: val={:.2}, sample_humid: val={:.2}, Press: val={:.2}, Gas_Ohm: val={:.0}, Gas_Volt_mV: val={:.0}",
                    h2s_d15, ch4, nh3,
                    temp_val, hum_val, press_val / 100.0, gas_res_val, gas_volt_val
                );
            }

            loop_counter += 1;
            sys::vTaskDelay(ms_to_ticks(1000));
        }
    }
}