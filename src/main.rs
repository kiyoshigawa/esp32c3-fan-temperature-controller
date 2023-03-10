//! This is a simple repo that uses a AHT10 temperature sensor to control the
//! speed of two 4-pin fans via their PWM control pins. It checks the measured
//! temperature on the AHT10 against a range specified in the code and linearly
//! maps the fans from off to full speed based on where the temperature falls
//! within (or outside) of that range.
//!
//! The esp32c3 is currently set up to use pins 0 and 1 as the two fan PWM
//! control pins, and pins 2 and 3 as the AHT10 temperature / humidity sensor
//! i2c pins. The fans I am using (Noctua NF-A12x25) will require a separate
//! 12V power source, but the PWM control pins and the temperature sensor
//! should run fine using the esp32c3's 3v3 for signal and power. Depending on
//! your fan you may need a level converter to provide 5V PWM signals to the
//! fans.
//!
//! In my hardware setup, I'm using a LM2596 buck converter to provide me with
//! 5V for powering the ESP32 via it's micro-USB port. I intentionally used the
//! USB port to avoid issues with connecting USB for programming while the
//! external 5V power is present. If you opt to directly wire to the 5V pin of
//! the esp32c3, I recommend physically disconnecting the 5V pin of the USB
//! connector, otherwise you may damage your voltage regulator.
//!
//! I had to use a modified version of the AHT10 library. The version on
//! crates.io was not working with the AHT10 sensor. There is a pull request at
//! https://github.com/heyitsanthony/aht10/pull/1 that has code that will work.
//! I manually applied it to a local copy of the library and now the sensor
//! works. Unfortunately it looks like the author is not currently responding
//! to the pull request, so you will likely need to make a copy and apply the PR
//! yourself to get the sensor working with this code.

#![no_std]
#![no_main]

use aht10::AHT10;
use channel::config::Config as ChannelConfig;
use embedded_graphics::mono_font::ascii::FONT_9X15;
use embedded_graphics::{
    mono_font::MonoTextStyleBuilder,
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::i2c::SevenBitAddress;
use esp32c3_hal::ledc::channel::ChannelIFace;
use esp32c3_hal::ledc::timer::TimerIFace;
use esp32c3_hal::pac::I2C0;
use esp32c3_hal::{
    clock::ClockControl, i2c, pac::Peripherals, prelude::*, system::SystemExt, timer::TimerGroup,
    Delay, Rtc, IO,
};
use esp_backtrace as _;
use esp_hal_common::clock::CpuClock;
use esp_hal_common::ledc::*;
use esp_println::println;
use format_no_std::show;
use riscv_rt::entry;
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
use timer::config::Config as TimerConfig;

// Lowest temp reading for fans to spin up:
const MIN_FAN_TEMP: f32 = 25.0; // 77 F

// Temp reading when fans should be at max speed:
const MAX_FAN_TEMP: f32 = 35.0; // 95 F

#[entry]
fn main() -> ! {
    let peripherals =
        Peripherals::take().expect("Your chip is probably borked: program will now crash.");
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock160MHz).freeze();

    // Disable the watchdog timers. For the ESP32-C3, this includes the Super WDT,
    // the RTC WDT, and the TIMG WDTs.
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;

    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.start(30_u64.secs());
    wdt1.disable();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // Use println!() macro for serial debug messages:
    println!("Debug serial printing is now operational.");

    // Set up delay for use in the loop to moderate reading frequency
    let mut delay = Delay::new(&clocks);

    // Set up PWM IO Pins:
    let led_pwm_pin_1 = io.pins.gpio0.into_push_pull_output();
    let led_pwm_pin_2 = io.pins.gpio1.into_push_pull_output();

    let mut ledc = LEDC::new(
        peripherals.LEDC,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    // Set up the i2c interface: SCL on pin 2, SDA on pin 3:
    // If this works, we can display errors on the OLED screen
    let mut i2c = i2c::I2C::new(
        peripherals.I2C0,
        io.pins.gpio3,
        io.pins.gpio2,
        100u32.kHz(),
        &mut system.peripheral_clock_control,
        &clocks,
    )
    .expect(I2C_INIT_ERROR);

    let mut pwm_timer = ledc.get_timer::<LowSpeed>(timer::Number::Timer2);
    match pwm_timer.configure(TimerConfig {
        duty: timer::config::Duty::Duty8Bit,
        clock_source: timer::LSClockSource::APBClk,
        frequency: 25u32.kHz(),
    }) {
        Ok(_) => {}
        Err(_) => pwm_timer_config_panic(&mut i2c),
    }

    let mut pwm_channel_0 = ledc.get_channel(channel::Number::Channel0, led_pwm_pin_1);
    let mut pwm_channel_1 = ledc.get_channel(channel::Number::Channel1, led_pwm_pin_2);

    // Set both fan PWM channels to 10% duty cycle by default:
    let initial_fan_speed = 10;

    let pwm_config = |fan_speed| ChannelConfig {
        timer: &(pwm_timer),
        duty_pct: fan_speed,
    };

    pwm_channel_0.configure(pwm_config(initial_fan_speed)).ok();
    pwm_channel_1.configure(pwm_config(initial_fan_speed)).ok();

    // Set up a delay for use in reading the AHT10 data:
    let mut temp_sensor_delay = Delay::new(&clocks);

    loop {
        wdt0.feed();

        // Read Temps:
        let (t, fs) = read_temp(&mut i2c, &mut temp_sensor_delay);

        // Set fan PWM channels to fan_speed:
        pwm_channel_0.configure(pwm_config(fs)).ok();
        pwm_channel_1.configure(pwm_config(fs)).ok();

        println!("Temp: {:.2}C, Fan Speed: {:?}%", t, fs);

        // Print to Screen:
        let mut buffer_line_1 = [0_u8; 20];
        let mut buffer_line_2 = [0_u8; 20];
        let print_string_line_1 = show(&mut buffer_line_1, format_args!("T: {:.2}C", t)).unwrap();
        let print_string_line_2 = show(&mut buffer_line_2, format_args!("FS: {:?}%", fs)).unwrap();

        oled_write(&mut i2c, print_string_line_1, print_string_line_2);

        delay.delay_ms(10_000_u32);
    }
}

fn oled_write<'a, 'b>(i2c_ref: &'a mut i2c::I2C<I2C0>, line_1_str: &'b str, line_2_str: &'b str) {
    let i2c_share = I2cShare::new(i2c_ref);
    let interface = I2CDisplayInterface::new(i2c_share);
    let mut oled_display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();

    if oled_display.init().is_ok() {
        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_9X15)
            .text_color(BinaryColor::On)
            .build();

        Text::with_baseline(line_1_str, Point::zero(), text_style, Baseline::Top)
            .draw(&mut oled_display)
            .unwrap();

        Text::with_baseline(line_2_str, Point::new(0, 16), text_style, Baseline::Top)
            .draw(&mut oled_display)
            .unwrap();

        oled_display.flush().unwrap();
    } else {
        oled_display_init_error(line_1_str, line_2_str);
    }
}

fn read_temp(i2c_ref: &mut i2c::I2C<I2C0>, delay_ref: &mut Delay) -> (f32, u8) {
    let i2c_share = I2cShare::new(i2c_ref);
    let delay_share = DelayShare::new(delay_ref);
    if let Ok(mut temp_sensor) = AHT10::new(i2c_share, delay_share) {
        if let Ok(temp_sensor_read_result) = temp_sensor.read() {
            let (_h, t) = temp_sensor_read_result;
            let t_celsius = t.celsius();
            let fan_speed = match t.celsius() {
                t if t <= MIN_FAN_TEMP => 1,
                t if t >= MAX_FAN_TEMP => 100,
                t => map(t, MIN_FAN_TEMP, MAX_FAN_TEMP, 1.1, 99.9) as u8,
            };
            (t_celsius, fan_speed)
        } else {
            // If there is an error in the read, this will panic
            temp_sensor_read_panic(i2c_ref);
            // the compiler doesn't know it will panic, so this lets me use the i2c_ref again
            (0.0, 0)
        }
    } else {
        // If there is an error in the initialization, this will panic
        temp_sensor_init_panic(i2c_ref);
        // the compiler doesn't know it will panic, so this lets me use the i2c_ref again
        (0.0, 0)
    }
}

struct I2cShare<'a> {
    i2c_bit: &'a mut i2c::I2C<I2C0>,
}

impl<'a> I2cShare<'a> {
    fn new(i2c_bit: &'a mut i2c::I2C<I2C0>) -> Self {
        Self { i2c_bit }
    }
}

impl<'a> embedded_hal::blocking::i2c::Write for I2cShare<'a> {
    type Error = i2c::Error;

    fn write(&mut self, address: SevenBitAddress, bytes: &[u8]) -> Result<(), Self::Error> {
        self.i2c_bit.write(address, bytes)
    }
}

impl<'a> embedded_hal::blocking::i2c::Read for I2cShare<'a> {
    type Error = i2c::Error;

    fn read(&mut self, address: SevenBitAddress, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c_bit.read(address, buffer)
    }
}

impl<'a> embedded_hal::blocking::i2c::WriteRead for I2cShare<'a> {
    type Error = i2c::Error;

    fn write_read(
        &mut self,
        address: SevenBitAddress,
        bytes: &[u8],
        buffer: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.i2c_bit.write_read(address, bytes, buffer)
    }
}

struct DelayShare<'a> {
    delay_bit: &'a mut Delay,
}

impl<'a> DelayShare<'a> {
    fn new(delay_bit: &'a mut Delay) -> Self {
        Self { delay_bit }
    }
}
impl<'a> DelayMs<u16> for DelayShare<'a> {
    fn delay_ms(&mut self, ms: u16) {
        self.delay_bit.delay_ms(ms)
    }
}

fn map(x: f32, in_min: f32, in_max: f32, out_min: f32, out_max: f32) -> f32 {
    (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
}

fn pwm_timer_config_panic(i2c_ref: &mut i2c::I2C<I2C0>) {
    oled_write(i2c_ref, "PWM init", "error");
    panic!("Unable to configure PWM Timer: program will now crash.");
}

fn temp_sensor_init_panic(i2c_ref: &mut i2c::I2C<I2C0>) {
    oled_write(i2c_ref, "AHT10 init", "error");
    panic!("Error initializing the temp_sensor: program will now crash.");
}

fn temp_sensor_read_panic(i2c_ref: &mut i2c::I2C<I2C0>) {
    oled_write(i2c_ref, "AHT10 read", "error");
    panic!("Unable to read temperature: program will now crash.");
}

fn oled_display_init_error(line_1_str: &str, line_2_str: &str) {
    println!("The OLED Was not able to initialize.");
    println!(
        "OLED Message was to be as follows:\r\n{:?}\r\n{:?}",
        line_1_str, line_2_str
    );
}

static I2C_INIT_ERROR: &str = "Unable to initialize I2C peripheral: program will now crash.";
