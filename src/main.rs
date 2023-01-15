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
use esp32c3_hal::ledc::channel::ChannelIFace;
use esp32c3_hal::ledc::timer::TimerIFace;
use esp32c3_hal::{
    clock::ClockControl, i2c, pac::Peripherals, prelude::*, system::SystemExt, timer::TimerGroup,
    Delay, Rtc, IO,
};
use esp_backtrace as _;
use esp_hal_common::clock::CpuClock;
use esp_hal_common::ledc::channel::ChannelHW;
use esp_hal_common::ledc::*;
use esp_println::println;
use riscv_rt::entry;

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

    let mut pwm_timer = ledc.get_timer::<LowSpeed>(timer::Number::Timer2);
    pwm_timer
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty8Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: 25u32.kHz(),
        })
        .expect("Unable to configure PWM Timer: program will now crash.");

    let mut pwm_channel_0 = ledc.get_channel(channel::Number::Channel0, led_pwm_pin_1);
    let mut pwm_channel_1 = ledc.get_channel(channel::Number::Channel1, led_pwm_pin_2);

    // Set both fan PWM channels to 1% duty cycle by default:
    let mut fan_speed = 1;
    pwm_channel_0
        .configure(channel::config::Config {
            timer: &(pwm_timer),
            duty_pct: fan_speed,
        })
        .ok();
    pwm_channel_1
        .configure(channel::config::Config {
            timer: &(pwm_timer),
            duty_pct: fan_speed,
        })
        .ok();

    // Set up a delay for use in reading the AHT10 data:
    let temp_sensor_delay = Delay::new(&clocks);

    // Set up the AHT10 i2c interface: SCL on pin 2, SDA on pin 3:
    let aht10_i2c = i2c::I2C::new(
        peripherals.I2C0,
        io.pins.gpio3,
        io.pins.gpio2,
        100u32.kHz(),
        &mut system.peripheral_clock_control,
        &clocks,
    )
    .expect("Unable to initialize I2C peripheral: program will now crash.");

    // Set up the AHT10:
    let mut temp_sensor = AHT10::new(aht10_i2c, temp_sensor_delay)
        .expect("Error initializing the temp_sensor: program will now crash.");

    loop {
        wdt0.feed();
        let (h, t) = temp_sensor
            .read()
            .expect("Unable to read temperature: program will now crash.");
        println!("Temperature: {:?}C, Humidity: {:?}%", t.celsius(), h.rh());
        fan_speed = match t.celsius() {
            t if t <= MIN_FAN_TEMP => 1,
            t if t >= MAX_FAN_TEMP => 100,
            t => map(t, MIN_FAN_TEMP, MAX_FAN_TEMP, 1.1, 99.9) as u8,
        };

        println!("Fan Speed Set To {:?}%", fan_speed);
        // Set fan PWM channels to fan_speed:
        pwm_channel_0.set_duty(fan_speed).ok();
        pwm_channel_0.configure_hw().ok();
        pwm_channel_1.set_duty(fan_speed).ok();
        pwm_channel_1.configure_hw().ok();

        delay.delay_ms(10_000_u32);
    }
}

fn map(x: f32, in_min: f32, in_max: f32, out_min: f32, out_max: f32) -> f32 {
    (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
}
