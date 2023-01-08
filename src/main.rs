//! This is a simple repo that uses a DHT22 temperature sensor to control the
//! speed of two 4-pin fans via their PWM control pins. It checks the measured
//! temperature on the DHT22 against a range specified in the code and linearly
//! maps the fans from off to full speed based on where the temperature falls
//! within (or outside) of that range.
//!
//! The esp32c3 is currently set up to use pins 0 and 1 as the two fan PWM
//! control pins, and pin 3 as the DHT22 temperature / humidity sensor pin.
//! The fans I am using (Noctua NF-A12x25) will require a separate 12V power
//! source, but the PWM control pins and the temperature sensor should run fine
//! using the esp32c3's 3v3 for signal and power. Depending on your fan you may
//! need a level converter to provide 5V PWM signals to the fans.
//!
//! In my hardware setup, I'm using a LM2596 buck converter to provide me with
//! 5V for powering the ESP32 via it's micro-USB port. I intentionally used the
//! USB port to avoid issues with connecting USB for programming while the
//! external 5V power is present. If you opt to directly wire to the 5V pin of
//! the esp32c3, I recommend physically disconnecting the 5V pin of the USB
//! connector, otherwise you may damage your voltage regulator.

#![no_std]
#![no_main]

use esp32c3_hal::ledc::channel::ChannelIFace;
use esp32c3_hal::ledc::timer::TimerIFace;
use esp32c3_hal::{
    clock::ClockControl, pac::Peripherals, prelude::*, system::SystemExt, timer::TimerGroup, Rtc,
    IO,
};
use esp_backtrace as _;
use esp_hal_common::ledc::*;
use esp_println::println;
use riscv_rt::entry;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the watchdog timers. For the ESP32-C3, this includes the Super WDT,
    // the RTC WDT, and the TIMG WDTs.
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;

    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    // Use println!() macro for serial debug messages:
    println!("Debug serial printing is now operational.");

    //Set up PWM IO Pins:
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
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
            duty: timer::config::Duty::Duty5Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: 25u32.kHz(),
        })
        .unwrap();

    let mut pwm_channel_0 = ledc.get_channel(channel::Number::Channel0, led_pwm_pin_1);
    let mut pwm_channel_1 = ledc.get_channel(channel::Number::Channel1, led_pwm_pin_2);

    //Set both fan PWM channels to 0% duty cycle by default:
    pwm_channel_0
        .configure(channel::config::Config {
            timer: &(pwm_timer),
            duty_pct: 0,
        })
        .ok();
    pwm_channel_1
        .configure(channel::config::Config {
            timer: &(pwm_timer),
            duty_pct: 0,
        })
        .ok();

    loop {}
}
