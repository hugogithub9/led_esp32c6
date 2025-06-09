//! Blinks an LED
//!
//! This assumes that a LED is connected to GPIO8.
//! Depending on your target and the board you are using you should change the pin.
//! If your board doesn't have on-board LEDs don't forget to add an appropriate resistor.
//!

use esp_idf_svc::hal::delay::FreeRtos;
use esp_idf_svc::hal::gpio::*;
use esp_idf_svc::hal::peripherals::Peripherals;

fn main() -> anyhow::Result<()> {
    esp_idf_svc::sys::link_patches();

    let peripherals = Peripherals::take().unwrap();
    let mut led = PinDriver::output(peripherals.pins.gpio8)?;

    println!("Hello world!");
    loop {
        led.set_high()?;
        println!("led on");
        // we are sleeping here to make sure the watchdog isn't triggered
        FreeRtos::delay_ms(1000);

        led.set_low()?;
        println!("led off");
        FreeRtos::delay_ms(1000);
    }
}

//conclusion : this code use a normal led and no succed with a rgb led, so the code is good without error but the led did not change
