use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::*;
use esp_idf_hal::peripherals::Peripherals;

fn main() -> anyhow::Result<()> {
    esp_idf_hal::sys::link_patches();

    let peripherals = Peripherals::take()?;
    //let mut led = PinDriver::output(peripherals.pins.gpio8)?;
    let mut button = PinDriver::input(peripherals.pins.gpio14)?;

    button.set_pull(Pull::Down)?;

    loop {
        // we are using thread::sleep here to make sure the watchdog isn't triggered
        FreeRtos::delay_ms(1000);

        if button.is_high() {
            //led.set_low()?;
            println!("button pressed")
        } else {
            //led.set_high()?;
            println!("button not pressed")
        }
    }
}
