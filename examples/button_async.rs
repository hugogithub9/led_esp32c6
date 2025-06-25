//! Turn an LED on/off depending on the state of a button
//!
//! This assumes that a LED is connected to GPIO8.
//! Additionally this assumes a button connected to GPIO14.
//! On an ESP32C3 development board this is the BOOT button.
//!
//! Depending on your target and the board you are using you should change the pins.
//! If your board doesn't have on-board LEDs don't forget to add an appropriate resistor.

use esp_idf_hal::gpio::*;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::task::*;

use anyhow::Result; //gestion d'erreur : Result<T> boite qui dit jai reussi ou foire et bail!() envoie erreur avec message
use esp_idf_hal::{
    rmt::{config::TransmitConfig, FixedLengthSignal, PinState, Pulse, TxRmtDriver}, //itilise canal RMT pour piloter la LED RGB
};
use std::time::Duration; //pour attendre entre chaque couleur

fn neopixel(rgb: Rgb, tx: &mut TxRmtDriver) -> Result<()> {
    //fct envoie une couleur RGB a une led neopixel via canal RMT
    let color: u32 = rgb.into(); //transforme couleur (r,g,b) en une seule valeur u32 pour manipuler bit par bit
    let ticks_hz = tx.counter_clock()?; //recup frquence du compteur materiel->pour calculer les durees des impulsions a envoyer
    let (t0h, t0l, t1h, t1l) = (
        //definit les pulses electriaues pour envoyer un 0 ou un 1 a une LED WS2812
        Pulse::new_with_duration(ticks_hz, PinState::High, &Duration::from_nanos(350))?,
        Pulse::new_with_duration(ticks_hz, PinState::Low, &Duration::from_nanos(800))?, //0 = HIGH 350ns + LOW 800ns
        Pulse::new_with_duration(ticks_hz, PinState::High, &Duration::from_nanos(700))?,
        Pulse::new_with_duration(ticks_hz, PinState::Low, &Duration::from_nanos(600))?, //1 = HIGH 700ns + LOW 600ns
    );
    let mut signal = FixedLengthSignal::<24>::new(); //prepare signal 24bit -8pourG, 8R et 8B
    for i in (0..24).rev() {
        //lit chaque bit de color si bit1:true si bit0:false (3lignes)
        let p = 2_u32.pow(i);
        let bit: bool = p & color != 0;
        let (high_pulse, low_pulse) = if bit { (t1h, t1l) } else { (t0h, t0l) }; //si bit 1ou0 ajt les pulses correspondants au signal
        signal.set(23 - i as usize, &(high_pulse, low_pulse))?;
    }
    tx.start_blocking(&signal)?; //une fois le signal complet on lenvoie a la LED via canal RMT
    Ok(()) //fin fct neopixel
}

struct Rgb {
    //struct couleur RGB rouge, vert, bleu entre 0 et 255
    r: u8,
    g: u8,
    b: u8,
}

impl Rgb {
    //constructeur simple
    pub fn new(r: u8, g: u8, b: u8) -> Self {
        Self { r, g, b }
    }
}

impl From<Rgb> for u32 {
    //definit comment transformer une couleur RGB en nb bonaire de 24 bits
    /// Convert RGB to u32 color value
    ///
    /// e.g. rgb: (1,2,4)
    /// G        R        B
    /// 7      0 7      0 7      0
    /// 00000010 00000001 00000100
    fn from(rgb: Rgb) -> Self {
        ((rgb.g as u32) << 16) | ((rgb.r as u32) << 8) | rgb.b as u32 //place g de 16a23bits, r:8a15, b:0a7
    }
}

fn main() -> anyhow::Result<()> {
    esp_idf_hal::sys::link_patches();

    let peripherals = Peripherals::take()?;

    //let mut led = PinDriver::output(peripherals.pins.gpio8)?;

    let led = peripherals.pins.gpio8;

    let channel = peripherals.rmt.channel0;
    let config = TransmitConfig::new().clock_divider(1);
    let mut tx = TxRmtDriver::new(channel, led, &config)?;
    let mut button = PinDriver::input(peripherals.pins.gpio14)?;

    button.set_pull(Pull::Down)?;

    block_on(async {
        //fct synchrone qui lance une tache asynchrone
        loop {
            button.wait_for_high().await?; //aspects asynchrones : appel non bloquant dans l'OS
                                           // libere le CPU pendant que le code attend l'event GPIO

            neopixel(Rgb::new(0, 255, 0), &mut tx).unwrap();

            button.wait_for_low().await?;

            neopixel(Rgb::new(255, 0, 0), &mut tx).unwrap();
        }
    })
}
