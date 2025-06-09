use std::time::Duration; //pour attendre entre chaque couleur

use anyhow::{bail, Result}; //gestion d'erreur : Result<T> boite qui dit jai reussi ou foire et bail!() envoie erreur avec message
use esp_idf_hal::{
    delay::FreeRtos,      //gerer les delais temporises
    prelude::Peripherals, //accede au hardware periph
    rmt::{config::TransmitConfig, FixedLengthSignal, PinState, Pulse, TxRmtDriver}, //itilise canal RMT pour piloter la LED RGB
};

pub fn main() -> Result<()> {
    //fct principale envoie erreur si y a un pb
    esp_idf_hal::sys::link_patches(); //toujours mettre ca pour linkage

    let peripherals = Peripherals::take()?; //recupe acces au periph - INDISPENSABLE
                                            // Onboard RGB LED pin
                                            // ESP32-C3-DevKitC-02 gpio8, ESP32-C3-DevKit-RUST-1 gpio2
    let led = peripherals.pins.gpio8; //dit que led est branche sur gpio8
    let channel = peripherals.rmt.channel0; //utilise channel0 du periph RMT /module de transmission de signaux
    let config = TransmitConfig::new().clock_divider(1); //config de base avec diviseur d'horloge de 1 (precision max)
    let mut tx = TxRmtDriver::new(channel, led, &config)?; //initialise pilote de transmission : channel0, broche gpio8, config celle juste creer

    // 3 seconds white at 10% brightness
    neopixel(Rgb::new(25, 25, 25), &mut tx)?; //envoie du blanc a 10% de luminosite (valeur max 255)
    FreeRtos::delay_ms(3000); //att 3s

    // infinite rainbow loop at 20% brightness
    (0..360).cycle().try_for_each(|hue| {
        //boucle infini sur les valeurs de 0 a 359 degre (degre de teinte =couleur cercle chromatique)
        FreeRtos::delay_ms(10); //att 0.01s
        let rgb = Rgb::from_hsv(hue, 100, 20)?; ////// convertit teinte hue : saturation 100% et luminosite 20% vers une couleur RGB
        neopixel(rgb, &mut tx) //renvoie couleur convertie a la LED
    })
}

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
    /// Converts hue, saturation, value to RGB
    pub fn from_hsv(h: u32, s: u32, v: u32) -> Result<Self> {
        //convertit HSV en RGB pour fairedes effets colores naturels
        if h > 360 || s > 100 || v > 100 {
            bail!("The given HSV values are not in valid range");
        }
        let s = s as f64 / 100.0;
        let v = v as f64 / 100.0;
        let c = s * v;
        let x = c * (1.0 - (((h as f64 / 60.0) % 2.0) - 1.0).abs());
        let m = v - c;
        let (r, g, b) = match h {
            0..=59 => (c, x, 0.0),
            60..=119 => (x, c, 0.0),
            120..=179 => (0.0, c, x),
            180..=239 => (0.0, x, c),
            240..=299 => (x, 0.0, c),
            _ => (c, 0.0, x),
        };
        Ok(Self {
            r: ((r + m) * 255.0) as u8,
            g: ((g + m) * 255.0) as u8,
            b: ((b + m) * 255.0) as u8,
        })
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
