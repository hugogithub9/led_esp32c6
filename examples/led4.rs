use std::time::Duration; //pour attendre entre chaque couleur

use anyhow::Result; //gestion d'erreur : Result<T> boite qui dit jai reussi ou foire et bail!() envoie erreur avec message
use esp_idf_hal::{
    delay::FreeRtos,      //gerer les delais temporises
    prelude::Peripherals, //accede au hardware periph
    rmt::{config::TransmitConfig, FixedLengthSignal, PinState, Pulse, TxRmtDriver}, //itilise canal RMT pour piloter la LED RGB
};

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

mod Generator {
    use rand::Rng;
    pub struct GeneratorState {
        sigma: f64,
        period: f64,
        clock: f64,
        //countOn: usize,
        //countOff: usize,
    }

    impl GeneratorState {
        pub fn new(period: f64) -> Self {
            Self {
                sigma: 0.0,
                period,
                clock: 0.0,
                //countOn: 0,
                //countOff:0,
            }
        }
    }

    xdevs::component!(
        ident = Generator,
        output = {
            output<bool>,
        },
        state = GeneratorState,
    );

    impl xdevs::Atomic for Generator {
        fn delta_int(state: &mut Self::State) {
            state.clock += state.sigma;
            state.sigma = state.period;
        }

        fn lambda(state: &Self::State, output: &mut Self::Output) {
            let random = rand::thread_rng().gen_range(0..=1);
            if random == 1 {
                //state.countOn+=1; probleme because self not utable
                output.output.add_value(true).unwrap();
            } else {
                //state.countOff+=1; same pb
                output.output.add_value(false).unwrap();
            }
        }

        fn ta(state: &Self::State) -> f64 {
            state.sigma
        }

        fn delta_ext(state: &mut Self::State, e: f64, input: &Self::Input) {
            state.sigma -= e;
            state.clock += e;
        }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    use crate::neopixel;
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    println!("Hello, world!");

    let peripherals = Peripherals::take()?; //recupe acces au periph - INDISPENSABLE
                                            // Onboard RGB LED pin
                                            // ESP32-C3-DevKitC-02 gpio8, ESP32-C3-DevKit-RUST-1 gpio2
    let led = peripherals.pins.gpio8; //dit que led est branche sur gpio8
    let channel = peripherals.rmt.channel0; //utilise channel0 du periph RMT /module de transmission de signaux
    let config = TransmitConfig::new().clock_divider(1); //config de base avec diviseur d'horloge de 1 (precision max)
    let mut tx = TxRmtDriver::new(channel, led, &config)?; //initialise pilote de transmission : channel0, broche gpio8, config celle juste creer

    // 3 seconds white at 10% brightness
    neopixel(Rgb::new(0, 200, 0), &mut tx)?; //envoie du blanc a 10% de luminosite (valeur max 255)
    FreeRtos::delay_ms(3000); //att 3s

    let period = 1.5;

    let generator = Generator::Generator::new(Generator::GeneratorState::new(period));

    let mut simulator = xdevs::simulator::Simulator::new(generator);

    simulator.simulate_rt(
        //start,stop
        0.0,
        60.0,
        xdevs::simulator::std::sleep(0.0, 1.0, None),
        |output| {
            if output.output.get_values()[0] {
                neopixel(Rgb::new(0, 0, 255), &mut tx).unwrap();
            } else {
                neopixel(Rgb::new(255, 0, 0), &mut tx).unwrap();
            }
        }, //hardware
    );
    neopixel(Rgb::new(0, 100, 100), &mut tx).unwrap();
    println!("Goodbye, world!");

    Ok(())
}
