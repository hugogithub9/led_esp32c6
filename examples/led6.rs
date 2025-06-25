//same model as led5.rs but we will delete the activator. The switch of the LED will be directly on hardware
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

mod button {
    use rand::Rng;
    pub struct ButtonState {
        sigma: f64,
        clock: f64,
        pressed: bool,
    }

    impl ButtonState {
        pub fn new() -> Self {
            Self {
                sigma: 0.0,
                clock: 0.0,
                pressed: false,
            }
        }
    }

    xdevs::component!(
        ident = Button,
        output = {
            pulse<bool>,
        },
        state = ButtonState,
    );

    impl xdevs::Atomic for Button {
        fn delta_int(state: &mut Self::State) {
            state.clock += state.sigma;
            let t_random = rand::thread_rng().gen_range(0..=10) as f64;
            state.sigma = t_random;
            state.pressed = !state.pressed;
        }

        fn lambda(state: &Self::State, output: &mut Self::Output) {
            if state.pressed {
                output.pulse.add_value(false);
                println!(
                    "[t={}]: the button is not pressed",
                    state.clock + state.sigma
                );
            } else {
                output.pulse.add_value(true);
                println!("[t={}]: the button is pressed", state.clock + state.sigma);
            }
        }

        fn ta(state: &Self::State) -> f64 {
            state.sigma
        }

        fn delta_ext(state: &mut Self::State, e: f64, input: &Self::Input) {
            //pas d'entree
            state.sigma -= e;
            state.clock += e;
        }
    }
}

mod controller {
    use crate::neopixel;

    pub struct ControllerState {
        sigma: f64,
        clock: f64,
        t_wait: f64,
        t1: f64,
        t2: f64,
    }

    impl ControllerState {
        pub fn new() -> Self {
            Self {
                sigma: 0.0,
                clock: 0.0,
                t_wait: 0.0,
                t1: 0.0,
                t2: 0.0,
            }
        }
    }

    xdevs::component!(
        ident = Controller,
        input= {
            pulse<bool>,
        },
        output = {
            color<char>,
        },
        state = ControllerState,
    );

    impl xdevs::Atomic for Controller {
        fn delta_int(state: &mut Self::State) {
            state.clock += state.sigma;
            println!("nothing receive t={} ", state.clock);
            state.sigma = f64::INFINITY;
        }

        fn lambda(state: &Self::State, output: &mut Self::Output) {
            if state.t_wait <= 3.0 {
                output.color.add_value('g');
                println!("send green request");
            }
            if state.t_wait > 3.0 && state.t_wait <= 6.0 {
                output.color.add_value('b');
                println!("send blue request");
            }
            if state.t_wait > 6.0 && state.t_wait <= 10.0 {
                output.color.add_value('r');
                println!("send red request");
            }
        }

        fn ta(state: &Self::State) -> f64 {
            state.sigma
        }

        fn delta_ext(state: &mut Self::State, e: f64, input: &Self::Input) {
            state.sigma -= e;
            state.clock += e;
            if input.pulse.get_values()[0] {
                state.t1 = state.clock;
                println!("Receive true at t1={} ", state.clock);
            } else {
                state.t2 = state.clock;
                println!("Receive false at t2={} ", state.clock);
            }
            if state.t2 >= state.t1 {
                state.t_wait = state.t2 - state.t1;
                println!("The button was pulsed during t={} ", state.t_wait);
                state.sigma = 0.;
            }
        }
    }
}

xdevs::component!(
    ident = ButtonController,
    output = {
        color<char>,
    },
    components = {
        button: button::Button,
        controller:controller::Controller,
    },
    couplings = {
        button.pulse -> controller.pulse,
        controller.color -> color,
    }
);

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
    let mut tx = TxRmtDriver::new(channel, led, &config)?;

    neopixel(Rgb::new(25, 25, 25), &mut tx)?; //envoie du blanc a 10% de luminosite (valeur max 255)
    FreeRtos::delay_ms(3000); //att 3s

    let button = button::Button::new(button::ButtonState::new());
    let controller = controller::Controller::new(controller::ControllerState::new());
    let buttoncontroller = ButtonController::new(button, controller);

    let mut simulator = xdevs::simulator::Simulator::new(buttoncontroller);
    let config = xdevs::simulator::Config::new(0.0, 60.0, 1.0, None);

    simulator.simulate_rt(
        //start,stop
        &config,
        xdevs::simulator::std::sleep(&config),
        |output| {
            let values = output.color.get_values();

            if let Some(c) = values.first() {
                if *c == 'g' {
                    neopixel(Rgb::new(0, 255, 0), &mut tx).unwrap();
                    println!("green LED")
                }
                if *c == 'b' {
                    neopixel(Rgb::new(0, 0, 255), &mut tx).unwrap();
                    println!("blue LED")
                }
                if *c == 'r' {
                    neopixel(Rgb::new(255, 0, 0), &mut tx).unwrap();
                    println!("red LED")
                }
            }
        }, //hardware
    );
    neopixel(Rgb::new(1, 1, 1), &mut tx).unwrap();
    Ok(())
}
