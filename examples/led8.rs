//same model as led7.rs but model button doesn't exist anymore it is an input function

use std::time::{Duration, SystemTime}; //pour attendre entre chaque couleur

use anyhow::Result; //gestion d'erreur : Result<T> boite qui dit jai reussi ou foire et bail!() envoie erreur avec message
use esp_idf_hal::gpio::*;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::{
    delay::FreeRtos, //gerer les delais temporises
    //prelude::Peripherals, //accede au hardware periph
    rmt::{config::TransmitConfig, FixedLengthSignal, PinState, Pulse, TxRmtDriver}, //itilise canal RMT pour piloter la LED RGB
};

use crate::controller::ControllerInput;

use esp_idf_hal::task::thread;

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

pub fn input_handler(
    button: PinDriver<'static, Gpio14, Input>, //take the nutton pin in input
) -> impl FnMut(Duration, &mut ControllerInput) {
    //return fct with 2 arg : Duration and Input
    // puedes añadir todo lo que quieras de configuración
    let mut n_times = 0; //count how many time fct called
    let mut pressed = false; //state of button

    //creation input

    move |duration, input| {
        // calcular en que instante hay que salir
        let timeout = SystemTime::now() + duration; //calculate the time we stop to look at button
        println!("function called {timeout:?}");
        // mientras no haya que salir,
        while SystemTime::now() < timeout {
            //  ver el estado del botón
            // si es diferente al anterior, inyecto mensaje y paro antes de tiempo
            //println!("in while loop");
            if button.is_high() && !pressed {
                println!("[n={n_times}]: the button is pressed ");
                input.pulse.add_value(true).unwrap();
                pressed = button.is_high();
                break;
            } else if button.is_low() && pressed {
                println!("[n={n_times}]: the button is not pressed anymore");
                input.pulse.add_value(false).unwrap();
                pressed = button.is_high();
                break;
            }

            FreeRtos::delay_ms(1); //prevent polling let other task
        }

        n_times += 1;
    }
}

mod controller {
    use core::f64;

    #[derive(Default)]
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
                sigma: f64::INFINITY,
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
                output.color.add_value('g').unwrap();
                println!("send green request");
            } else if state.t_wait <= 6.0 {
                output.color.add_value('b').unwrap();
                println!("send blue request");
            } else if state.t_wait <= 10.0 {
                output.color.add_value('r').unwrap();
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
    input = {
            pulse<bool>,
        },
    components = {
        //button: button::Button,
        controller:controller::Controller,
    },
    couplings = {
        //button.pulse -> controller.pulse,
        controller.color -> color,
        pulse -> controller.pulse
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

    let mut button = PinDriver::input(peripherals.pins.gpio14).unwrap();
    button.set_pull(Pull::Down).unwrap();

    let ihandler = input_handler(button);

    neopixel(Rgb::new(25, 25, 25), &mut tx)?; //envoie du blanc a 10% de luminosite (valeur max 255)
    FreeRtos::delay_ms(3000); //att 3s

    //let button = button::Button::new(button::ButtonState::new(tick));
    let controller = controller::Controller::new(controller::ControllerState::new());
    //let buttoncontroller = ButtonController::new(controller);

    let mut simulator = xdevs::simulator::Simulator::new(controller);
    let config = xdevs::simulator::Config::new(0.0, 60.0, 1.0, None);

    simulator.simulate_rt(
        &config,
        xdevs::simulator::std::wait_event(&config, ihandler),
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
