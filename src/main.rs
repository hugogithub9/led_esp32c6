mod Generator {
    use rand::Rng;
    pub struct GeneratorState {
        sigma: f64,
        period :f64,
        clock:f64,
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
            }
            else {
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

mod Activator {
    use esp_idf_hal::gpio::{PinDriver, Output, Gpio8};/////////
    pub struct ActivatorState {
        sigma: f64,
        clock:f64,
        count: usize,
        led: PinDriver<'a, Output<Gpio8>>,/////////////////
    }
 
    impl ActivatorState {
        pub fn new(led: PinDriver<'a, Output<Gpio8>>) -> Self {///////////////
            Self {
                sigma: 0.0,
                clock: 0.0,
                count: 0,
                led,////////////////
            }
        }
    }
 

    xdevs::component!(
        ident = Activator,
        input = {
        input<bool>,
        },
        state = ActivatorState,
    );
 
    impl xdevs::Atomic for Activator {
        fn delta_int(state: &mut Self::State) {
            state.clock += state.sigma;
            state.sigma = f64::INFINITY;
        }
 
        fn lambda(state: &Self::State, output: &mut Self::Output) {
 
        }
 
        fn ta(state: &Self::State) -> f64 {
            state.sigma
        }

        fn delta_ext(state: &mut Self::State, e: f64, input: &Self::Input) {
            state.sigma -= e;
            state.clock += e;
 
            let values = unsafe { input.input.get_values() };

            for value in values.iter() {
                if *value {
                    println!("LED ON");
                    //line to change LED state :
                    state.led.set_high().unwrap();/////////
                }
                else {
                    println!("LED OFF");
                    //line to change LED state:
                    state.led.set_high().unwrap();//////////
                }
            }
        }
    }
}


xdevs::component!(
    ident = LED,
    components = {
        generator: Generator::Generator,
        activator: Activator::Activator,
    },
    couplings = {
        generator.output -> activator.input,
    }
);


fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    println!("Hello, world!");

    use esp_idf_hal::prelude::*;
    use esp_idf_hal::gpio::*;


    let period=3.;

    let peripherals = Peripherals::take().unwrap();
    let led_pin = PinDriver::output(peripherals.pins.gpio8).unwrap();

    let generator = Generator::Generator::new(Generator::GeneratorState::new(period));
    let activator = Activator::Activator::new(Activator::ActivatorState::new(led_pin));
    let led = LED::new(generator, activator);
 
    let mut simulator = xdevs::simulator::Simulator::new(led);
    simulator.simulate_vt( //start,stop
        0.0,
        60.0,
    );
}
