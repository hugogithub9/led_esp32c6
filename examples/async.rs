//this code wait 5sec OR the button to be pressed
//using programmation asynchronous with embassy without blocking the system

use anyhow::Result;
use esp_idf_hal::gpio::*;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::task::*;

//use esp_idf_svc::timer::embassy_time_driver;

use futures::{select, FutureExt};

extern crate embassy_executor;
extern crate esp_idf_svc;

use embassy_executor::Executor;
use embassy_time::{Duration, Timer}; //use Instant after
use static_cell::StaticCell; //necessary to execute asynchrone task

async fn wait_button(button: &mut PinDriver<'static, Gpio14, Input>) {
    let _ = button.wait_for_high().await; //wait to be pressed
    println!("button pressed!");
}

async fn wait_timeout(duration: Duration) {
    Timer::after(duration).await;
    println!("time finished") //wait during duration
}

//fct with no blocking programme logic
#[embassy_executor::task]
async fn main_task(mut button: PinDriver<'static, Gpio14, Input>) {
    loop {
        //creation of 2 concurents futures
        let button_future = wait_button(&mut button).fuse(); //wait button
        let timeout_future = wait_timeout(Duration::from_secs(5)).fuse(); //wait 5sec

        futures::pin_mut!(button_future, timeout_future); //necessary to use select

        select! {//act like wait until first future completes
            _ = button_future => {//if button ...
                println!("Button detected !");
            },
            _ = timeout_future => {
                println!("Time elapsed !");
            }
        }
    }
}

fn main() -> Result<()> {
    esp_idf_hal::sys::link_patches();

    let peripherals = Peripherals::take()?;
    let mut button = PinDriver::input(peripherals.pins.gpio14)?;
    button.set_pull(Pull::Down)?;

    // creation of executor async static embassy
    static EXECUTOR: StaticCell<Executor> = StaticCell::new();
    let executor = EXECUTOR.init(Executor::new());

    //launch of embassy executor and start of asynchronous task
    executor.run(|spawner| {
        // Spawner Embassy : launch task async
        spawner.spawn(main_task(button)).unwrap();
    });
    Ok(())
}
