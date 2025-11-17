#![no_std]
#![no_main]

use defmt_or_log::*;
use embassy_executor::Spawner;
#[cfg(feature = "log")]
use embassy_rp::bind_interrupts;
#[cfg(feature = "log")]
use embassy_rp::peripherals::USB;
#[cfg(feature = "log")]
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

#[cfg(feature = "log")]
bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

#[cfg(feature = "log")]
#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

#[embassy_executor::task]
async fn heartbeat_task(mut output: embassy_rp::gpio::Output<'static>) {
    loop {
        output.toggle();
        Timer::after_secs(1).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    #[cfg(feature = "log")]
    {
        let driver = Driver::new(p.USB, Irqs);
        spawner.spawn(logger_task(driver).unwrap());
    }
    let heartbeat_led = embassy_rp::gpio::Output::new(p.PIN_5, embassy_rp::gpio::Level::Low);
    spawner.spawn(heartbeat_task(heartbeat_led).unwrap());

    for _ in 0..3 {
        info!(".");
        Timer::after_secs(1).await;
    }
    info!("Staartstuk Controller started!");

    Timer::after_secs(1).await;

    let mut counter = 0;
    loop {
        counter += 1;
        info!("Ticker {}", counter);
        Timer::after_secs(1).await;
    }
}
