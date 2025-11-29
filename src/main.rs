#![no_std]
#![no_main]

use defmt_or_log::*;
use embassy_executor::Spawner;
use embassy_rp::adc::Adc;
#[cfg(feature = "log")]
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::Pull;
#[cfg(feature = "log")]
use embassy_rp::peripherals::USB;
use embassy_rp::pwm::SetDutyCycle;
#[cfg(feature = "log")]
use embassy_rp::usb::Driver;
use embassy_time::{Duration, Ticker, Timer};
use micromath::F32Ext;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    #[cfg(feature = "log")]
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<USB>;
    ADC_IRQ_FIFO => embassy_rp::adc::InterruptHandler;
});

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

    let cfg: embassy_rp::pwm::Config = Default::default();
    let pwm_input = embassy_rp::pwm::Pwm::new_input(
        p.PWM_SLICE6,
        p.PIN_13,
        Pull::Up,
        embassy_rp::pwm::InputMode::RisingEdge,
        cfg,
    );
    spawner.spawn(monitor_flow_task(pwm_input).unwrap());

    let adc = Adc::new(p.ADC, Irqs, embassy_rp::adc::Config::default());
    let adc_mcu_temp = embassy_rp::adc::Channel::new_temp_sensor(p.ADC_TEMP_SENSOR);
    let adc_water_temp = embassy_rp::adc::Channel::new_pin(p.PIN_27, Pull::None);
    spawner.spawn(monitor_temperature_task(adc, adc_mcu_temp, adc_water_temp).unwrap());

    // PWM for water pump control, slowly ramp up duty cycle to limit inrush current
    let desired_freq_hz = 10_000;
    let clock_freq_hz = embassy_rp::clocks::clk_sys_freq();
    let divider = 16u8;
    let period = (clock_freq_hz / (desired_freq_hz * divider as u32)) as u16 - 1;

    let mut c = embassy_rp::pwm::Config::default();
    c.top = period;
    c.divider = divider.into();
    let mut pwm_output = embassy_rp::pwm::Pwm::new_output_b(p.PWM_SLICE3, p.PIN_23, c.clone());
    pwm_output.set_duty_cycle_fully_off().unwrap();
    for pump_duty_cycle in (0..=100).step_by(5) {
        pwm_output.set_duty_cycle_percent(pump_duty_cycle).unwrap();
        info!("Pump duty cycle set to {}%", pump_duty_cycle);
        Timer::after(Duration::from_millis(50)).await;
    }
    pwm_output.set_duty_cycle_fully_on().unwrap();

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}

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

#[embassy_executor::task]
async fn monitor_flow_task(pwm: embassy_rp::pwm::Pwm<'static>) {
    // 22.9 Hz (measured) at about 1,88L/min (by datasheet)
    let mut ticker = Ticker::every(Duration::from_secs(10));
    loop {
        let frequency = pwm.counter() as f32 / 10.0;
        info!("Flow meter input frequency: {:.1} Hz", frequency);
        let liter_min = frequency / (22.9 / 1.88);
        info!("Estimated flow: {:.2} L/min", liter_min);
        pwm.set_counter(0);
        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn monitor_temperature_task(
    mut adc: Adc<'static, embassy_rp::adc::Async>,
    mut adc_mcu_temp: embassy_rp::adc::Channel<'static>,
    mut adc_water_temp: embassy_rp::adc::Channel<'static>,
) {
    loop {
        let mcu_temp = adc.read(&mut adc_mcu_temp).await.unwrap();
        info!("MCU Temp:     {:.1} °C", mcu_convert_to_celsius(mcu_temp));
        let water_temp = adc.read(&mut adc_water_temp).await.unwrap();
        info!(
            "Water Temp:   {:.1} °C",
            flow_meter_convert_to_celsius(water_temp)
        );
        Timer::after(Duration::from_secs(1)).await;
    }
}

fn mcu_convert_to_celsius(raw_temp: u16) -> f32 {
    // According to chapter 4.9.5. Temperature Sensor in RP2040 datasheet
    27.0 - (raw_temp as f32 * 3.3 / 4096.0 - 0.706) / 0.001721
}

fn flow_meter_convert_to_celsius(raw_temp: u16) -> f32 {
    // top resistor 51k plus 1k so 52k total
    // bottom resistor 50k (at 25°C) ntc (b value 3950)
    // voltage divider output to adc

    const R_TOP: f32 = 52000.0; // 52kΩ
    // const R_NTC_25C: f32 = 50000.0; // 50kΩ at 25°C
    const R_NTC_25C: f32 = 48000.0; //(calibrated with ice water)
    const B_VALUE: f32 = 3950.0;
    const T_25C: f32 = 298.15; // 25°C in Kelvin
    const ADC_MAX: f32 = 4096.0;

    // Calculate NTC resistance from voltage divider ratio
    // ADC/ADC_MAX = R_ntc / (R_top + R_ntc)
    // R_ntc = (ADC * R_top) / (ADC_MAX - ADC)
    let r_ntc = (raw_temp as f32 * R_TOP) / (ADC_MAX - raw_temp as f32);

    // Steinhart-Hart simplified (B parameter equation)
    // 1/T = 1/T0 + (1/B) * ln(R/R0)
    let temp_kelvin = 1.0 / (1.0 / T_25C + (1.0 / B_VALUE) * (r_ntc / R_NTC_25C).ln());
    temp_kelvin - 273.15
}
