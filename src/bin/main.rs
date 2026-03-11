#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

extern crate alloc;

use alloc::string::String;
use core::convert::From;
use efmt::uformat;
use embassy_executor::Spawner;
use embassy_net::dns::DnsQueryType;
use embassy_net::tcp::TcpSocket;
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_time::{Delay, Duration};
use embedded_graphics::geometry::Point;
use embedded_hal::delay::DelayNs;
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig};
use esp_hal::rng::Rng;
use esp_hal::rtc_cntl::Rtc;
use esp_hal::spi::{
    Mode,
    master::{Config, Spi},
};
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{
    dma::{DmaRxBuf, DmaTxBuf},
    dma_buffers,
};
use esp_storage::FlashStorage;
use esp32c6_semla::network::{connection, net_task};
use esp32c6_semla::storage::Storage;
use jiff::ToSpan;
use defmt::{error, info, warn};
use sntpc::NtpContext;
use sntpc::NtpTimestampGenerator;
use sntpc_net_embassy::UdpSocketWrapper;
use ssd1675::Color;

use {panic_rtt_target as _};

const ROWS: u16 = 212;
const COLS: u8 = 104;

const BUFFER_SIZE: usize = ROWS as usize * COLS as usize / 8;

static mut BLACK_BUFFER: [u8; BUFFER_SIZE] = [0u8; BUFFER_SIZE];
static mut RED_BUFFER: [u8; BUFFER_SIZE] = [0u8; BUFFER_SIZE];

#[rustfmt::skip]
#[allow(dead_code)]
const LUT_BW: [u8; 70] = [
    // Phase 0     Phase 1     Phase 2     Phase 3     Phase 4     Phase 5     Phase 6
    // A B C D     A B C D     A B C D     A B C D     A B C D     A B C D     A B C D
    0b01001000, 0b10100000, 0b00010000, 0b00010000, 0b00010011, 0b00000000, 0b00000000,  // LUT0 - Black
    0b01001000, 0b10100000, 0b10000000, 0b00000000, 0b00000011, 0b00000000, 0b00000000,  // LUT1 - White
    0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,  // IGNORE
    0b01001000, 0b10100101, 0b00000000, 0b10111011, 0b00000000, 0b00000000, 0b00000000,  // LUT3 - Red
    0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,  // LUT4 - VCOM

// Duration        | Repeat
//   A   B   C   D |
    16,  4,  4,  4,  4, // 0 Flash
    16,  4,  4,  4,  4, // 1 clear
     4,  8,  8, 16,  8, // 2 bring in the black
     2,  2,  2, 64,  8, // 3 time for red
     0,  0,  0,  0,  0, // 4 final black sharpen phase
     0,  0,  0,  0,  0, // 5
     0,  0,  0,  0,  0  // 6
    ];

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

const CONFIG_WIFI_SSID: u8 = 0x10;
const CONFIG_WIFI_PASSWORD: u8 = 0x11;

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    rtt_target::rtt_init_defmt!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let wake_reason = esp_hal::rtc_cntl::wakeup_cause();
    match wake_reason {
        esp_hal::system::SleepSource::Timer => info!("Woke up from timer!"),
        esp_hal::system::SleepSource::Ulp => info!("Woke up from ULP!"),
        esp_hal::system::SleepSource::Gpio => info!("Woke up from GPIO!"),
        esp_hal::system::SleepSource::Ext0 => info!("Woke up from EXT0!"),
        esp_hal::system::SleepSource::Ext1 => info!("Woke up from EXT1!"),
        esp_hal::system::SleepSource::TouchPad => info!("Woke up from touchpad!"),
        esp_hal::system::SleepSource::Undefined => info!("Woke up from undefined source!"),
        esp_hal::system::SleepSource::All => info!("Woke up from multiple sources!"),
        esp_hal::system::SleepSource::Uart => info!("Woke up from UART!"),
        esp_hal::system::SleepSource::Wifi => info!("Woke up from WiFi!"),
        esp_hal::system::SleepSource::BT => info!("Woke up from BT!"),
        esp_hal::system::SleepSource::Cocpu => info!("Woke up from CoCPU!"),
        esp_hal::system::SleepSource::CocpuTrapTrig => info!("Woke up from CoCPU!"),
    }

    let mut rtc = Rtc::new(peripherals.LPWR);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 64 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    let environment_ssid = String::from(SSID);
    let environment_password = String::from(PASSWORD);

    let mut flash = FlashStorage::new(peripherals.FLASH);
    let (wifi_ssid, wifi_password) = match Storage::new(&mut flash) {
        Some(mut storage) => {
            match (
                storage.get_string(CONFIG_WIFI_SSID).await,
                storage.get_string(CONFIG_WIFI_PASSWORD).await,
            ) {
                (Some(ssid), Some(password)) => {
                    (ssid, password)
                }
                (Some(ssid), None) => {
                    if !environment_password.is_empty() {
                        storage
                            .set_string(CONFIG_WIFI_PASSWORD, environment_password.clone())
                            .await;
                    }
                    (ssid, environment_password)
                }
                (None, Some(password)) => {
                    if !environment_ssid.is_empty() {
                        storage
                            .set_string(CONFIG_WIFI_SSID, environment_ssid.clone())
                            .await;
                    }
                    (environment_ssid, password)
                }
                _ => {
                    if !environment_ssid.is_empty() {
                        storage
                            .set_string(CONFIG_WIFI_SSID, environment_ssid.clone())
                            .await;
                    }
                    if !environment_password.is_empty() {
                        storage
                            .set_string(CONFIG_WIFI_PASSWORD, environment_password.clone())
                            .await;
                    }
                    (environment_ssid, environment_password)
                }
            }
        }
        None => (environment_ssid, environment_password),
    };

    if wifi_ssid.is_empty() || wifi_password.is_empty() {
        warn!("Wi-Fi credentials not set. Please set SSID and PASSWORD environment variables.");
    }

    static ESP_RADIO_CONTROLLER: static_cell::StaticCell<esp_radio::Controller> =
        static_cell::StaticCell::new();
    let radio_controller = ESP_RADIO_CONTROLLER
        .init(esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller"));

    let (wifi_controller, wifi_interfaces) =
        esp_radio::wifi::new(radio_controller, peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");

    let wifi_interface = wifi_interfaces.sta;

    let config = embassy_net::Config::dhcpv4(Default::default());

    let rng = Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    static NETWORK_RESOURCES: static_cell::StaticCell<embassy_net::StackResources<4>> =
        static_cell::StaticCell::new();

    // Init network stack
    let (net_stack, net_runner) = embassy_net::new(
        wifi_interface,
        config,
        NETWORK_RESOURCES.init(embassy_net::StackResources::new()),
        seed,
    );

    let dma_channel = peripherals.DMA_CH0;
    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(32000);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    // GPIO pins
    // 18 - BUSY
    // 19 - RST
    // 05 - DC
    // 04 - CS
    // 03 - SCK
    // 02 - MOSI

    let epd_sck = peripherals.GPIO3;
    let epd_mosi = peripherals.GPIO2;

    let epd_cs = Output::new(peripherals.GPIO4, Level::High, OutputConfig::default());
    let epd_dc = Output::new(peripherals.GPIO5, Level::High, OutputConfig::default());
    let epd_busy = Input::new(peripherals.GPIO18, InputConfig::default());
    let epd_rst = Output::new(peripherals.GPIO19, Level::High, OutputConfig::default());

    let epd_spi = Spi::new(
        peripherals.SPI2,
        Config::default()
            .with_frequency(Rate::from_khz(100))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(epd_sck)
    .with_mosi(epd_mosi)
    .with_dma(dma_channel)
    .with_buffers(dma_rx_buf, dma_tx_buf);

    let epd_spi_dev = ExclusiveDevice::new(epd_spi, epd_cs, Delay).expect("SPI device");

    let mut epd_delay = Delay;

    let controller = ssd1675::Interface::new(epd_spi_dev, epd_busy, epd_dc, epd_rst);
    let config = if let Ok(cfg) = ssd1675::Builder::new()
        .dimensions(ssd1675::Dimensions {
            rows: ROWS,
            cols: COLS,
        })
        .rotation(ssd1675::Rotation::Rotate90)
        .lut(&LUT_BW)
        .build()
    {
        cfg
    } else {
        panic!("Failed to configure display");
    };

    let display = ssd1675::Display::new(controller, config);
    let black_buffer = unsafe { &mut BLACK_BUFFER[..] };
    let red_buffer = unsafe { &mut RED_BUFFER[..] };
    let mut display_graphics = ssd1675::GraphicDisplay::new(display, black_buffer, red_buffer);

    match display_graphics.reset(&mut epd_delay) {
        Ok(_) => (),
        Err(_) => error!("Failed to reset display"),
    }

    display_graphics.clear(Color::White);

    display_graphics
        .reset(&mut epd_delay)
        .expect("display reset");

    static NET_STACK: static_cell::StaticCell<embassy_net::Stack> = static_cell::StaticCell::new();

    let _ = spawner.spawn(connection(wifi_controller, wifi_ssid, wifi_password));
    let _ = spawner.spawn(net_task(net_runner));

    let time = get_ntp_time(NET_STACK.init(net_stack)).await;

    display_graphics.clear(Color::White);

    let small_font = u8g2_fonts::FontRenderer::new::<u8g2_fonts::fonts::u8g2_font_logisoso16_tf>();

    let big_font = u8g2_fonts::FontRenderer::new::<u8g2_fonts::fonts::u8g2_font_logisoso38_tf>();

    use u8g2_fonts::types::{FontColor, HorizontalAlignment, VerticalPosition};

    let sleep_seconds = if let Some(time) = time {
        let timestamp = jiff::Timestamp::from_microsecond(time as i64).unwrap();
        let datetime = timestamp.to_zoned(TIMEZONE).datetime();

        let fettisdag = get_fettisdag_date(datetime.year());
        let fettisdag = if fettisdag < datetime.date() {
            // If we've already passed Fettisdag this year, calculate for next year
            get_fettisdag_date(datetime.year() + 1)
        } else {
            fettisdag
        };

        let days_until_fettisdag = (fettisdag - datetime.date()).get_days();

        let date_str = format_date(&timestamp);
        let time_str = format_time(&timestamp);

        info!(
            "Current time: {} {}, Days until Fettisdagen: {}",
            date_str, time_str, days_until_fettisdag
        );

        let pos = Point::new(ROWS as i32 / 2, 20);

        small_font
            .render_aligned(
                date_str.as_ref(),
                pos,
                VerticalPosition::Baseline,
                HorizontalAlignment::Center,
                FontColor::Transparent(Color::Black),
                &mut display_graphics,
            )
            .expect("font render");

        let pos = pos + Point::new(0, 48);

        let easter_str = uformat!(24, "{}", days_until_fettisdag).unwrap();
        big_font
            .render_aligned(
                easter_str.as_ref(),
                pos,
                VerticalPosition::Baseline,
                HorizontalAlignment::Center,
                FontColor::Transparent(Color::Black),
                &mut display_graphics,
            )
            .expect("font render");

        let pos = pos + Point::new(0, 28);

        let easter_str = uformat!(24, "dagar till semla").unwrap();
        small_font
            .render_aligned(
                easter_str.as_ref(),
                pos,
                VerticalPosition::Baseline,
                HorizontalAlignment::Center,
                FontColor::Transparent(Color::Black),
                &mut display_graphics,
            )
            .expect("font render");

        let next_day = datetime.date().tomorrow().unwrap();
        // 5 minutes after midnight to be sure that it wakes up after the date has changed
        let midnight = next_day.at(1, 30, 0, 0);
        let until_midnight = datetime.duration_until(midnight);
        let seconds_until_midnight = until_midnight.as_secs() as u64;
        seconds_until_midnight
    } else {
        let pos = Point::new(ROWS as i32 / 2, COLS as i32 / 2);
        small_font
            .render_aligned(
                "Failed to get time",
                pos,
                VerticalPosition::Center,
                HorizontalAlignment::Center,
                FontColor::Transparent(Color::Black),
                &mut display_graphics,
            )
            .expect("font render");
        warn!("Failed to get time");
        60 * 5
    };

    match display_graphics.update(&mut epd_delay) {
        Ok(_) => (),
        Err(_) => error!("Failed to update display"),
    }
    match display_graphics.deep_sleep() {
        Ok(_) => (),
        Err(_) => error!("Failed to put display into deep sleep"),
    }

    let mut sleep_delay = embassy_time::Delay;
    let sleep_text = format_seconds(&sleep_seconds);
    info!("Entering deep sleep for {}", sleep_text);
    enter_deep_sleep(&mut rtc, &mut sleep_delay, sleep_seconds);
}

const NTP_SERVER: &str = "pool.ntp.org";

/// Microseconds in a second
const USEC_IN_SEC: u64 = 1_000_000;

const TIMEZONE: jiff::tz::TimeZone = jiff::tz::get!("Europe/Stockholm");

#[derive(Clone, Copy)]
struct Timestamp {
    current_time_us: u64,
}

impl NtpTimestampGenerator for Timestamp {
    fn init(&mut self) {
        self.current_time_us = 0;
    }

    fn timestamp_sec(&self) -> u64 {
        self.current_time_us / 1_000_000
    }

    fn timestamp_subsec_micros(&self) -> u32 {
        (self.current_time_us % 1_000_000) as u32
    }
}

fn format_date(ts: &jiff::Timestamp) -> heapless::String<16> {
    let utc = ts.to_zoned(TIMEZONE);
    uformat!(16, "{:4}-{:02}-{:02}", utc.year(), utc.month(), utc.day(),).unwrap()
}

fn format_time(ts: &jiff::Timestamp) -> heapless::String<8> {
    let utc = ts.to_zoned(TIMEZONE);
    uformat!(8, "{:02}:{:02}", utc.hour(), utc.minute(),).unwrap()
}

fn format_seconds(seconds: &u64) -> heapless::String<16> {
    let hours = seconds / 3600;
    let minutes = (seconds % 3600) / 60;
    let seconds = seconds % 60;
    uformat!(16, "{:02}:{:02}:{:02}", hours, minutes, seconds).unwrap()
}

/// Returns the (month, day) of Easter Sunday for the given year using the
/// Meeus/Jones/Butcher algorithm.
fn get_easter_date(year: i32) -> jiff::civil::Date {
    let a = year % 19;
    let b = year / 100;
    let c = year % 100;
    let d = b / 4;
    let e = b % 4;
    let f = (b + 8) / 25;
    let g = (b - f + 1) / 3;
    let h = (19 * a + b - d - g + 15) % 30;
    let i = c / 4;
    let k = c % 4;
    let l = (32 + 2 * e + 2 * i - h - k) % 7;
    let m = (a + 11 * h + 22 * l) / 451;
    let month = (h + l - 7 * m + 114) / 31;
    let day = (h + l - 7 * m + 114) % 31 + 1;
    jiff::civil::date(year as i16, month as i8, day as i8)
}

// Returns the date of Fettisdag (Shrove Tuesday) for the given year, which is 47 days before Easter Sunday.
fn get_fettisdag_date(year: i16) -> jiff::civil::Date {
    get_easter_date(year as i32) - 47.days()
}

pub async fn get_ntp_time(stack: &'static embassy_net::Stack<'static>) -> Option<u64> {
    info!("Hardware address: {:?}", stack.hardware_address());
    stack.wait_link_up().await;
    info!("Link is up");
    stack.wait_config_up().await;
    info!("Network configured");

    let ntp_addrs = stack.dns_query(NTP_SERVER, DnsQueryType::A).await.unwrap();

    if ntp_addrs.is_empty() {
        warn!("Failed to resolve DNS. Empty result");
        return None;
    }

    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];

    let mut socket = TcpSocket::new(*stack, &mut rx_buffer, &mut tx_buffer);
    socket.set_timeout(Some(Duration::from_secs(60)));

    let mut ntp_rx_meta = [PacketMetadata::EMPTY; 8];
    let mut ntp_rx_buffer = [0; 512];
    let mut ntp_tx_meta = [PacketMetadata::EMPTY; 8];
    let mut ntp_tx_buffer = [0; 512];

    // Within an embassy async context
    let mut ntp_socket = UdpSocket::new(
        *stack,
        &mut ntp_rx_meta,
        &mut ntp_rx_buffer,
        &mut ntp_tx_meta,
        &mut ntp_tx_buffer,
    );
    ntp_socket.bind(123).unwrap();
    // binding and other required steps
    let ntp_socket = UdpSocketWrapper::from(ntp_socket);

    let addr: core::net::IpAddr = ntp_addrs[0].into();
    let time_usec = match sntpc::get_time(
        core::net::SocketAddr::from((addr, 123)),
        &ntp_socket,
        NtpContext::new(Timestamp { current_time_us: 0 }),
    )
    .await
    {
        Ok(ntp_result) => {
            let ntp_usec = (u64::from(ntp_result.sec()) * USEC_IN_SEC)
                + (u64::from(ntp_result.sec_fraction()) * USEC_IN_SEC >> 32);
            Some(ntp_usec)
        }
        Err(e) => {
            error!("Failed to get NTP time: {:?}", e);
            None
        }
    };
    time_usec
}

/// Enter deep sleep with timer and KEY button (GPIO4) wake sources
fn enter_deep_sleep(
    rtc: &mut esp_hal::rtc_cntl::Rtc,
    delay: &mut embassy_time::Delay,
    seconds: u64,
) -> ! {
    // Configure wake sources
    let timer =
        esp_hal::rtc_cntl::sleep::TimerWakeupSource::new(core::time::Duration::from_secs(seconds));

    let sleep_cfg = esp_hal::rtc_cntl::sleep::RtcSleepConfig::deep();

    // Small delay to let serial output flush
    delay.delay_ms(100);

    // Enter deep sleep (never returns - device reboots on wake)
    rtc.sleep(&sleep_cfg, &[&timer]);
    unreachable!();
}
