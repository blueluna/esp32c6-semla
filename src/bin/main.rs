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
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Sender};
use embassy_sync::rwlock::RwLock;
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_graphics::geometry::Point;
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_bootloader_esp_idf::partitions;
use esp_bootloader_esp_idf::partitions::{DataPartitionSubType, PartitionType};
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
use jiff::ToSpan;
use panic_rtt_target as _;
use sntpc::NtpContext;
use sntpc::NtpTimestampGenerator;
use sntpc_net_embassy::UdpSocketWrapper;
use ssd1675::Color;

const BUFFER_SIZE: usize = ROWS as usize * COLS as usize / 8;

const ROWS: u16 = 212;
const COLS: u8 = 104;

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

#[derive(Debug, Clone, Copy, PartialEq)]
struct DisplayState {
    time: u64,
}

static DISPLAY_CHANNEL: Channel<CriticalSectionRawMutex, DisplayState, 4> = Channel::new();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    rtt_target::rtt_init_defmt!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let rtc = Rtc::new(peripherals.LPWR);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 64 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    let mut flash = FlashStorage::new(peripherals.FLASH);

    let mut pt_mem = [0u8; partitions::PARTITION_TABLE_MAX_LEN];
    let partition_table = partitions::read_partition_table(&mut flash, &mut pt_mem).unwrap();

    let mut configuration_partition_0 = usize::MAX;
    // let mut configuration_partition_1 = usize::MAX;

    for n in 0..partition_table.len() {
        let entry = partition_table.get_partition(n).unwrap();

        if entry.partition_type() == PartitionType::Data(DataPartitionSubType::Spiffs) {
            if entry.label_as_str() == "cfg_0" {
                configuration_partition_0 = n;
            }
            if entry.label_as_str() == "cfg_1" {
                // configuration_partition_1 = n;
            }
        }
    }

    let (wifi_ssid, wifi_password) = if configuration_partition_0 < usize::MAX {
        match partition_table.get_partition(configuration_partition_0) {
            Ok(partition) => {
                use embassy_embedded_hal::adapter::BlockingAsync;
                use sequential_storage::cache::KeyPointerCache;
                use sequential_storage::map::{MapConfig, MapStorage};
                let region = partition.as_embedded_storage(&mut flash);
                let storage = BlockingAsync::new(region);

                /*
                defmt::info!("Erasing configuration partition 0...");

                storage
                    .erase(0, 0x2000)
                    .await
                    .expect("Failed to erase storage");
                */

                const MAP_FLASH_RANGE: core::ops::Range<u32> = 0x000000..0x002000;

                let mut map_storage = MapStorage::new(
                    storage,
                    const { MapConfig::new(MAP_FLASH_RANGE) },
                    KeyPointerCache::<4, u8, 8>::new(),
                );

                let mut scratch = [0u8; 128];
                let wifi_ssid = match map_storage
                    .fetch_item::<String>(&mut scratch, &CONFIG_WIFI_SSID)
                    .await
                {
                    Ok(Some(ssid)) => {
                        defmt::info!("Loaded Wi-Fi SSID from storage: '{}'", &ssid);
                        ssid
                    }
                    Ok(None) => {
                        let ssid = String::from(SSID);
                        defmt::warn!("Wi-Fi SSID not found in storage");
                        if !ssid.is_empty() {
                            match map_storage
                                .store_item(&mut scratch, &CONFIG_WIFI_SSID, &ssid)
                                .await
                            {
                                Ok(()) => {
                                    defmt::info!("Stored default Wi-Fi SSID to storage");
                                }
                                Err(_) => {
                                    defmt::warn!("Failed to store default Wi-Fi SSID to storage");
                                }
                            }
                        }
                        ssid
                    }
                    Err(error) => {
                        match error {
                            sequential_storage::Error::Storage { value } => {
                                defmt::warn!("Storage error while loading Wi-Fi SSID: {:?}", value);
                            }
                            sequential_storage::Error::FullStorage => {
                                defmt::warn!("Storage full error while loading Wi-Fi SSID");
                            }
                            sequential_storage::Error::Corrupted {} => {
                                defmt::warn!("Corrupted data error while loading Wi-Fi SSID");
                            }
                            sequential_storage::Error::LogicBug {} => {
                                defmt::warn!("Logic bug error while loading Wi-Fi SSID");
                            }
                            sequential_storage::Error::BufferTooBig => {
                                defmt::warn!("Wi-Fi SSID buffer too big");
                            }
                            sequential_storage::Error::BufferTooSmall(size) => {
                                defmt::warn!("Wi-Fi SSID buffer too small, need {} bytes", size);
                            }
                            sequential_storage::Error::SerializationError(error) => {
                                defmt::warn!("Wi-Fi SSID serialization error: {:?}", error);
                            }
                            sequential_storage::Error::ItemTooBig => {
                                defmt::warn!("Wi-Fi SSID item too big");
                            }
                            _ => {
                                defmt::warn!("Unknown error while loading Wi-Fi SSID");
                            }
                        }
                        String::from(SSID)
                    }
                };

                let wifi_password = match map_storage
                    .fetch_item::<String>(&mut scratch, &CONFIG_WIFI_PASSWORD)
                    .await
                {
                    Ok(Some(password)) => {
                        defmt::info!("Loaded Wi-Fi password from storage");
                        password
                    }
                    Ok(None) => {
                        defmt::warn!("Wi-Fi password not found in storage");
                        let password = String::from(PASSWORD);
                        if !password.is_empty() {
                            match map_storage
                                .store_item(&mut scratch, &CONFIG_WIFI_PASSWORD, &password)
                                .await
                            {
                                Ok(()) => {
                                    defmt::info!("Stored default Wi-Fi password to storage");
                                }
                                Err(_) => {
                                    defmt::warn!(
                                        "Failed to store default Wi-Fi password to storage"
                                    );
                                }
                            }
                        }
                        String::from(PASSWORD)
                    }
                    Err(error) => {
                        match error {
                            sequential_storage::Error::Storage { value } => {
                                defmt::warn!(
                                    "Storage error while loading Wi-Fi password: {:?}",
                                    value
                                );
                            }
                            sequential_storage::Error::FullStorage => {
                                defmt::warn!("Storage full error while loading Wi-Fi password");
                            }
                            sequential_storage::Error::Corrupted {} => {
                                defmt::warn!("Corrupted data error while loading Wi-Fi password");
                            }
                            sequential_storage::Error::LogicBug {} => {
                                defmt::warn!("Logic bug error while loading Wi-Fi password");
                            }
                            sequential_storage::Error::BufferTooBig => {
                                defmt::warn!("Wi-Fi password buffer too big");
                            }
                            sequential_storage::Error::BufferTooSmall(size) => {
                                defmt::warn!(
                                    "Wi-Fi password buffer too small, need {} bytes",
                                    size
                                );
                            }
                            sequential_storage::Error::SerializationError(error) => {
                                defmt::warn!("Wi-Fi password serialization error: {:?}", error);
                            }
                            sequential_storage::Error::ItemTooBig => {
                                defmt::warn!("Wi-Fi password item too big");
                            }
                            _ => {
                                defmt::warn!("Unknown error while loading Wi-Fi password");
                            }
                        }
                        String::from(PASSWORD)
                    }
                };
                (wifi_ssid, wifi_password)
            }
            Err(e) => {
                defmt::error!("Failed to get configuration partition 0: {:?}", e);
                (String::from(SSID), String::from(PASSWORD))
            }
        }
    } else {
        defmt::warn!("No configuration partition found");
        (String::from(SSID), String::from(PASSWORD))
    };

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
        .rotation(ssd1675::Rotation::Rotate270)
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

    display_graphics.clear(Color::White);

    display_graphics
        .reset(&mut epd_delay)
        .expect("display reset");

    static NET_STACK: static_cell::StaticCell<embassy_net::Stack> = static_cell::StaticCell::new();
    static RTC: static_cell::StaticCell<Rtc> = static_cell::StaticCell::new();

    let _ = spawner.spawn(connection(wifi_controller, wifi_ssid, wifi_password));
    let _ = spawner.spawn(net_task(net_runner));

    let _ = spawner.spawn(network_stack(
        NET_STACK.init(net_stack),
        RTC.init(rtc),
        DISPLAY_CHANNEL.sender(),
    ));

    let mut year = 0;
    let mut fettisdag = jiff::civil::date(1970, 1, 1);

    loop {
        let state = DISPLAY_CHANNEL.receive().await;

        let timestamp = jiff::Timestamp::from_microsecond(state.time as i64).unwrap();
        let datetime = timestamp.to_zoned(TIMEZONE).datetime();

        if datetime.year() != year {
            fettisdag = get_fettisdag_date(datetime.year());
            if fettisdag < datetime.date() {
                defmt::info!("Fettisdag has passed, calculating for next year");
                fettisdag = get_fettisdag_date(datetime.year() + 1);
            }
            year = datetime.year();
        }

        let days_until_fettisdag = (fettisdag - datetime.date()).get_days();

        display_graphics.clear(Color::White);

        let small_font =
            u8g2_fonts::FontRenderer::new::<u8g2_fonts::fonts::u8g2_font_logisoso16_tf>();

        let big_font =
            u8g2_fonts::FontRenderer::new::<u8g2_fonts::fonts::u8g2_font_logisoso38_tf>();

        let time = jiff::Timestamp::from_microsecond(state.time as i64).unwrap();
        let date_str = format_date(&time);
        let _time_str = format_time(&time);

        use u8g2_fonts::types::{FontColor, HorizontalAlignment, VerticalPosition};

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

        display_graphics
            .update(&mut epd_delay)
            .expect("display update");
    }
}

const NTP_SERVER: &str = "pool.ntp.org";

/// Microseconds in a second
const USEC_IN_SEC: u64 = 1_000_000;

const TIMEZONE: jiff::tz::TimeZone = jiff::tz::get!("Europe/Stockholm");

#[derive(Clone, Copy)]
struct Timestamp<'a> {
    rtc: &'a Rtc<'a>,
    current_time_us: u64,
}

impl NtpTimestampGenerator for Timestamp<'_> {
    fn init(&mut self) {
        self.current_time_us = self.rtc.current_time_us();
    }

    fn timestamp_sec(&self) -> u64 {
        self.current_time_us / 1_000_000
    }

    fn timestamp_subsec_micros(&self) -> u32 {
        (self.current_time_us % 1_000_000) as u32
    }
}

static INSTANCE_OFFSET_US: RwLock<CriticalSectionRawMutex, u64> = RwLock::new(0);

fn format_zoned(ts: &jiff::Zoned) -> heapless::String<32> {
    uformat!(
        32,
        "{:4}-{:02}-{:02} {:02}:{:02}:{:02}.{:03}",
        ts.year(),
        ts.month(),
        ts.day(),
        ts.hour(),
        ts.minute(),
        ts.second(),
        ts.millisecond()
    )
    .unwrap()
}

fn format_timestamp(ts: &jiff::Timestamp) -> heapless::String<32> {
    format_zoned(&ts.to_zoned(TIMEZONE))
}

fn format_date(ts: &jiff::Timestamp) -> heapless::String<16> {
    let utc = ts.to_zoned(TIMEZONE);
    uformat!(16, "{:4}-{:02}-{:02}", utc.year(), utc.month(), utc.day(),).unwrap()
}

fn format_time(ts: &jiff::Timestamp) -> heapless::String<8> {
    let utc = ts.to_zoned(TIMEZONE);
    uformat!(8, "{:02}:{:02}", utc.hour(), utc.minute(),).unwrap()
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

#[embassy_executor::task]
pub async fn network_stack(
    stack: &'static embassy_net::Stack<'static>,
    rtc: &'static Rtc<'static>,
    channel: Sender<'static, CriticalSectionRawMutex, DisplayState, 4>,
) {
    defmt::info!("Hardware address: {:?}", stack.hardware_address());

    stack.wait_link_up().await;
    defmt::info!("Link is up");
    stack.wait_config_up().await;
    defmt::info!("Network configured");

    defmt::info!("Query DNS for NTP server address: {}", NTP_SERVER);

    let ntp_addrs = stack.dns_query(NTP_SERVER, DnsQueryType::A).await.unwrap();

    if ntp_addrs.is_empty() {
        panic!("Failed to resolve DNS. Empty result");
    }

    let now = jiff::Timestamp::from_microsecond(rtc.current_time_us() as i64).unwrap();
    defmt::info!("RTC time {}", format_timestamp(&now));

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

    let mut display_state = DisplayState { time: 0 };

    const HOUR_SECONDS: u64 = 60 * 60;
    const HOUR: Duration = Duration::from_secs(HOUR_SECONDS);

    loop {
        let addr: core::net::IpAddr = ntp_addrs[0].into();
        let expire_at = match sntpc::get_time(
            core::net::SocketAddr::from((addr, 123)),
            &ntp_socket,
            NtpContext::new(Timestamp {
                rtc: &rtc,
                current_time_us: 0,
            }),
        )
        .await
        {
            Ok(ntp_result) => {
                let current_elapsed = Instant::now();
                let ntp_usec = (u64::from(ntp_result.sec()) * USEC_IN_SEC)
                    + (u64::from(ntp_result.sec_fraction()) * USEC_IN_SEC >> 32);
                let instance_offset_us = ntp_usec - current_elapsed.as_micros();
                {
                    let mut writer = INSTANCE_OFFSET_US.write().await;
                    *writer = instance_offset_us;
                }
                let ntp_now = jiff::Timestamp::from_microsecond(ntp_usec as i64).unwrap();

                let datetime = ntp_now.to_zoned(TIMEZONE).datetime();
                let next_day = datetime.date().tomorrow().unwrap();
                let midnight = next_day.at(0, 0, 0, 0);
                let until_midnight = datetime.duration_until(midnight);
                let seconds_until_midnight = until_midnight.as_secs() as u64;
                let midnight_instant =
                    current_elapsed + Duration::from_secs(seconds_until_midnight);
                display_state.time = ntp_usec;
                midnight_instant
            }
            Err(e) => {
                defmt::error!("Failed to get NTP time: {:?}", e);
                Instant::now() + HOUR
            }
        };
        channel.send(display_state).await;
        defmt::info!(
            "Next NTP update in {} seconds",
            (expire_at - Instant::now()).as_secs()
        );
        Timer::at(expire_at).await;
    }
}
