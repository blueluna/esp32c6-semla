extern crate alloc;

use alloc::string::String;
use embassy_net::Runner;
use embassy_time::{Duration, Timer};
use esp_radio::wifi::{
    ClientConfig, ModeConfig, WifiController, WifiDevice, WifiEvent, WifiStaState,
};
use defmt::info;

#[embassy_executor::task]
pub async fn connection(mut controller: WifiController<'static>, ssid: String, password: String) {
    loop {
        match esp_radio::wifi::sta_state() {
            WifiStaState::Connected => {
                // wait until we're no longer connected
                controller.wait_for_event(WifiEvent::StaDisconnected).await;
                Timer::after(Duration::from_millis(5000)).await
            }
            _ => {}
        }
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = ModeConfig::Client(
                ClientConfig::default()
                    .with_ssid(ssid.clone())
                    .with_password(password.clone()),
            );
            controller.set_config(&client_config).unwrap();
            controller.start_async().await.unwrap();
            info!("Wifi started!");
        }
        info!("Associate with wifi network: {}", ssid);

        match controller.connect_async().await {
            Ok(_) => info!("Wifi connected!"),
            Err(e) => {
                info!("Failed to connect to wifi: {:?}", e);
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

#[embassy_executor::task]
pub async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    runner.run().await
}
