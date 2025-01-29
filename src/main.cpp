#include <ArduinoBLE.h>

#ifdef NANO_33_BLE
#include <Arduino_BMI270_BMM150.h>
#endif

#ifdef NANO_33_IOT
#include <Arduino_LSM6DS3.h>
#endif

#include "log.h"
#include "utils.h"
#include "Btn.h"
#include "IMUProcessor.h"


const pin_size_t GPIO_BTN_1 = 2; // btn toward outside
const pin_size_t GPIO_BTN_2 = 3; // btn toward center

Btn btn1(GPIO_BTN_1);
Btn btn2(GPIO_BTN_2);
IMUProcessor imu_processor;

enum AppState
{
    Idle,
    Scanning,
    Connected,
};

AppState app_state;

void change_app_state(AppState new_state);
void Idle_enter();

void setup()
{
#ifdef DEBUG_OVER_SERIAL
    // setup serial
    Serial.begin(115200);
    // while (!Serial);
    delay(1000);
    Serial.println(SERIAL_WELCOME);
#endif
    delay(1000);

    // setup leds
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

#ifdef NANO_33_BLE
    digitalWrite(LED_POWER, LOW);
#endif

    // setup app
    app_state = AppState::Idle;
    Idle_enter();

    btn1.begin();
    btn2.begin();

    // setup IMU
    if (!IMU.begin())
    {
        LOG("ERROR: Failed to initialize IMU!");
        while (1) {}
    }

    float SR_acceleration = IMU.accelerationSampleRate();
    float SR_gyroscope = IMU.gyroscopeSampleRate();

    if(SR_acceleration == SR_gyroscope) {
        LOG_("IMU sample rate: ");  LOG(SR_acceleration);
        imu_processor.begin(SR_acceleration);
    }
    else {
        LOG_("ERROR: IMU sample rate mismatch (a/g): "); LOG(SR_acceleration); LOG(SR_gyroscope);
        while(1) {}
    }

    // setup BLE
    if (!BLE.begin())
    {
        LOG("ERROR: Failed to initialize BLE!");
        while (1) {}
    }

    // start scanning
    change_app_state(AppState::Scanning);
}

BLEDevice selected_peripheral;
BLECharacteristic tx_characteristic;
BLECharacteristic rx_characteristic;
int event_count = 0; // debug only

void Idle_enter()
{
    set_led_state(LedState::NotConnected);
}
void Idle_loop() {}
void Idle_exit() {}

void Scanning_enter()
{
    LOG("Scanning...");
    BLE.scanForUuid("53300001-0023-4bd4-bbd5-a6920e4c5653", true);
    set_led_state(LedState::Scanning);
}

void Scanning_loop()
{
    BLEDevice peripheral = BLE.available();

    if (peripheral)
    {
        // discovered a peripheral
        LOG_("Found: ");
        LOG(peripheral.localName());

        selected_peripheral = peripheral;
        change_app_state(AppState::Connected);
        return;
    }
}

void Scanning_exit()
{
    BLE.stopScan();
    LOG("Stop Scan!");
}

void Connected_enter()
{
    LOG("Connecting...");
    set_led_state(LedState::Connected);

    if (!selected_peripheral.connect())
    {
        LOG("Failed to Connect!");
        change_app_state(AppState::Scanning);
        return;
    }

    LOG_("RSSI: ");
    LOG(selected_peripheral.rssi());

    if (selected_peripheral.discoverService("53300001-0023-4bd4-bbd5-a6920e4c5653"))
    {
        LOG("Found Proper Service!");
    }
    else
    {
        LOG("No Proper Service!");
        change_app_state(AppState::Scanning);
        return;
    }

    tx_characteristic = selected_peripheral.characteristic("53300002-0023-4bd4-bbd5-a6920e4c5653");
    if (!tx_characteristic)
    {
        LOG("no TX characteristic found!");
        change_app_state(AppState::Scanning);
        return;
    }

    rx_characteristic = selected_peripheral.characteristic("53300003-0023-4bd4-bbd5-a6920e4c5653");
    if (!rx_characteristic)
    {
        LOG("no RX characteristic found!");
        change_app_state(AppState::Scanning);
        return;
    }

    LOG("Connected!");
}

void Connected_loop()
{
    if (!selected_peripheral || !selected_peripheral.connected())
    {
        LOG("Peripheral NOT Connected!");
        change_app_state(AppState::Scanning);
        return;
    }

    static char tx_buffer[64] = "rem_b:00;";

    bool updated = false;
    if (btn1.event() == BtnEvent::Press)
    {
        tx_buffer[6] = '1';
        updated = true;
    }
    else if (btn1.event() == BtnEvent::Release)
    {
        tx_buffer[6] = '0';
        updated = true;
    }

    if (btn2.event() == BtnEvent::Press)
    {
        tx_buffer[7] = '1';
        updated = true;
    }
    else if (btn2.event() == BtnEvent::Release)
    {
        tx_buffer[7] = '0';
        updated = true;
    }

    if (updated)
    {
        tx_characteristic.writeValue(tx_buffer);
        LOG_("TX: ");
        LOG(tx_buffer);
    }
}

void Connected_exit()
{
    if (selected_peripheral)
    {
        LOG("No Longer Connected!");
        selected_peripheral.disconnect();
        // selected_peripheral = BLEDevice();
    }
}

void change_app_state(AppState new_state)
{
    if (new_state != app_state)
    {
        if (app_state == AppState::Idle)
            Idle_exit();
        else if (app_state == AppState::Scanning)
            Scanning_exit();
        else if (app_state == AppState::Connected)
            Connected_exit();

        app_state = new_state;

        if (app_state == AppState::Idle)
            Idle_enter();
        else if (app_state == AppState::Scanning)
            Scanning_enter();
        else if (app_state == AppState::Connected)
            Connected_enter();
    }
}

void loop()
{
    btn1.update();
    btn2.update();

    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable())
    {
        IMU.readAcceleration(imu_processor.ax, imu_processor.ay, imu_processor.az);
        IMU.readGyroscope(imu_processor.gx, imu_processor.gy, imu_processor.gz);
        imu_processor.update();
    }

    #ifdef DEBUG_OVER_SERIAL
        if (btn1.event() == BtnEvent::Press) { LOG_(event_count++); LOG(": press [1]"); }
        if (btn2.event() == BtnEvent::Press) { LOG_(event_count++); LOG(": press [2]"); }
    #endif

    switch (app_state)
    {
    case AppState::Idle:
        Idle_loop();
        break;

    case AppState::Scanning:
        Scanning_loop();
        break;

    case AppState::Connected:
        Connected_loop();
        break;
    };

    delay(5);
}

// IMU.readAcceleration(ax, ay, az);
// IMU.readGyroscope(gx, gy, gz);

// ax_dc = dc_alpha * ax + (1 - dc_alpha) * ax_dc;
// ay_dc = dc_alpha * ay + (1 - dc_alpha) * ay_dc;
// az_dc = dc_alpha * az + (1 - dc_alpha) * az_dc;

// ax_d = ax - ax_dc;
// ay_d = ay - ay_dc;
// az_d = az - az_dc;

// float a_d = ax_d*ax_d + ay_d*ay_d + az_d*az_d;
// float ay_ratio = ay_d*ay_d / a_d;

// if(a_d < 1) ay_ratio = 0;

// float trigger0 = 0;

// if(ay_ratio > 0.85) {
//     activation_millis = millis();
// }

// if(a_d > 10 && millis() - activation_millis < 25) {
//     trigger0 = 10;
// }

// if (n % 1 == 0 && false) {
//     Serial.print("a_d:");
//     Serial.print(a_d);
//     Serial.print(",");
//     Serial.print("ay_ratio:");
//     Serial.print(ay_ratio);
//     Serial.print(",");
//     Serial.print("trigger:");
//     Serial.print(trigger0);
//     // Serial.print(",");
//     // Serial.print(4);
//     // Serial.print(",");
//     // Serial.print(-4);
//     Serial.println("");
// }

// n++;
