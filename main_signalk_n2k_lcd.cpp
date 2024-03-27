#include <Arduino.h>
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/sensors/system_info.h"
#include "sensesp_app_builder.h"

// CAN bus (NMEA 2000) pins on SH-ESP32
#include <N2kMessages.h>
#include <NMEA2000_esp32.h>
#define CAN_RX_PIN GPIO_NUM_34
#define CAN_TX_PIN GPIO_NUM_32
#define RECOVERY_RETRY_MS 1000  // How long to attempt CAN bus recovery
#define MAX_RX_WAIT_TIME_MS 30000  // Time after which we should reboot if we haven't received any CAN messages

// 1-Wire data pin on SH-ESP32
// #define ONEWIRE_PIN 4
// #include "sensesp_onewire/onewire_temperature.h"

// MAX6675
#include <max6675.h>
#define thermoDO GPIO_NUM_18
#define thermoCLK GPIO_NUM_25
#define thermo1CS GPIO_NUM_26
#define thermo2CS GPIO_NUM_27

// Display
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>  // i2c
#define SDA_PIN 16
#define SCL_PIN 17
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels

using namespace sensesp;

tNMEA2000* nmea2000;

// NEED TO UPDATE FOR EXHAUST TEMP PGN
// void SendEngineTemperatures() {
//   tN2kMsg N2kMsg;
//   SetN2kEngineDynamicParam(N2kMsg,
//                            0,  // instance of a single engine is always 0
//                            N2kDoubleNA,  // oil pressure
//                            oil_temperature, coolant_temperature,
//                            N2kDoubleNA,  // alternator voltage
//                            N2kDoubleNA,  // fuel rate
//                            N2kDoubleNA,  // engine hours
//                            N2kDoubleNA,  // engine coolant pressure
//                            N2kDoubleNA,  // engine fuel pressure
//                            N2kInt8NA,    // engine load
//                            N2kInt8NA,    // engine torque
//                            (tN2kEngineDiscreteStatus1)0,
//                            (tN2kEngineDiscreteStatus2)0);
//   nmea2000->SendMsg(N2kMsg);
// }

ReactESP app;

// MAX6675 setup
MAX6675 thermocouple0(thermoCLK, thermo1CS, thermoDO);
MAX6675 thermocouple1(thermoCLK, thermo2CS, thermoDO);

float temp0 = 0;
float temp1 = 0;

float temp0_callback() {
  temp0 = thermocouple0.readCelsius() + 273.15;
  return (temp0);
}

float temp1_callback() {
  temp1 = thermocouple1.readCelsius() + 273.15;
  return (temp1);
}

// Display setup
Adafruit_SSD1306 *display;
TwoWire *i2c;

// CANbus setup
String can_state;

void RecoverFromCANBusOff() {
  // This recovery routine first discussed in
  // https://www.esp32.com/viewtopic.php?t=5010 and also implemented in
  // https://github.com/wellenvogel/esp32-nmea2000
  static bool recovery_in_progress = false;
  static elapsedMillis recovery_timer;
  if (recovery_in_progress && recovery_timer < RECOVERY_RETRY_MS) {
    return;
  }
  recovery_in_progress = true;
  recovery_timer = 0;
  // Abort transmission
  MODULE_CAN->CMR.B.AT = 1;
  // read SR after write to CMR to settle register changes
  (void)MODULE_CAN->SR.U;

  // Reset error counters
  MODULE_CAN->TXERR.U = 127;
  MODULE_CAN->RXERR.U = 0;
  // Release Reset mode
  MODULE_CAN->MOD.B.RM = 0;
}

// CAN controller registers are SJA1000 compatible.
// Bus status value 0 indicates bus-on; value 1 indicates bus-off.
void PollCANStatus() {
  unsigned int bus_status = MODULE_CAN->SR.B.BS;

  switch (bus_status) {
    case 0:
      can_state = "RUNNING";
      break;
    case 1:
      can_state = "BUS-OFF";
      // try to automatically recover
      RecoverFromCANBusOff();
      break;
  }
}

void setup() {
#ifndef SERIAL_DEBUG_DISABLED
  SetupSerialDebug(115200);
#endif

  SensESPAppBuilder builder;

  sensesp_app = (&builder)
                  // Set a custom hostname for the app.
                  ->set_hostname("egt-temp")
                  // Optionally, hard-code the WiFi and Signal K server
                  // settings. This is normally not needed.
                  ->set_wifi("Off Hand 2.4G", "2222222222")
                  //->set_wifi("kitty3", "2222222222")
                  //->set_sk_server("192.168.8.10", 3443)
                  ->get_app();

  // DallasTemperatureSensors* dts = new DallasTemperatureSensors(ONEWIRE_PIN);

  // define three 1-Wire temperature sensors that update every 1000 ms
  // and have specific web UI configuration paths
  // auto engine_0_egt_temperature =
  //     new OneWireTemperature(dts, 1000, "/Engine0ExhaustTemp/oneWire");

  // auto engine_1_egt_temperature =
  //     new OneWireTemperature(dts, 1000, "/Engine1ExhaustTemp/oneWire");

  auto* engine_0_egt_temperature = new RepeatSensor<float>(1000, temp0_callback); 
  auto* engine_1_egt_temperature = new RepeatSensor<float>(1000, temp1_callback);

  // define metadata for sensors
  auto engine_egt_temperature_metadata =
      new SKMetadata("K",                        // units
                     "Exhaust Gas Temperature",  // display name
                     "Exhaust Gas Temperature",  // description
                     "EGT",                      // short name
                     10.                         // timeout, in seconds
      );

  // connect the sensors to Signal K output paths
  engine_0_egt_temperature->connect_to(
      new SKOutput<float>("propulsion.0.exhaustTemperature",
                          engine_egt_temperature_metadata));

  engine_1_egt_temperature->connect_to(
      new SKOutput<float>("propulsion.1.exhaustTemperature",
                          engine_egt_temperature_metadata));

  // temp0->connect_to(new SKOutputFloat("propulsion.0.exhaustTemperature"));
  // temp1->connect_to(new SKOutputFloat("propulsion.1.exhaustTemperature"));

  // initialize the NMEA 2000 subsystem
  // instantiate the NMEA2000 object
  nmea2000 = new tNMEA2000_esp32(CAN_TX_PIN, CAN_RX_PIN);

  // Reserve enough buffer for sending all messages. This does not work on small
  // memory devices like Uno or Mega
  nmea2000->SetN2kCANSendFrameBufSize(250);
  nmea2000->SetN2kCANReceiveFrameBufSize(250);

  // Set Product information
  nmea2000->SetProductInformation(
      "7630101",  // Manufacturer's Model serial code (max 32 chars)
      103,         // Manufacturer's product code
      "SME EGT Interface",  // Manufacturer's Model ID (max 33 chars)
      "0.1 (2024-03-22)",  // Manufacturer's Software version code (max 40
                               // chars)
      "0.1 (2024-03-22)"   // Manufacturer's Model version (max 24 chars)
  );

  // Set device information
  nmea2000->SetDeviceInformation(
      7630101.01,    // Unique number. Use e.g. Serial number.
      160,  // Device function=Engine Gateway
      50,   // Device class=Propulsion
            // https://manualzz.com/doc/12647142/nmea2000-class-and-function-codes
      2012  // Manufacture code, free number taken from
            // https://ttlappalainen.github.io/NMEA2000/md_7_glossary.html#secRefMfgCodes
  );

  nmea2000->SetMode(tNMEA2000::N2km_NodeOnly, 22);
  // Disable all msg forwarding to USB (=Serial)
  nmea2000->EnableForward(false);
  nmea2000->Open();

  // No need to parse the messages at every single loop iteration; 1 ms will do
  app.onRepeat(1, []() {
    PollCANStatus();
    nmea2000->ParseMessages();
  });

  // Implement the N2K PGN sending.

  // hijack the exhaust gas temperature for wet exhaust temperature
  // measurement
  engine_0_egt_temperature->connect_to(
      new LambdaConsumer<float>([](float temperature) {
        tN2kMsg N2kMsg;
        SetN2kTemperature(N2kMsg,
                          1,                            // SID
                          0,                            // TempInstance
                          N2kts_ExhaustGasTemperature,  // TempSource
                          temperature                   // actual temperature
        );
        nmea2000->SendMsg(N2kMsg);
      }));

  engine_1_egt_temperature->connect_to(
      new LambdaConsumer<float>([](float temperature) {
        tN2kMsg N2kMsg;
        SetN2kTemperature(N2kMsg,
                          1,                            // SID
                          1,                            // TempInstance
                          N2kts_ExhaustGasTemperature,  // TempSource
                          temperature                   // actual temperature
        );
        nmea2000->SendMsg(N2kMsg);
      }));


  // initialize the display
  i2c = new TwoWire(0);
  i2c->begin(SDA_PIN, SCL_PIN);
  display = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, i2c, -1);
  if (!display->begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  delay(100);
  display->setRotation(2);
  display->clearDisplay();
  display->display();

  // update results on display
  app.onRepeat(1000, [&]() {
    display->clearDisplay();
    display->setTextSize(2);
    display->setCursor(0, 0);
    display->setTextColor(SSD1306_WHITE);
    display->printf("SME EGT\n");
    display->setTextSize(1);
    display->printf("CAN: %s\n", can_state.c_str());
    display->printf("SSID: %s\n", WiFi.SSID().c_str());
    display->printf("IP: %s\n", WiFi.localIP().toString().c_str());
    display->printf("Temperature 0: %.0f F\n", thermocouple0.readFahrenheit());
    display->printf("Temperature 1: %.0f F\n", thermocouple1.readFahrenheit());
    display->display();
  });

  sensesp_app->start();
}

// main program loop
void loop() { app.tick(); }