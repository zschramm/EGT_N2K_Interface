

using namespace sensesp;



// SensESP builds upon the ReactESP framework. Every ReactESP application
// must instantiate the "app" object.
ReactESP app;




int vccPin = 3;
int gndPin = 2;


#include <Arduino.h>
#include <max6675.h>
#include <N2kMessages.h>
#include <NMEA2000_esp32.h>

#include "sensesp/signalk/signalk_output.h"
#include "sensesp_app.h"
#include "sensesp_app_builder.h"

// CAN bus (NMEA 2000) pins on SH-ESP32
#define CAN_RX_PIN GPIO_NUM_34
#define CAN_TX_PIN GPIO_NUM_32

// MAX6675 PINS
#define thermoDO_PIN = 4;
#define thermoCLK_PIN = 4;
#define thermoCS1_PIN = 4;
#define thermoCS2_PIN = 4;



// define temperature display units
#define TEMP_DISPLAY_FUNC KelvinToCelsius
//#define TEMP_DISPLAY_FUNC KelvinToFahrenheit

using namespace sensesp;

MAX6675 thermocouple1(thermoCLK, thermoCS1, thermoDO);
MAX6675 thermocouple2(thermoCLK, thermoCS2, thermoDO);

tNMEA2000* nmea2000;

float KelvinToCelsius(float temp) { return temp - 273.15; }

float KelvinToFahrenheit(float temp) { return (temp - 273.15) * 9. / 5. + 32.; }

double thermo1 = N2kDoubleNA;
double thermo2 = N2kDoubleNA;

//void SetN2kPGN130316() {
//  tN2kMsg & 	N2kMsg,
// unsigned char 	SID,
// unsigned char 	TempInstance,
// tN2kTempSource 	N2kts_ExhaustGasTemperature,
// double 	ActualTemperature,
// double 	SetTemperature = N2kDoubleNA 
// )
// }

ReactESP app;

void setup() {
#ifndef SERIAL_DEBUG_DISABLED
  SetupSerialDebug(115200);
#endif

  SensESPAppBuilder builder;

  sensesp_app = builder.set_hostname("temperatures")->get_app();

  


  DallasTemperatureSensors* dts = new DallasTemperatureSensors(ONEWIRE_PIN);

  // define three 1-Wire temperature sensors that update every 1000 ms
  // and have specific web UI configuration paths

  auto main_engine_oil_temperature =
      new OneWireTemperature(dts, 1000, "/mainEngineOilTemp/oneWire");
  auto main_engine_coolant_temperature =
      new OneWireTemperature(dts, 1000, "/mainEngineCoolantTemp/oneWire");
  auto main_engine_exhaust_temperature =
      new OneWireTemperature(dts, 1000, "/mainEngineWetExhaustTemp/oneWire");

  // define metadata for sensors

  auto main_engine_oil_temperature_metadata =
      new SKMetadata("K",                       // units
                     "Engine Oil Temperature",  // display name
                     "Engine Oil Temperature",  // description
                     "Oil Temperature",         // short name
                     10.                        // timeout, in seconds
      );
  auto main_engine_coolant_temperature_metadata =
      new SKMetadata("K",                           // units
                     "Engine Coolant Temperature",  // display name
                     "Engine Coolant Temperature",  // description
                     "Coolant Temperature",         // short name
                     10.                            // timeout, in seconds
      );
  auto main_engine_temperature_metadata =
      new SKMetadata("K",                   // units
                     "Engine Temperature",  // display name
                     "Engine Temperature",  // description
                     "Temperature",         // short name
                     10.                    // timeout, in seconds
      );
  auto main_engine_exhaust_temperature_metadata =
      new SKMetadata("K",                        // units
                     "Wet Exhaust Temperature",  // display name
                     "Wet Exhaust Temperature",  // description
                     "Exhaust Temperature",      // short name
                     10.                         // timeout, in seconds
      );

  // connect the sensors to Signal K output paths

  main_engine_oil_temperature->connect_to(new SKOutput<float>(
      "propulsion.main.oilTemperature", "/mainEngineOilTemp/skPath",
      main_engine_oil_temperature_metadata));
  main_engine_coolant_temperature->connect_to(new SKOutput<float>(
      "propulsion.main.coolantTemperature", "/mainEngineCoolantTemp/skPath",
      main_engine_coolant_temperature_metadata));
  // transmit coolant temperature as overall engine temperature as well
  main_engine_coolant_temperature->connect_to(new SKOutput<float>(
      "propulsion.main.temperature", "/mainEngineTemp/skPath",
      main_engine_temperature_metadata));
  // propulsion.*.wetExhaustTemperature is a non-standard path
  main_engine_exhaust_temperature->connect_to(
      new SKOutput<float>("propulsion.main.wetExhaustTemperature",
                          "/mainEngineWetExhaustTemp/skPath",
                          main_engine_exhaust_temperature_metadata));

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
  display->setTextSize(1);
  display->setTextColor(SSD1306_WHITE);
  display->setCursor(0, 0);
  display->printf("Host: %s", sensesp_app->get_hostname().c_str());
  display->display();

  // Add display updaters for temperature values
  main_engine_oil_temperature->connect_to(new LambdaConsumer<float>(
      [](float temperature) { PrintTemperature(1, "Oil", temperature); }));
  main_engine_coolant_temperature->connect_to(new LambdaConsumer<float>(
      [](float temperature) { PrintTemperature(2, "Coolant", temperature); }));
  main_engine_exhaust_temperature->connect_to(new LambdaConsumer<float>(
      [](float temperature) { PrintTemperature(3, "Exhaust", temperature); }));

  // initialize the NMEA 2000 subsystem

  // instantiate the NMEA2000 object
  nmea2000 = new tNMEA2000_esp32(CAN_TX_PIN, CAN_RX_PIN);

  // Reserve enough buffer for sending all messages. This does not work on small
  // memory devices like Uno or Mega
  nmea2000->SetN2kCANSendFrameBufSize(250);
  nmea2000->SetN2kCANReceiveFrameBufSize(250);

  // Set Product information
  nmea2000->SetProductInformation(
      "20210405",  // Manufacturer's Model serial code (max 32 chars)
      103,         // Manufacturer's product code
      "SH-ESP32 Temp Sensor",  // Manufacturer's Model ID (max 33 chars)
      "0.1.0.0 (2021-04-05)",  // Manufacturer's Software version code (max 40
                               // chars)
      "0.0.3.1 (2021-03-07)"   // Manufacturer's Model version (max 24 chars)
  );
  // Set device information
  nmea2000->SetDeviceInformation(
      1,    // Unique number. Use e.g. Serial number.
      130,  // Device function=Analog to NMEA 2000 Gateway. See codes on
            // http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
      75,   // Device class=Inter/Intranetwork Device. See codes on
           // http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
      2046  // Just choosen free from code list on
            // http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
  );

  nmea2000->SetMode(tNMEA2000::N2km_NodeOnly, 22);
  // Disable all msg forwarding to USB (=Serial)
  nmea2000->EnableForward(false);
  nmea2000->Open();

  // No need to parse the messages at every single loop iteration; 1 ms will do
  app.onRepeat(1, []() { nmea2000->ParseMessages(); });

  // Implement the N2K PGN sending. Engine (oil) temperature and coolant
  // temperature are a bit more complex because they're sent together
  // as part of a Engine Dynamic Parameter PGN.

  main_engine_oil_temperature->connect_to(
      new LambdaConsumer<float>([](float temperature) {
        oil_temperature = temperature;
        SendEngineTemperatures();
      }));
  main_engine_coolant_temperature->connect_to(
      new LambdaConsumer<float>([](float temperature) {
        coolant_temperature = temperature;
        SendEngineTemperatures();
      }));
  // hijack the exhaust gas temperature for wet exhaust temperature
  // measurement
  main_engine_exhaust_temperature->connect_to(
      new LambdaConsumer<float>([](float temperature) {
        tN2kMsg N2kMsg;
        SetN2kTemperature(N2kMsg,
                          1,                            // SID
                          2,                            // TempInstance
                          N2kts_ExhaustGasTemperature,  // TempSource
                          temperature                   // actual temperature
        );
        nmea2000->SendMsg(N2kMsg);
      }));

  sensesp_app->start();
}

// main program loop
void loop() { app.tick(); }