#include <N2kMessages.h>
#include <NMEA2000_esp32.h>

#include "sensesp/signalk/signalk_output.h"
#include "sensesp_app_builder.h"
#include "sensesp_onewire/onewire_temperature.h"

// 1-Wire data pin on SH-ESP32
#define ONEWIRE_PIN 4

// CAN bus (NMEA 2000) pins on SH-ESP32
#define CAN_RX_PIN GPIO_NUM_34
#define CAN_TX_PIN GPIO_NUM_32

using namespace sensesp;

tNMEA2000* nmea2000;

/**
 * @brief Send Engine Dynamic Parameter data
 *
 * Send engine temperature data using the Engine Dynamic Parameter PGN.
 * All unused fields that are sent with undefined value except the status
 * bit fields are sent as zero. Hopefully we're not resetting anybody's engine
 * warnings...
 */

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

void setup() {
#ifndef SERIAL_DEBUG_DISABLED
  SetupSerialDebug(115200);
#endif

  SensESPAppBuilder builder;

  sensesp_app = builder.set_hostname("temperatures")->get_app();

  DallasTemperatureSensors* dts = new DallasTemperatureSensors(ONEWIRE_PIN);

  // define three 1-Wire temperature sensors that update every 1000 ms
  // and have specific web UI configuration paths
  auto engine_0_egt_temperature =
      new OneWireTemperature(dts, 1000, "/Engine0ExhaustTemp/oneWire");

  auto engine_1_egt_temperature =
      new OneWireTemperature(dts, 1000, "/Engine1ExhaustTemp/oneWire");

  // define metadata for sensors
  auto engine_egt_temperature_metadata =
      new SKMetadata("K",                        // units
                     "Exhaust Gas Temperature",  // display name
                     "Exhaust Gas Temperature",  // description
                     "Exhaust Gas Temperature",  // short name
                     10.                         // timeout, in seconds
      );

  // connect the sensors to Signal K output paths
  engine_0_egt_temperature->connect_to(
      new SKOutput<float>("propulsion.0.exhaustTemperature",
                          "/Engine0ExhaustTemp/skPath",
                          engine_egt_temperature_metadata));

  engine_1_egt_temperature->connect_to(
      new SKOutput<float>("propulsion.1.exhaustTemperature",
                          "/Engine1ExhaustTemp/skPath",
                          engine_egt_temperature_metadata));

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
  app.onRepeat(1, []() { nmea2000->ParseMessages(); });

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

  sensesp_app->start();
}

// main program loop
void loop() { app.tick(); }