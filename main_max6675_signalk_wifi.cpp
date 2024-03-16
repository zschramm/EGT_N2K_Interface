#include <Arduino.h>
#include <max6675.h>

#include "sensesp/signalk/signalk_output.h"
#include "sensesp_app.h"
#include "sensesp_app_builder.h"

using namespace sensesp;

reactesp::ReactESP app;

int thermoDO = GPIO_NUM_25;
int thermoCLK = GPIO_NUM_26;
int thermo1CS = GPIO_NUM_32;
int thermo2CS = GPIO_NUM_33;

MAX6675 thermocouple0(thermoCLK, thermo1CS, thermoDO);
MAX6675 thermocouple1(thermoCLK, thermo2CS, thermoDO);

float temp0 = 0;
float temp1 = 0;

float temp0_callback() {
  temp0 = thermocouple0.readCelsius() + 273;
  return (temp0);
}

float temp1_callback() {
  temp1 = thermocouple1.readCelsius() + 273;
  return (temp1);
}

void setup() {
#ifndef SERIAL_DEBUG_DISABLED
  SetupSerialDebug(115200);
#endif

  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("egt-temp")
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    ->set_wifi("kitty3", "2222222222")
                    //->set_sk_server("192.168.10.3", 80)
                    ->get_app();

  auto* temp0 = new RepeatSensor<float>(1000, temp0_callback); 
  auto* temp1 = new RepeatSensor<float>(1000, temp1_callback);

  temp0->connect_to(new SKOutputFloat("propulsion.0.exhaustemperature"));
  temp1->connect_to(new SKOutputFloat("propulsion.1.exhaustTemperature"));

  // Start networking, SK server connections and other SensESP internals
  sensesp_app->start();
}

void loop() { app.tick(); }