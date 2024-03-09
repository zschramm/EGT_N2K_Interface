#include <Arduino.h>
#include <max6675.h>

#include "sensesp/signalk/signalk_output.h"
#include "sensesp/transforms/linear.h"
#include "sensesp_app.h"
#include "sensesp_app_builder.h"

using namespace sensesp;

// SensESP builds upon the ReactESP framework. Every ReactESP application
// must instantiate the "app" object.
reactesp::ReactESP app;

int thermoDO = 4;
int thermoCS1 = 5;
int thermoCS2 = 7;
int thermoCLK = 6;

MAX6675 thermocouple1(thermoCLK, thermoCS1, thermoDO);
MAX6675 thermocouple2(thermoCLK, thermoCS2, thermoDO);
int vccPin = 3;
int gndPin = 2;

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}