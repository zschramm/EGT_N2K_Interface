#include <Arduino.h>
#include <max6675.h>

int thermoDO = GPIO_NUM_25;
int thermoCLK = GPIO_NUM_26;
int thermo1CS = GPIO_NUM_32;
int thermo2CS = GPIO_NUM_33;

MAX6675 thermocouple1(thermoCLK, thermo1CS, thermoDO);
MAX6675 thermocouple2(thermoCLK, thermo2CS, thermoDO);

int vccPin = 3;
int gndPin = 2;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  delay(1000);

}

void loop() {
  Serial.print("Temperature 1 (F): ");
  Serial.println(thermocouple1.readFahrenheit());
  Serial.print("Temperature 2 (F): ");
  Serial.println(thermocouple2.readFahrenheit());
  delay(1000);
}