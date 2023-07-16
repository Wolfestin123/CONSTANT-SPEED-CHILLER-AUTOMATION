#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 8  //D8 of nano

OneWire oneWire(ONE_WIRE_BUS);

DallasTemperature sensors(&oneWire);

float Celsius = 0;
float Fahrenheit = 0;

void setup() {
  sensors.begin();
  Serial.begin(9600);
   pinMode(3, OUTPUT);//D3 as PWM
}

void loop() {
  sensors.requestTemperatures();

  Celsius = sensors.getTempCByIndex(0);
  
  float t=(Celsius/100)*255;//100 or 128 division factor
  analogWrite(3,t );//xbee to d3 
  
  Serial.print(t + String(" - "));
  Serial.print(Celsius);
  Serial.println(" C  ");
  delay(1000);
}
