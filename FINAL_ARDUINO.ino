#include "ZMPT101B.h"
#include "ACS712.h"

// ZMPT101B sensor connected to A0 pin of arduino
ZMPT101B voltageSensor(A1);

// 5 amps version sensor (ACS712_05B) connected to A1 pin of arduino
ACS712 currentSensor(ACS712_05B, A0);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  delay(100); //wait 1 sec before powering current
  voltageSensor.calibrate();
  currentSensor.calibrate();
}
int l;
void loop() {
  if (Serial.available() >= 21)
    if (Serial.read() == 0x7E) {
      for (int i = 1; i < 19; i++) {
        if (i > 3 && i < 12)
        {
          l += (Serial.read());
        }
        else
        {
          byte discardByte = Serial.read();
        }
      }

      int MSB = Serial.read();
      int LSB = Serial.read();
      int R = LSB + (MSB * 256);
     
      float U = (voltageSensor.getVoltageAC()) *  12.5348*(240/233.5) ;//230/18.35
      float I = currentSensor.getCurrentAC();

      // To calculate the power we need voltage multiplied by current
      float P = U * I;

      Serial.print(String("*") +l + String("*"));
      Serial.print(R);
      Serial.print(String("*") + U + "*");
      Serial.print( I +String("*"));
     //Serial.println(String("*") + P );

      l = 0;
      Serial.println();

    }
}
