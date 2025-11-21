#include <Wire.h>
#include <MPU9250_asukiaaa.h>

MPU9250_asukiaaa mySensor;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  mySensor.setWire(&Wire);
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();

}

void loop() {
  mySensor.accelUpdate();
  mySensor.gyroUpdate();
  mySensor.magUpdate();

  Serial.print("ACCEL:");
  Serial.print(mySensor.accelX());
  Serial.print(","); 
  Serial.print(mySensor.accelY());
  Serial.print(","); 
  Serial.print(mySensor.accelZ()); 
  Serial.print(";GYRO:"); 
  Serial.print(mySensor.gyroX()); 
  Serial.print(","); 
  Serial.print(mySensor.gyroY()); 
  Serial.print(","); 
  Serial.print(mySensor.gyroZ()); 
  Serial.print(";MAG:"); 
  Serial.print(mySensor.magX()); 
  Serial.print(","); 
  Serial.print(mySensor.magY()); 
  Serial.print(","); 
  Serial.println(mySensor.magZ()); 
  delay(100);
}
