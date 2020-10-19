#include "BMX-055.h"

BMX055 bmx(Wire,BMX055_ACCEL_DEFAULT_ADDRESS,BMX055_GYRO_DEFAULT_ADDRESS,BMX055_MAG_DEFAULT_ADDRESS);

void setup() {
  
  Serial.begin(115200);

  Serial.println("BMX055 Initialization: ");
  
  int status = bmx.begin();
  if(status != 1){
    Serial.print("Error code: "); Serial.println(status);
  }

  Serial.println("BMP Init OK");
}

void loop() {
  // put your main code here, to run repeatedly:
  bmx.readAccel();

  Serial.print("Accel: ");
  Serial.print("X ");
  Serial.print(getAccelX_mss());
  Serial.print(" Y ");
  Serial.print(getAccelY_mss());
  Serial.print(" Z ");
  Serial.println(getAccelZ_mss());
  
  delay(300);
}
