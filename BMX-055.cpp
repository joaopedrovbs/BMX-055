#include "Arduino.h"
#include "BMX-055.h"

/* BMX055 object, input the I2C bus and address */
BMX055::BMX055(
  TwoWire &bus, 
  uint8_t addressAccel, 
  uint8_t addressGyro, 
  uint8_t addressMag){
  _i2c = &bus; // I2C bus
  _accelAddress = addressAccel; // I2C Accel Address
  _gyroAddress = addressGyro;   // I2C Gyro Address
  _magAddress = addressMag;     // I2C Mag Address
}

/* starts communication with the BMX055 */
int BMX055::begin(){
    // starting the I2C bus
    _i2c->begin();
    // setting the I2C clock
    _i2c->setClock(_i2cRate);

    if(!(readAccelRegisters(BGW_CHIPID) == BMX055_ACCEL_ID)){
      return -1;
    }

    if(!setAccelRange(ACCEL_RANGE_8G)){
      return -2;
    }

    if(!setSleepPeriod(MS_1)){
      return -3;
    }
  // successful init, return 1
  return 1;
}

// Sets the range for accelerometer measurements
bool BMX055::setAccelRange(BMX055_ACCEL_RANGE_t range){
  rangeBuffer = range;
  if(!writeAccelRegister(PMU_RANGE, range)){
    return false;
  }
  return true;
}

// Sets the Bandwidth for the Low Pass Filter
bool BMX055::setAccelBandwidth(BMX_ACCEL_DATA_BANDWIDTH_t bandwidth){
  if(!writeAccelRegister(PMU_BW, bandwidth)){
    return false;
  }
  return true;
}

// Sets the duration of the sleep period during 
bool BMX055::setSleepPeriod(BMX_ACCEL_SLEEP_DUR_t duration){
  if(!writeAccelRegister(PMU_LPW, duration)){
    return false;
  }
  return true;
}

// Performs accelerometer read and conversion
void BMX055::readAccel(void){
  // Buffers for raw 8-bit data
  uint8_t xRawData[2];
  uint8_t yRawData[2];
  uint8_t zRawData[2];
  
  // Reading registers
  xRawData[0] = readAccelRegisters(ACCD_X_LSB);
  xRawData[1] = readAccelRegisters(ACCD_X_MSB);
  
   // Reading registers
  yRawData[0] = readAccelRegisters(ACCD_Y_LSB);
  yRawData[1] = readAccelRegisters(ACCD_Y_MSB);

   // Reading registers
  zRawData[0] = readAccelRegisters(ACCD_Z_LSB);
  zRawData[1] = readAccelRegisters(ACCD_Z_MSB);

  // Converting registers to 12 bit variables
  int16_t accelX12Bit = (( (xRawData[1] << 8) | (xRawData[0] & 0xF0) ) >> 4);
  int16_t accelY12Bit = (( (yRawData[1] << 8) | (yRawData[0] & 0xF0) ) >> 4);
  int16_t accelZ12Bit = (( (zRawData[1] << 8) | (zRawData[0] & 0xF0) ) >> 4);

  // Convert data into a + and - range
  if (accelX12Bit > 2047)
  {
    accelX12Bit -= 4096;
  }
   // Convert data into a + and - range
  if (accelY12Bit > 2047)
  {
    accelY12Bit -= 4096;
  }
   // Convert data into a + and - range
  if (accelZ12Bit > 2047)
  {
    accelZ12Bit -= 4096;
  }

  uint16_t divider = 1;

  // Sensitivity Scale from the datasheet
  if (rangeBuffer == ACCEL_RANGE_2G)
    divider = 1024;
  if (rangeBuffer == ACCEL_RANGE_4G)
    divider = 512;
  if (rangeBuffer == ACCEL_RANGE_8G)
    divider = 256;
  if (rangeBuffer == ACCEL_RANGE_16G)
    divider = 128;

  // Save to local values
  _ax = (float)accelX12Bit/divider;
  _ay = (float)accelY12Bit/divider;
  _az = (float)accelZ12Bit/divider;

}

// returns the accelerometer measurement in the x direction, m/s^2 
float BMX055::getAccelX_mss(void){
  return _ax*G;
}
// returns the accelerometer measurement in the y direction, m/s^2 
float BMX055::getAccelY_mss(void){
  return _ay*G;
}
// returns the accelerometer measurement in the z direction, m/s^2 
float BMX055::getAccelZ_mss(void){
  return _az*G;
}

/* writes a byte to BMX055 Accelerometer register given a register address and data */
bool BMX055::writeAccelRegister(BMX055_Accel_reg_t regAddress, uint8_t data){
  /* write data to device */
  _i2c->beginTransmission((uint8_t)_accelAddress); // open the device
  _i2c->write(regAddress); // write the register address
  _i2c->write(data); // write the data
  _i2c->endTransmission();

  /* check the read back register against the written register */
  if(readAccelRegisters(regAddress) == data) {
    return true;
  }
  return false;
}

/* reads registers from BMX055 Accelerometer given a starting register address, number of bytes, and a pointer to store data */
uint8_t BMX055::readAccelRegisters(BMX055_Accel_reg_t regAddress){
  _i2c->beginTransmission((uint8_t)_accelAddress); // open the device
  _i2c->write(regAddress); // specify the starting register address
  _i2c->endTransmission(false);
  _i2c->requestFrom(_accelAddress, (uint8_t)1); // specify the number of bytes to receive
  return _i2c->read();
}

// /* writes a byte to BMX055 register given a register address and data */
// bool BMX055::writeGyroRegister(BMX055_reg_t regAddress, uint8_t data){
//   /* write data to device */
//   _i2c->beginTransmission((uint8_t)_gyroAddress); // open the device
//   _i2c->write(regAddress); // write the register address
//   _i2c->write(data); // write the data
//   _i2c->endTransmission();

//   /* check the read back register against the written register */
//   if(readGyroRegisters(regAddress) == data) {
//     return true;
//   }
//   return false;
// }
// /* reads registers from BMX055 given a starting register address, number of bytes, and a pointer to store data */
// uint8_t BMX055::readGyroRegisters(BMX055_reg_t regAddress){
//   _i2c->beginTransmission(_gyroAddress); // open the device
//   _i2c->write(regAddress); // specify the starting register address
//   _i2c->endTransmission(false);
//   _i2c->requestFrom(_gyroAddress, 1); // specify the number of bytes to receive
//   return _i2c->read();
// }

// /* writes a byte to BMX055 register given a register address and data */
// bool BMX055::writeMagRegister(BMX055_reg_t regAddress, uint8_t data){
//   /* write data to device */
//   _i2c->beginTransmission((uint8_t)_magAddress); // open the device
//   _i2c->write(regAddress); // write the register address
//   _i2c->write(data); // write the data
//   _i2c->endTransmission();

//   /* check the read back register against the written register */
//   if(readMagRegisters(regAddress) == data) {
//     return true;
//   }
//   return false;
// }

// /* reads registers from BMX055 given a starting register address, number of bytes, and a pointer to store data */
// uint8_t BMX055::readMagRegisters(BMX055_reg_t regAddress){
//   _i2c->beginTransmission(_magAddress); // open the device
//   _i2c->write(regAddress); // specify the starting register address
//   _i2c->endTransmission(false);
//   _i2c->requestFrom(_magAddress, 1); // specify the number of bytes to receive
//   return _i2c->read();
// }

