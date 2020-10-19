#include "Arduino.h"
#include "BMX-055.h"

/* BMX055 object, input the I2C bus and address */
BMX055::BMX055(
  TwoWire &bus, 
  uint8_t addressAccel = BMX055_ACCEL_DEFAULT_ADDRESS, 
  uint8_t addressGyro = BMX055_GYRO_DEFAULT_ADDRESS, 
  uint8_t addressMag = BMX055_MAG_DEFAULT_ADDRESS){
  _i2c = &bus; // I2C bus
  _accelAddress = addressAccel; // I2C Accel Address
  _gyroAddress = addressGyro;   // I2C Gyro Address
  _magAddress = addressMag;     // I2C Mag Address
}

/* starts communication with the MPU-9250 */
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

    if
  // successful init, return 1
  return 1;
}

bool BMX055::setAccelRange(BMX055_ACCEL_RANGE_t range){
  if(!writeAccelRegister(PMU_RANGE, range)){
    return false;
  }
  return true;
}

bool BMX055::setAccelBandwidth(BMX_ACCEL_DATA_BANDWIDTH_t bandwidth){
  if(!writeAccelRegister(PMU_BW, bandwidth)){
    return false;
  }
  return true;
}

bool BMX055::setSleepPeriod(BMX_ACCEL_SLEEP_DUR_t duration){
  if(!writeAccelRegister(PMU_LPW, duration)){
    return false;
  }
  return true;
}

/* writes a byte to BMX055 register given a register address and data */
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

/* reads registers from BMX055 given a starting register address, number of bytes, and a pointer to store data */
uint8_t BMX055::readAccelRegisters(BMX055_Accel_reg_t regAddress){
  _i2c->beginTransmission((uint8_t)_accelAddress); // open the device
  _i2c->write(regAddress); // specify the starting register address
  _i2c->endTransmission(false);
  _i2c->requestFrom(_accelAddress, 1); // specify the number of bytes to receive
  return _i2c->read();
}

/* writes a byte to BMX055 register given a register address and data */
bool BMX055::writeGyroRegister(BMX055_reg_t regAddress, uint8_t data){
  /* write data to device */
  _i2c->beginTransmission((uint8_t)_gyroAddress); // open the device
  _i2c->write(regAddress); // write the register address
  _i2c->write(data); // write the data
  _i2c->endTransmission();

  /* check the read back register against the written register */
  if(readGyroRegisters(regAddress) == data) {
    return true;
  }
  return false;
}
/* reads registers from BMX055 given a starting register address, number of bytes, and a pointer to store data */
uint8_t BMX055::readGyroRegisters(BMX055_reg_t regAddress){
  _i2c->beginTransmission(_gyroAddress); // open the device
  _i2c->write(regAddress); // specify the starting register address
  _i2c->endTransmission(false);
  _i2c->requestFrom(_gyroAddress, 1); // specify the number of bytes to receive
  return _i2c->read();
}

/* writes a byte to BMX055 register given a register address and data */
bool BMX055::writeMagRegister(BMX055_reg_t regAddress, uint8_t data){
  /* write data to device */
  _i2c->beginTransmission((uint8_t)_magAddress); // open the device
  _i2c->write(regAddress); // write the register address
  _i2c->write(data); // write the data
  _i2c->endTransmission();

  /* check the read back register against the written register */
  if(readMagRegisters(regAddress) == data) {
    return true;
  }
  return false;
}

/* reads registers from BMX055 given a starting register address, number of bytes, and a pointer to store data */
uint8_t BMX055::readMagRegisters(BMX055_reg_t regAddress){
  _i2c->beginTransmission(_magAddress); // open the device
  _i2c->write(regAddress); // specify the starting register address
  _i2c->endTransmission(false);
  _i2c->requestFrom(_magAddress, 1); // specify the number of bytes to receive
  return _i2c->read();
}

