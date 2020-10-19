/*!
 *  @file BMX-055.h
 *
 *  This is a library for the BMX055 Inertial Measurement Unit
 *
 *  These sensors use I2C to communicate, 2 pins are required to interface.
 *
 *  João Pedro Vilas (github.com/joaopedrovbs)
 *
 *  MIT license, all text above must be included in any redistribution
 */

#ifndef __ADAFRUIT_BMX055_H__
#define __ADAFRUIT_BMX055_H__

#include "Arduino.h"
#include <Wire.h>

// BMX055 default I2C addresses
#define BMX055_ACCEL_DEFAULT_ADDRESS  0x18
#define BMX055_GYRO_DEFAULT_ADDRESS   0x68
#define BMX055_MAG_DEFAULT_ADDRESS    0x10

/** BMX055 ID **/
#define BMX055_ACCEL_ID       0xFA
#define BMX055_GYRO_ID        0x0F
#define BMX055_MAG_ID         0x32

class BMX055 {
  public:

    /* BMX055 object, input the I2C bus and addresses */
    BMX055(TwoWire &bus, uint8_t, uint8_t, uint8_t);

    int begin(void);


    typedef enum {
      NORMAL_MODE,      //  All parts held powered-up and data acquisition is performed continuosly
      DEEP_SUSPEND,     //  Only interface Section is kept alive. No Data Acquisition is performed
      SUSPEND_MODE,     //  The whole analog part is powered down. No Data Acquisition is performed 
                        //  there are some restriction to writing and reading (wait for some time)
      STANDBY_MODE,     //  Same as suspend but without restrictions
      LOW_POWER_MODE_1, //  Device periodically switches between a sleep(suspend mode) and a wake-up phase(normal mode)
      LOW_POWER_MODE_2, //  Similar to mode 1 but register access can happen anytime without restrictions
    }BMX055_ACCEL_POWER_MODES_t;
    
    typedef enum{
      MS_0_5  =0x05,
      MS_1    =0x06,
      MS_2    =0x07,
      MS_4    =0x08,
      MS_6    =0x09,
      MS_10   =0x0A,
      MS_25   =0x0B,
      MS_50   =0x0C,
      MS_100  =0x0D,
      MS_500  =0x0E,
      MS_1000 =0x0F,
    }BMX_ACCEL_SLEEP_DUR_t;

    enum{
      FILTERED = 0,
      UNFILTERED,
    }BMX_ACCEL_DATA_STREAM_t;

    typedef enum{
      HZ_7_81   =0x08,
      HZ_15_63  =0x09,
      HZ_31_25  =0x0A,
      HZ_62_5   =0x0B,
      HZ_125    =0x0C,
      HZ_250    =0x0D,
      HZ_500    =0x0E,
      HZ_1000   =0x10,
    }BMX_ACCEL_DATA_BANDWIDTH_t;

    typedef enum{ 
      ACCEL_RANGE_2G    =0x03,
      ACCEL_RANGE_4G    =0x05,
      ACCEL_RANGE_8G    =0x08,
      ACCEL_RANGE_16G   =0x0C
    }BMX055_ACCEL_RANGE_t;

    bool setAccelRange(BMX055_ACCEL_RANGE_t);

    bool setAccelBandwidth(BMX_ACCEL_DATA_BANDWIDTH_t);

    bool setSleepPeriod(BMX_ACCEL_SLEEP_DUR_t);

    void readAccel(void);

    float getAccelX_mss();
    float getAccelY_mss();
    float getAccelZ_mss();

  private:
    /** BMX055 Registers **/
    typedef enum {
      BGW_CHIPID    =0x00,
      ACCD_X_LSB    =0x02,
      ACCD_X_MSB    =0x03,
      ACCD_Y_LSB    =0x04,
      ACCD_Y_MSB    =0x05,
      ACCD_Z_LSB    =0x06,
      ACCD_Z_MSB    =0x07,
      ACCD_TEMP     =0x08,
      INT_STATUS_0  =0x09,
      INT_STATUS_1  =0x0A,
      INT_STATUS_2  =0x0B,
      INT_STATUS_3  =0x0C,
      FIFO_STATUS   =0x0E,
      PMU_RANGE     =0x0F,
      PMU_BW        =0x10,
      PMU_LPW       =0x11,
      PMU_LOW_POWER =0x12,
      ACCD_HBW      =0x13,
      BGW_SOFTRESET =0x14,
      INT_EN_0      =0x16,
      INT_EN_1      =0x17,
      INT_EN_2      =0x18,
      INT_MAP_0     =0x19,
      INT_MAP_1     =0x1A,
      INT_MAP_2     =0x1B,
      INT_SRC       =0x1E,
      INT_OUT_CTRL  =0x20,
      INT_RST_LATCH =0x21,
      INT_0         =0x22,
      INT_1         =0x23,
      INT_2         =0x24,
      INT_3         =0x25,
      INT_4         =0x26,
      INT_5         =0x27,
      INT_6         =0x28,
      INT_7         =0x29,
      INT_8         =0x2A,
      INT_9         =0x2B,
      INT_A         =0x2C,
      INT_B         =0x2D,
      INT_C         =0x2E,
      INT_D         =0x2F,
      FIFO_CONFIG_0 =0x30,
      PMU_SELF_TEST =0x32,
      TRIM_NVM_CTRL =0x33,
      BGW_SPI3_WDT  =0x34,
      OFC_CTRL      =0x36,
      OFC_SETTING   =0x37,
      OFC_OFFSET_X  =0x38,
      OFC_OFFSET_Y  =0x39,
      OFC_OFFSET_Z  =0x3A,
      TRIM_GP0      =0x3B,
      TRIM_GP1      =0x3C,
      FIFO_CONFIG_1 =0x3E,
      FIFO_DATA     =0x3F,
    }BMX055_Accel_reg_t;
    
    // i2c
    TwoWire *_i2c;
    const uint32_t _i2cRate = 400000; // 400 kHz
    
    uint8_t _accelAddress;
    uint8_t _gyroAddress;
    uint8_t _magAddress;

    BMX055_ACCEL_RANGE_t rangeBuffer;
    
    float _ax, _ay, _az;
    float _gx, _gy, _gz;
    float _hx, _hy, _hz;

    bool writeAccelRegister(BMX055_Accel_reg_t , uint8_t );
    uint8_t readAccelRegisters(BMX055_Accel_reg_t);

    // bool writeGyroRegister(BMX055_Gyro_reg_t regAddress, uint8_t data);
    // uint8_t readGyroRegisters(BMX055_Gyro_reg_t regAddress);

    // bool writeMagRegister(BMX055_Mag_reg_t regAddress, uint8_t data);
    // uint8_t readMagRegisters(BMX055_Mag_reg_t regAddress);
};

#endif