#ifndef _BNO055_H_
#define _BNO055_H_

#include "I2C_Device.h"
#include "BNO055_REG.h"

#define COEF_UNIT_MPS2 100.0f
#define COEF_UNIT_MG   1.0f
#define COEF_UNIT_uT   16.0f
#define COEF_UNIT_DPS  16.0f
#define COEF_UNIT_RPS  900.0f
#define COEF_UNIT_DEG  16.0f
#define COEF_UNIT_RAD  900.0f
#define COEF_UNIT_LESS 16384.0f

typedef struct {
  float X, Y, Z;
} Axis_3D_t;

typedef struct {
  float W, X, Y, Z;
} Axis_4D_t;

typedef struct {
  Axis_3D_t ACC;
  Axis_3D_t MAG;
  Axis_3D_t GYR;
  Axis_3D_t EUL;
  Axis_4D_t QUA;
  Axis_3D_t LIA;
  Axis_3D_t GRV;
  int8_t TEMP;
} BNO055_t;

typedef class  {
  public:
  uint8_t setMode(uint8_t reg);
  void setReset(uint8_t reg);
  void setPower(uint8_t reg);
  void setUnitSel(uint8_t sys, uint8_t temp, uint8_t eul, uint8_t gyr, uint8_t acc);
  void setAccConfig(uint8_t pwr, uint8_t bw, uint8_t range);
  void setMagConfig(uint8_t pwr, uint8_t opr, uint8_t rate);
  void setGyrConfig(uint8_t pwr, uint8_t bw, uint8_t range);
  void setAxisRemap(uint8_t remap);
  uint8_t setAccOffsets(int16_t *Data);
  uint8_t setMagOffsets(int16_t *Data);
  uint8_t setGyrOffsets(int16_t *Data);
  void setAccRadiusOffsets(int16_t *Data);
  void setMagRadiusOffsets(int16_t *Data);
  bool Init(TwoWire *_i2c, uint8_t _add);
  
  uint8_t getCalibStat(uint8_t reg);
  uint8_t getSTStat(uint8_t reg);
  uint8_t getSysStat();
  void getAccOffsets(int16_t *Data);
  void getMagOffsets(int16_t *Data);
  void getGyrOffsets(int16_t *Data);
  void getAccRadiusOffsets(int16_t *Data);
  void getMagRadiusOffsets(int16_t *Data);
  void getAccCalibration(int16_t *acc_offs);
  void getMagCalibration(uint8_t remap, int16_t *mag_offs);
  void getGyrCalibration(int16_t *gyr_offs);

  bool getAccRaw(int16_t *Acc);
  bool getMagRaw(int16_t *Mag);
  bool getGyrRaw(int16_t *Gyr);
  bool getAccelaration(float *Acc);
  bool getMagnetometer(float *Mag);
  bool getGyroscope(float *Gyr);
  bool getEuler(float *Eul);
  bool getQuaternion(float *Qua);
  bool getAccLinear(float *Lia);
  bool getGravity(float *Grv);
  bool getTemperature(int8_t *Temp);  
  bool getAllData(BNO055_t *AHRS);
  
  private:
  I2C_Device I2C;
  float COEF_UNIT_ACC = COEF_UNIT_MPS2;
  float COEF_UNIT_GYR = COEF_UNIT_DPS;
  float COEF_UNIT_EUL = COEF_UNIT_DEG;
  uint16_t COEF_REMAP[8] = {((0x21 << 8) | 0x04), ((0x24 << 8) | 0x00), 
                            ((0x24 << 8) | 0x06), ((0x21 << 8) | 0x02), 
                            ((0x24 << 8) | 0x03), ((0x21 << 8) | 0x01), 
                            ((0x21 << 8) | 0x07), ((0x24 << 8) | 0x05)};
} BNO_Sensor;

extern BNO_Sensor BNO;

#endif /*_BNO050_H_*/