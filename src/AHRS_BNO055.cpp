#include "AHRS_BNO055.h"

/* Set Mode BN0
Mode 0  : CFG
Mode 1  : ACC
Mode 2  : MAG
Mode 3  : GYR
Mode 4  : ACC & MAG
Mode 5  : ACC & GYR
Mode 6  : MAG & GYR
Mode 7  : ACC, MAG & GYR
Mode 8  : IMU
Mode 9  : CMP
Mode 10 : M4G 
Mode 11 : NDOF_FMC
Mode 12 : NDOF
*/
uint8_t BNO_Sensor::setMode(uint8_t reg) {
    return Read16(BNO055_OPR_MODE, reg);
}
  
// Set to reseting system setup sensor
void BNO_Sensor::setReset(uint8_t reg) {
    Write16(BNO055_SYS_TRIGGER, (1 << reg));
    delay(10);
}

/* Set Power BN0 Microkontroller
Mode 0 : Normal
Mode 1 : Low Power
Mode 2 : Suspend
*/
void BNO_Sensor::setPower(uint8_t reg) {
    Write16(BNO055_PWR_MODE, reg);
    delay(10);
}

/* Set to Unit sensor
Bit 0 : Set Accelerometer Unit to MG or M/S^2
Bit 1 : Set Gyroscope Unit to RPS or DPS
Bit 2 : Set Euler Data Unit to DEG or RAD
Bit 4 : Set Temperature Unit to Celcius or Fahrenheit
Bit 7 : Set Orientation Mode to Window Orientation or Android Orientation
Datasheet page 70
*/
void BNO_Sensor::setUnitSel(uint8_t sys, uint8_t temp, uint8_t eul, uint8_t gyr, uint8_t acc) {
    Write16(BNO055_UNIT_SEL, (sys << BNO055_SYS_UNIT) | (temp << BNO055_TEMP_UNIT) | (eul << BNO055_EUL_UNIT) | (gyr << BNO055_GYR_UNIT) | (acc << BNO055_ACC_UNIT));
    COEF_UNIT_EUL = (eul) ? COEF_UNIT_RAD : COEF_UNIT_DEG;
    COEF_UNIT_GYR = (gyr) ? COEF_UNIT_RPS : COEF_UNIT_DPS;
    COEF_UNIT_ACC = (acc) ? COEF_UNIT_MG  : COEF_UNIT_MPS2;
    delay(10);
}
  
/* Set Mode Accelerometer sensor
Bit 0-1 : Set Range Accelerometer 2g to 16g
Bit 2-4 : Set Bandwidth Accelerometer 7.81Hz to 1000Hz
Bit 5-7 : Set Power mode Accelerometer config check in datasheet
*/
void BNO_Sensor::setAccConfig(uint8_t pwr, uint8_t bw, uint8_t range) {
    setMode(OPR_CFG_MODE);
    Write16(BNO055_ACC_CONFIG, (pwr << BNO055_ACC_PWR_MODE) | (bw << BNO055_ACC_BW) | (range << BNO055_ACC_RANGE));
    delay(10);
}

/* Set Mode Magnetometer sensor
Bit 0-2 : Set Rate Magnetometer 2Hz to 30Hz
Bit 3-4 : Set Mode Operasi (OPR) Magnetometer
Bit 5-6 : Set Power mode Magnetometer config check in datasheet
*/
void BNO_Sensor::setMagConfig(uint8_t pwr, uint8_t opr, uint8_t rate) {
    setMode(OPR_CFG_MODE);
    Write16(BNO055_MAG_CONFIG, (pwr << BNO055_CFG_PWR_MODE) | (opr << BNO055_CFG_OPR_MODE) | (rate << BNO055_CFG_RATE_MODE));
    delay(10);
}
  
/* Set Mode Gyroscope sensor
Config 0
Bit 0-2 : Set Range Gyroscope 125Dps to 2000Dps
Bit 3-5 : Set Bandwidth Gyroscope 32Hz to 523Hz

Config 1
Bit 0-2 : Set Power mode Gyroscope config check in datasheet
*/
void BNO_Sensor::setGyrConfig(uint8_t pwr, uint8_t bw, uint8_t range) {
    setMode(OPR_CFG_MODE);
    Write16(BNO055_GYR_CONFIG_0, (bw  << BNO055_GYR_BW) | (range << BNO055_GYR_RANGE));
    Write16(BNO055_GYR_CONFIG_1, (pwr << BNO055_GYR_PWR_MODE));
    delay(10);
}
  
/* Set Orientation sensor
Default Orientation sensor P2
Check in datasheet for configuration
*/
void BNO_Sensor::setAxisRemap(uint8_t remap) {
    Write16(BNO055_AXIS_MAP_CFG,  (uint8_t)(COEF_REMAP[remap] >> 8));
    Write16(BNO055_AXIS_MAP_SIGN, (uint8_t)(COEF_REMAP[remap] & 0xFF));
    COEF_HEADING = COEF_HEADING_P[remap];
    delay(10);
}

/*
Write offset Accelerometer 
Byte 0 LSB Accelerometer X
Byte 1 MSB Accelerometer X
Byte 2 LSB Accelerometer Y
Byte 3 MSB Accelerometer Y
Byte 4 LSB Accelerometer Z
Byte 5 MSB Accelerometer Z
*/
void BNO_Sensor::setAccOffsets(int16_t *Data) {
    WriteBytes(BNO055_ACC_OFFSET_X_LSB, (uint8_t *)Data, 6);
    delay(10);
    // return getCalibStat(BNO055_CALIB_STAT_ACC);
}
  
/*
Write offset Magnetometer 
Byte 0 LSB Magnetometer X
Byte 1 MSB Magnetometer X
Byte 2 LSB Magnetometer Y
Byte 3 MSB Magnetometer Y
Byte 4 LSB Magnetometer Z
Byte 5 MSB Magnetometer Z
*/
void BNO_Sensor::setMagOffsets(int16_t *Data) {
    WriteBytes(BNO055_MAG_OFFSET_X_LSB, (uint8_t *)Data, 6);
    delay(10);
    // return getCalibStat(BNO055_CALIB_STAT_MAG);
}

/*
Write offset Gyroscope 
Byte 0 LSB Gyroscope X
Byte 1 MSB Gyroscope X
Byte 2 LSB Gyroscope Y
Byte 3 MSB Gyroscope Y
Byte 4 LSB Gyroscope Z
Byte 5 MSB Gyroscope Z
*/
void BNO_Sensor::setGyrOffsets(int16_t *Data) {
    WriteBytes(BNO055_GYR_OFFSET_X_LSB, (uint8_t *)Data, 6);
    delay(10);
    // return getCalibStat(BNO055_CALIB_STAT_GYR);
}

/*
Write offset Accelerometer Radius 
Byte 0 LSB Accelerometer Radius
Byte 1 MSB Accelerometer Radius
*/
void BNO_Sensor::setAccRadiusOffsets(int16_t *Data) {
    WriteBytes(BNO055_ACC_RADIUS_LSB, (uint8_t *)Data, 2);
    delay(10);
}

/*
Write offset Magnetometer Radius 
Byte 0 LSB Magnetometer Radius
Byte 1 MSB Magnetometer Radius
*/
void BNO_Sensor::setMagRadiusOffsets(int16_t *Data) {
    WriteBytes(BNO055_MAG_RADIUS_LSB, (uint8_t *)Data, 2);
    delay(10);
}

bool BNO_Sensor::Init(TwoWire *_i2c, uint8_t _add) {
    setAddress(_i2c, _add);
    delay(10);
    return (Read8(BNO055_CHIP_ID) == 0xA0) ? 1 : 0;
}

/* Calibration status reading value
Bit 0-1 : Calibration Status Magnetometer
Bit 2-3 : Calibration Status Accelerometer
Bit 4-5 : Calibration Status Gyroscope
Bit 6-7 : Calibration Status All Sensor
*/
uint8_t BNO_Sensor::getCalibStat(uint8_t reg) {
    return ((Read8(BNO055_CALIB_STAT) >> reg) & 0x03);
}

/* System result reading value
Bit 0 : Status Accelerometer
Bit 1 : Status Magnetometer
Bit 2 : Status Gyroscope
Bit 3 : Status MCU
*/
uint8_t BNO_Sensor::getSTStat(uint8_t reg) {
    return ((Read8(BNO055_ST_RESULT) >> reg) & 0x01);
}

/* System Status reading value
1. System error
2. Initializing peripherals
3. System initialization
4. Executing selftest
5. Sensor fusion algorithm running
6. System running without fusion algorithm
*/
uint8_t BNO_Sensor::getSysStat() {
    return Read8(BNO055_SYS_STATUS);
}

/*
Read offset Accelerometer 
Byte 0 LSB Accelerometer X
Byte 1 MSB Accelerometer X
Byte 2 LSB Accelerometer Y
Byte 3 MSB Accelerometer Y
Byte 4 LSB Accelerometer Z
Byte 5 MSB Accelerometer Z
*/
void BNO_Sensor::getAccOffsets(int16_t *Data) {
    ReadBytes(BNO055_ACC_OFFSET_X_LSB, (uint8_t*)Data, 6);
}

/*
Read offset Magnetometer 
Byte 0 LSB Magnetometer X
Byte 1 MSB Magnetometer X
Byte 2 LSB Magnetometer Y
Byte 3 MSB Magnetometer Y
Byte 4 LSB Magnetometer Z
Byte 5 MSB Magnetometer Z
*/
void BNO_Sensor::getMagOffsets(int16_t *Data) {
    ReadBytes(BNO055_MAG_OFFSET_X_LSB, (uint8_t*)Data, 6);
}

/*
Read offset Gyroscope 
Byte 0 LSB Gyroscope X
Byte 1 MSB Gyroscope X
Byte 2 LSB Gyroscope Y
Byte 3 MSB Gyroscope Y
Byte 4 LSB Gyroscope Z
Byte 5 MSB Gyroscope Z
*/
void BNO_Sensor::getGyrOffsets(int16_t *Data) {
    ReadBytes(BNO055_GYR_OFFSET_X_LSB, (uint8_t*)Data, 6);
}

/*
Read offset Accelerometer Radius 
Byte 0 LSB Accelerometer Radius
Byte 1 MSB Accelerometer Radius
*/
void BNO_Sensor::getAccRadiusOffsets(int16_t *Data) {
    ReadBytes(BNO055_ACC_RADIUS_LSB, (uint8_t*)Data, 2);
}

/*
Read offset Magnetometer Radius 
Byte 0 LSB Magnetometer Radius
Byte 1 MSB Magnetometer Radius
*/
void BNO_Sensor::getMagRadiusOffsets(int16_t *Data) {
    ReadBytes(BNO055_MAG_RADIUS_LSB, (uint8_t*)Data, 2);
}

void BNO_Sensor::getAccCalib(int16_t *acc_offs) {
    int16_t AccRaw[3] = {0, 0, 0};
    int16_t AccSum[3] = {0, 0, 0};

    setMode(OPR_CFG_MODE);
    setAccOffsets(acc_offs);
    setMode(OPR_ACC_MODE);

    for (int i = 0; i < 256; i++) {
      getAccRaw(AccRaw);

      AccSum[0] += AccRaw[0];
      AccSum[1] += AccRaw[1];
      AccSum[2] += (AccRaw[2] - 100);
      delay(10);
    }

    acc_offs[0] = (int16_t)(AccSum[0] / 256);
    acc_offs[1] = (int16_t)(AccSum[1] / 256);
    acc_offs[2] = (int16_t)(AccSum[2] / 256);
}

// void BNO_Sensor::getMagOffsets(uint8_t remap, int16_t *mag_offs) {
void BNO_Sensor::getMagCalib(int16_t *mag_offs) {
    int16_t Mag[3] = {0, 0, 0};
    int16_t Min[3] = { 32767,  32767,  32767};
    int16_t Max[3] = {-32768, -32768, -32768};
    
    setMode(OPR_CFG_MODE);
    setMagOffsets(mag_offs);
    setMode(OPR_MAG_MODE);

    unsigned long startTime = millis();
    while (millis() - startTime < 30000) {  // 🔄 30 detik kalibrasi manual
        getMagRaw(Mag);

        if (Mag[0] < Min[0]) Min[0] = Mag[0]; 
        if (Mag[0] > Max[0]) Max[0] = Mag[0];
        if (Mag[1] < Min[1]) Min[1] = Mag[1]; 
        if (Mag[1] > Max[1]) Max[1] = Mag[1];
        if (Mag[2] < Min[2]) Min[2] = Mag[2]; 
        if (Mag[2] > Max[2]) Max[2] = Mag[2];
        delay(10);
    }

    // ⚡ Kalkulasi offset baru
    mag_offs[0] = (int16_t)((Max[0] + Min[0]) / 2);
    mag_offs[1] = (int16_t)((Max[1] + Min[1]) / 2);
    mag_offs[2] = (int16_t)((Max[2] + Min[2]) / 2);
}

void BNO_Sensor::getGyrCalib(int16_t *gyr_offs) {
    int16_t GyrRaw[3] = {0, 0, 0};
    int16_t GyrSum[3] = {0, 0, 0}; //3*gyro

    setMode(OPR_CFG_MODE);
    setGyrOffsets(gyr_offs);
    setMode(OPR_GYR_MODE);

    for (int i = 0; i < 256; i++) {
      getGyrRaw(GyrRaw);

      GyrSum[0] += GyrRaw[0];
      GyrSum[1] += GyrRaw[1];
      GyrSum[2] += GyrRaw[2];
      delay(10);
    }
    
    gyr_offs[0] = (int16_t)(GyrSum[0] / 256);
    gyr_offs[1] = (int16_t)(GyrSum[1] / 256);
    gyr_offs[2] = (int16_t)(GyrSum[2] / 256);
}

bool BNO_Sensor::getAccRaw(int16_t *Acc, size_t len) {
    return ReadBytes(BNO055_ACC_DATA_X_LSB, (uint8_t*)Acc, len);
}

bool BNO_Sensor::getMagRaw(int16_t *Mag, size_t len) {
    return ReadBytes(BNO055_MAG_DATA_X_LSB, (uint8_t*)Mag, len);
}

bool BNO_Sensor::getGyrRaw(int16_t *Gyr, size_t len) {
    return ReadBytes(BNO055_GYR_DATA_X_LSB, (uint8_t*)Gyr, len);
}

bool BNO_Sensor::getAccelaration(Axis_3D_t *Acc) {
    int16_t AccRaw[3] = {0, 0, 0};
    if (getAccRaw(AccRaw)) {
        Acc->X = AccRaw[0] / COEF_UNIT_ACC;
        Acc->Y = AccRaw[1] / COEF_UNIT_ACC;
        Acc->Z = AccRaw[2] / COEF_UNIT_ACC;
        return 1;
    }
    return 0;
}

bool BNO_Sensor::getMagnetometer(Axis_3D_t *Mag) {
    int16_t MagRaw[3] = {0, 0, 0};
    if (getMagRaw(MagRaw)) {
        Mag->X = MagRaw[0] / COEF_UNIT_uT;
        Mag->Y = MagRaw[1] / COEF_UNIT_uT;
        Mag->Z = MagRaw[2] / COEF_UNIT_uT;
        return 1;
    }
    return 0;
}

bool BNO_Sensor::getGyroscope(Axis_3D_t *Gyr) {
    int16_t GyrRaw[3] = {0, 0, 0};
    if (getGyrRaw(GyrRaw)) {
        Gyr->X = GyrRaw[0] / COEF_UNIT_GYR;
        Gyr->Y = GyrRaw[1] / COEF_UNIT_GYR;
        Gyr->Z = GyrRaw[2] / COEF_UNIT_GYR;
        return 1;
    }
    return 0;
}

bool BNO_Sensor::getEuler(Axis_3D_t *Eul) {
    int16_t rx_buff[3] = {0, 0, 0};
    if (ReadBytes(BNO055_EUL_DATA_Z_LSB, (uint8_t*)rx_buff, sizeof(rx_buff))) {
        Eul->X = rx_buff[1] / COEF_UNIT_EUL;
        Eul->Y = rx_buff[2] / COEF_UNIT_EUL;
        Eul->Z = rx_buff[0] / COEF_UNIT_EUL;
        return 1;
    }
    return 0;
}

bool BNO_Sensor::getQuaternion(Axis_4D_t *Qua) {
    int16_t rx_buff[4] = {0, 0, 0, 0};
    if (ReadBytes(BNO055_QUA_DATA_W_LSB, (uint8_t*)rx_buff, sizeof(rx_buff))) {
        Qua->W = rx_buff[0] / COEF_UNIT_LESS;
        Qua->X = rx_buff[1] / COEF_UNIT_LESS;
        Qua->Y = rx_buff[2] / COEF_UNIT_LESS;
        Qua->Z = rx_buff[3] / COEF_UNIT_LESS;
        return 1;
    }
    return 0;
}

bool BNO_Sensor::getAccLinear(Axis_3D_t *Lia) {
    int16_t rx_buff[3] = {0, 0, 0};
    if (ReadBytes(BNO055_LIA_DATA_X_LSB, (uint8_t*)rx_buff, sizeof(rx_buff))) {
        Lia->X = rx_buff[0] / COEF_UNIT_ACC;
        Lia->Y = rx_buff[1] / COEF_UNIT_ACC;
        Lia->Z = rx_buff[2] / COEF_UNIT_ACC;
        return 1;
    }
    return 0;
}

bool BNO_Sensor::getGravity(Axis_3D_t *Grv) {
    int16_t rx_buff[3] = {0, 0, 0};
    if (ReadBytes(BNO055_GRV_DATA_X_LSB, (uint8_t*)rx_buff, sizeof(rx_buff))) {
        Grv->X = rx_buff[0] / COEF_UNIT_ACC;
        Grv->Y = rx_buff[1] / COEF_UNIT_ACC;
        Grv->Z = rx_buff[2] / COEF_UNIT_ACC;
        return 1;
    }
    return 0;
}

bool BNO_Sensor::getTemperature(int8_t *Temp) {
    int8_t rx_buff = 0;
    if (ReadBytes(BNO055_DATA_TEMP, (uint8_t*)&rx_buff, sizeof(rx_buff))) {
        *Temp = rx_buff;
        return 1;
    }
    return 0;
}

bool BNO_Sensor::getAllData(BNO055_t *AHRS) {
    int16_t rx_buff[23];
    #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega2560__) 
    // Arduino Uno, Nano and Mega reading max 32 byte i2c data
    if (ReadBytes(BNO055_ACC_DATA_X_LSB, (uint8_t*)&rx_buff[0], 24)) {
        AHRS->ACC.X = rx_buff[0]  / COEF_UNIT_ACC;
        AHRS->ACC.Y = rx_buff[1]  / COEF_UNIT_ACC;
        AHRS->ACC.Z = rx_buff[2]  / COEF_UNIT_ACC;
        
        AHRS->MAG.X = rx_buff[3]  / COEF_UNIT_uT;
        AHRS->MAG.Y = rx_buff[4]  / COEF_UNIT_uT;
        AHRS->MAG.Z = rx_buff[5]  / COEF_UNIT_uT;
        
        AHRS->GYR.X = rx_buff[6]  / COEF_UNIT_GYR;
        AHRS->GYR.Y = rx_buff[7]  / COEF_UNIT_GYR;
        AHRS->GYR.Z = rx_buff[8]  / COEF_UNIT_GYR;
        
        AHRS->EUL.Z = rx_buff[9]  / COEF_UNIT_EUL;
        AHRS->EUL.X = rx_buff[10] / COEF_UNIT_EUL;
        AHRS->EUL.Y = rx_buff[11] / COEF_UNIT_EUL;

        if (ReadBytes(BNO055_QUA_DATA_W_LSB, (uint8_t*)&rx_buff[12], 21)) {
            AHRS->QUA.W = rx_buff[12] / COEF_UNIT_LESS;
            AHRS->QUA.X = rx_buff[13] / COEF_UNIT_LESS;
            AHRS->QUA.Y = rx_buff[14] / COEF_UNIT_LESS;
            AHRS->QUA.Z = rx_buff[15] / COEF_UNIT_LESS;
            
            AHRS->LIA.X = rx_buff[16] / COEF_UNIT_ACC;
            AHRS->LIA.Y = rx_buff[17] / COEF_UNIT_ACC;
            AHRS->LIA.Z = rx_buff[18] / COEF_UNIT_ACC;
            
            AHRS->GRV.X = rx_buff[19] / COEF_UNIT_ACC;
            AHRS->GRV.Y = rx_buff[20] / COEF_UNIT_ACC;
            AHRS->GRV.Z = rx_buff[21] / COEF_UNIT_ACC;
            
            AHRS->TEMP  = rx_buff[22] & 0xFF;
            return 1;
        }
    }

    #else
    if (ReadBytes(BNO055_ACC_DATA_X_LSB, (uint8_t*)rx_buff, sizeof(rx_buff) - 1)) {
        AHRS->ACC.X = rx_buff[0]  / COEF_UNIT_ACC;
        AHRS->ACC.Y = rx_buff[1]  / COEF_UNIT_ACC;
        AHRS->ACC.Z = rx_buff[2]  / COEF_UNIT_ACC;
        
        AHRS->MAG.X = rx_buff[3]  / COEF_UNIT_uT;
        AHRS->MAG.Y = rx_buff[4]  / COEF_UNIT_uT;
        AHRS->MAG.Z = rx_buff[5]  / COEF_UNIT_uT;
        
        AHRS->GYR.X = rx_buff[6]  / COEF_UNIT_GYR;
        AHRS->GYR.Y = rx_buff[7]  / COEF_UNIT_GYR;
        AHRS->GYR.Z = rx_buff[8]  / COEF_UNIT_GYR;
        
        AHRS->EUL.Z = rx_buff[9]  / COEF_UNIT_EUL;
        AHRS->EUL.X = rx_buff[10] / COEF_UNIT_EUL;
        AHRS->EUL.Y = rx_buff[11] / COEF_UNIT_EUL;
        
        AHRS->QUA.W = rx_buff[12] / COEF_UNIT_LESS;
        AHRS->QUA.X = rx_buff[13] / COEF_UNIT_LESS;
        AHRS->QUA.Y = rx_buff[14] / COEF_UNIT_LESS;
        AHRS->QUA.Z = rx_buff[15] / COEF_UNIT_LESS;
        
        AHRS->LIA.X = rx_buff[16] / COEF_UNIT_ACC;
        AHRS->LIA.Y = rx_buff[17] / COEF_UNIT_ACC;
        AHRS->LIA.Z = rx_buff[18] / COEF_UNIT_ACC;
        
        AHRS->GRV.X = rx_buff[19] / COEF_UNIT_ACC;
        AHRS->GRV.Y = rx_buff[20] / COEF_UNIT_ACC;
        AHRS->GRV.Z = rx_buff[21] / COEF_UNIT_ACC;
        
        AHRS->TEMP  = rx_buff[22] & 0xFF;
        return 1;
    }
    #endif
    return 0;
}

float BNO_Sensor::getHeading(Axis_4D_t *Qua, Axis_3D_t *Mag) {
    Axis_2D_t Angle = {0, 0};
    Axis_2D_t Magno = {0, 0};

    // Calculate angle rad with quaternion
    Angle.X = asinf(2.0f * (Qua->W * Qua->X + Qua->Y * Qua->Z)); // Rad Roll
    Angle.Y = asinf(2.0f * (Qua->W * Qua->Y - Qua->X * Qua->Z)); // Rad Pitch

    // Calculate compensation heading
    Magno.X = Mag->X * cosf(Angle.Y) + Mag->Z * sinf(Angle.Y);
    Magno.Y = Mag->X * sinf(Angle.X) * sinf(Angle.Y) + Mag->Y * cosf(Angle.X) - Mag->Z * sinf(Angle.X) * cosf(Angle.Y);

    return wrap((atan2f(Magno.Y, Magno.X) * RAD_TO_DEG) - COEF_HEADING, 180);
}

int16_t BNO_Sensor::getRadiusCompute(int16_t *Axis) {
    return sqrt(Axis[0] * Axis[0] + Axis[1] * Axis[1] + Axis[2] * Axis[2]);
}

BNO_Sensor BNO;