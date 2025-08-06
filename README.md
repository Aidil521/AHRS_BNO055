# AHRS BNO055

## Overview
AHRS_BNO055 is a lightweight library designed to provide essential functionalities for Arduino with minimal footprint. It is ideal for resource-constrained projects where efficiency and simplicity are crucial.

## Features
- Reading Sensor AHRS BNO055 with I2C Communication
- Used to read Accel, Gyro, Magneto, Euler, Quaternion, Linear Accel, Gravity and Temperature.

## Installation
1. Download the latest release from [GitHub](https://github.com/Aidil521/AHRS_BNO055).
2. Extract the folder and place it inside the Arduino `libraries` directory.
3. Restart the Arduino IDE.
4. Include the library in your project:

   ```cpp
   #include <AHRS_BNO055.h>
   ```

## Usage
### Read Euler Data
```cpp
#include <AHRS_BNO055.h>

// Read datasheet and file BNO055_REG.h if want to set configuration BNO055
int16_t MagOff[3]  = {0, 0, 0};
int16_t sMagOff[3] = {164, -5, 94};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
  
  while (!BNO.Init(&Wire, BNO055_ADD_SLAVE_L)) { // Alternate Address: BNO055_ADD_SLAVE_H
    Serial.println("Error, failed detection ID BNO055!");
  }

  Serial.println("Start Initializing BNO");
  if (BNO.setMode(OPR_CFG_MODE) == OPR_CFG_MODE) Serial.println("OPR_CFG_MODE");
  BNO.setMagOffsets(sMagOff);  
  BNO.setPower(BNO055_NORMAL_MODE);
  BNO.setUnitSel(SYS_UNIT_ANDROID, TEMP_UNIT_C, ACC_UNIT_DEG, GYR_UNIT_DPS, ACC_UNIT_MPS2);
  BNO.setAccConfig(BNO055_ACC_PWR_NORMAL, BNO055_ACC_BW_250Hz, BNO055_ACC_RANGE_16G);
  BNO.setMagConfig(BNO055_MAG_PWR_NORMAL, BNO055_MAG_OPR_RGR,  BNO055_MAG_RATE_30Hz);
  BNO.setGyrConfig(BNO055_GYR_PWR_NORMAL, BNO055_GYR_BW_230Hz, BNO055_GYR_RANGE_125dps);
  BNO.setAxisRemap(REMAP_CFG_P0);
  if (BNO.setMode(OPR_NDOF_MODE) == OPR_NDOF_MODE) Serial.println("OPR_NDOF_MODE");
  Serial.println("Wait timeout 5s");
  BNO.getMagOffsets(MagOff);
  Serial.println(MagOff[0]);
  Serial.println(MagOff[1]);
  Serial.println(MagOff[2]);
  delay(5000);
}

void loop() {
  // put your main code here, to run repeatedly:
  BNO055_t AHRS;
  if (BNO.getEuler(&AHRS.EUL)) {
    Serial.println("EUL X : " + String(AHRS.EUL.X));
    Serial.println("EUL Y : " + String(AHRS.EUL.Y));
    Serial.println("EUL Z : " + String(AHRS.EUL.Z));
  }
  Serial.println();
  delay(100);
}
```

### Read All Data
```cpp
#include <AHRS_BNO055.h>

// Read datasheet and file BNO055_REG.h if want to set configuration BNO055
int16_t MagOff[3]  = {0, 0, 0};
int16_t sMagOff[3] = {164, -5, 94};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
  
  while (!BNO.Init(&Wire, BNO055_ADD_SLAVE_L)) { // Alternate Address: BNO055_ADD_SLAVE_H
    Serial.println("Error, failed detection ID BNO055!");
  }

  Serial.println("Start Initializing BNO");
  if (BNO.setMode(OPR_CFG_MODE) == OPR_CFG_MODE) Serial.println("OPR_CFG_MODE");
  BNO.setMagOffsets(sMagOff);  
  BNO.setPower(BNO055_NORMAL_MODE);
  BNO.setUnitSel(SYS_UNIT_ANDROID, TEMP_UNIT_C, ACC_UNIT_DEG, GYR_UNIT_DPS, ACC_UNIT_MPS2);
  BNO.setAccConfig(BNO055_ACC_PWR_NORMAL, BNO055_ACC_BW_250Hz, BNO055_ACC_RANGE_16G);
  BNO.setMagConfig(BNO055_MAG_PWR_NORMAL, BNO055_MAG_OPR_RGR,  BNO055_MAG_RATE_30Hz);
  BNO.setGyrConfig(BNO055_GYR_PWR_NORMAL, BNO055_GYR_BW_230Hz, BNO055_GYR_RANGE_125dps);
  BNO.setAxisRemap(REMAP_CFG_P0);
  if (BNO.setMode(OPR_NDOF_MODE) == OPR_NDOF_MODE) Serial.println("OPR_NDOF_MODE");
  Serial.println("Wait timeout 5s");
  BNO.getMagOffsets(MagOff);
  Serial.println(MagOff[0]);
  Serial.println(MagOff[1]);
  Serial.println(MagOff[2]);
  delay(5000);
}

void loop() {
  // put your main code here, to run repeatedly:
  BNO055_t AHRS;
  if (BNO.getAllData(&AHRS)) {
    Serial.println("ACC X : " + String(AHRS.ACC.X));
    Serial.println("ACC Y : " + String(AHRS.ACC.Y));
    Serial.println("ACC Z : " + String(AHRS.ACC.Z));
    
    Serial.println("MAG X : " + String(AHRS.MAG.X));
    Serial.println("MAG Y : " + String(AHRS.MAG.Y));
    Serial.println("MAG Z : " + String(AHRS.MAG.Z));

    Serial.println("GYR X : " + String(AHRS.GYR.X));
    Serial.println("GYR Y : " + String(AHRS.GYR.Y));
    Serial.println("GYR Z : " + String(AHRS.GYR.Z));

    Serial.println("EUL X : " + String(AHRS.EUL.X));
    Serial.println("EUL Y : " + String(AHRS.EUL.Y));
    Serial.println("EUL Z : " + String(AHRS.EUL.Z));

    Serial.println("QUA W : " + String(AHRS.QUA.W, 5));
    Serial.println("QUA X : " + String(AHRS.QUA.X, 5));
    Serial.println("QUA Y : " + String(AHRS.QUA.Y, 5));
    Serial.println("QUA Z : " + String(AHRS.QUA.Z, 5));

    Serial.println("LIA X : " + String(AHRS.LIA.X));
    Serial.println("LIA Y : " + String(AHRS.LIA.Y));
    Serial.println("LIA Z : " + String(AHRS.LIA.Z));

    Serial.println("GRV X : " + String(AHRS.GRV.X));
    Serial.println("GRV Y : " + String(AHRS.GRV.Y));
    Serial.println("GRV Z : " + String(AHRS.GRV.Z));

    Serial.println("TEMP  : " + String(AHRS.TEMP));
  }
  else {
    Serial.println("Erorr, failed read data BNO055!");
  }
  Serial.println();
  delay(100);
}
```

### Calibration Magnetometer
```cpp
#include <AHRS_BNO055.h>

// Read datasheet and file BNO055_REG.h if want to set configuration BNO055
int16_t MagOff[3]  = {0, 0, 0};
int16_t sMagOff[3] = {0, 0, 0};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
  
  while (!BNO.Init(&Wire, BNO055_ADD_SLAVE_L)) { // Alternate Address: BNO055_ADD_SLAVE_H
    Serial.println("Error, failed detection ID BNO055!");
  }
  delay(1000);
  Serial.println("🔄 Mulai kalibrasi manual magnetometer BNO055...");
  BNO.getMagCalib(sMagOff);
  Serial.println("✅ Kalibrasi selesai.");
  Serial.print("Offset X: "); Serial.println(sMagOff[0]);
  Serial.print("Offset Y: "); Serial.println(sMagOff[1]);
  Serial.print("Offset Z: "); Serial.println(sMagOff[2]);
  BNO.setMagOffsets(sMagOff);
  Serial.println("Write Data Offset on BNO055");

  if (BNO.setMode(OPR_CFG_MODE) == OPR_CFG_MODE) Serial.println("OPR_CFG_MODE");
  BNO.setPower(BNO055_NORMAL_MODE);
  BNO.setUnitSel(SYS_UNIT_ANDROID, TEMP_UNIT_C, ACC_UNIT_DEG, GYR_UNIT_DPS, ACC_UNIT_MPS2);
  BNO.setAccConfig(BNO055_ACC_PWR_NORMAL, BNO055_ACC_BW_250Hz, BNO055_ACC_RANGE_16G);
  BNO.setMagConfig(BNO055_MAG_PWR_NORMAL, BNO055_MAG_OPR_RGR,  BNO055_MAG_RATE_30Hz);
  BNO.setGyrConfig(BNO055_GYR_PWR_NORMAL, BNO055_GYR_BW_230Hz, BNO055_GYR_RANGE_125dps);
  BNO.setAxisRemap(REMAP_CFG_P0);
  if (BNO.setMode(OPR_NDOF_MODE) == OPR_NDOF_MODE) Serial.println("OPR_NDOF_MODE");
  Serial.println("Wait timeout 5s");
  Serial.println("Read Data Offset on BNO055");
  BNO.getMagOffsets(MagOff);
  Serial.println(MagOff[0]);
  Serial.println(MagOff[1]);
  Serial.println(MagOff[2]);
  delay(5000);
}

void loop() {
  BNO055_t AHRS;
  if (BNO.getMagnetometer(&AHRS.MAG)) {
    Serial.println("MAG X : " + String(AHRS.MAG.X));
    Serial.println("MAG Y : " + String(AHRS.MAG.Y));
    Serial.println("MAG Z : " + String(AHRS.MAG.Z));
  }
  if (BNO.getEuler(&AHRS.EUL)) {
    Serial.println("EUL X : " + String(AHRS.EUL.X));
    Serial.println("EUL Y : " + String(AHRS.EUL.Y));
    Serial.println("EUL Z : " + String(AHRS.EUL.Z));
  }
  Serial.println();
  delay(100);
}
```


## Contributing
Contributions are welcome! Please fork the repository and submit a pull request with your improvements.

## Author
**@Aidil521**  
[GitHub](https://github.com/Aidil521)  
[Email](mailto:muhamadaidil776@mail.com)
