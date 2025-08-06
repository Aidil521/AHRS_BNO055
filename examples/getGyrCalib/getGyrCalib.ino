#include <AHRS_BNO055.h>

// Read datasheet and file BNO055_REG.h if want to set configuration BNO055
int16_t GyrOff[3]  = {0, 0, 0};
int16_t sGyrOff[3] = {0, 0, 0};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
  
  while (!BNO.Init(&Wire, BNO055_ADD_SLAVE_L)) { // Alternate Address: BNO055_ADD_SLAVE_H
    Serial.println("Error, failed detection ID BNO055!");
  }
  delay(1000);
  Serial.println("🔄 Mulai kalibrasi manual accelerometer BNO055...");
  BNO.getAccCalib(sGyrOff);
  Serial.println("✅ Kalibrasi selesai.");
  Serial.print("Offset X: "); Serial.println(sGyrOff[0]);
  Serial.print("Offset Y: "); Serial.println(sGyrOff[1]);
  Serial.print("Offset Z: "); Serial.println(sGyrOff[2]);
  BNO.setGyrOffsets(sGyrOff);
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
  BNO.getGyrOffsets(GyrOff);
  Serial.println(GyrOff[0]);
  Serial.println(GyrOff[1]);
  Serial.println(GyrOff[2]);
  delay(5000);
}

void loop() {
  BNO055_t AHRS;
  if (BNO.getGyroscope(&AHRS.GYR)) {
    Serial.println("GYR X : " + String(AHRS.GYR.X));
    Serial.println("GYR Y : " + String(AHRS.GYR.Y));
    Serial.println("GYR Z : " + String(AHRS.GYR.Z));
  }
  if (BNO.getEuler(&AHRS.EUL)) {
    Serial.println("EUL X : " + String(AHRS.EUL.X));
    Serial.println("EUL Y : " + String(AHRS.EUL.Y));
    Serial.println("EUL Z : " + String(AHRS.EUL.Z));
  }
  Serial.println();
  delay(100);
}