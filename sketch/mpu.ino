#include <Wire.h>

// Endereço I2C - scann de endereços mostrou 0x68
#define MPU_ADDR 0x68 

// Registradores básicos
#define PWR_MGMT_1   0x6B
#define SMPLRT_DIV   0x19
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define TEMP_OUT_H   0x41
#define GYRO_XOUT_H  0x43
#define WHO_AM_I     0x75

// Escolhas de escala (ajuste conforme quiser)
#define ACCEL_RANGE_SEL 2   // 0=±2g,1=±4g,2=±8g,3=±16g
#define GYRO_RANGE_SEL  1   // 0=±250°/s,1=±500°/s,2=±1000,3=±2000

// Escalas em LSB por unidade (correspondente às seleções acima)
float accel_lsb_per_g() {
  switch (ACCEL_RANGE_SEL) {
    case 0: return 16384.0; // ±2g
    case 1: return 8192.0;  // ±4g
    case 2: return 4096.0;  // ±8g
    case 3: return 2048.0;  // ±16g
  }
  return 16384.0;
}
float gyro_lsb_per_dps() {
  switch (GYRO_RANGE_SEL) {
    case 0: return 131.0;   // ±250
    case 1: return 65.5;    // ±500
    case 2: return 32.8;    // ±1000
    case 3: return 16.4;    // ±2000
  }
  return 131.0;
}

// Offsets de calibração (inicial 0, use calibrateOffsets para preencher)
long ax_off = 0, ay_off = 0, az_off = 0;
long gx_off = 0, gy_off = 0, gz_off = 0;

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)1);
  if (Wire.available()) return Wire.read();
  return 0xFF;
}

void readRegs(uint8_t reg, uint8_t count, uint8_t *buf) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, count);
  for (uint8_t i=0;i<count && Wire.available();i++) buf[i] = Wire.read();
}

void wakeSensor() {
  // wake up
  writeReg(PWR_MGMT_1, 0x00);
  delay(50);
}

void configureSensor() {
  // sample rate divider: keep 1k/(1+SMPLRT_DIV) default
  writeReg(SMPLRT_DIV, 0x00);
  // DLPF - low pass filter: 0x03 ~ 44Hz (ajuste se quiser)
  writeReg(CONFIG, 0x03);

  // set gyro FS_SEL (bits 4:3)
  writeReg(GYRO_CONFIG, (GYRO_RANGE_SEL << 3));
  // set accel AFS_SEL (bits 4:3)
  writeReg(ACCEL_CONFIG, (ACCEL_RANGE_SEL << 3));
  delay(20);
}

// void setup() {
//   Serial.begin(115200);
//   delay(100);
//   Wire.begin(21, 22); // SDA, SCL
//   Wire.setClock(100000); // 100kHz é seguro

//   Serial.println("Init raw MPU-style driver (ignora WHO_AM_I)");
//   wakeSensor();
//   configureSensor();

//   // opcional: imprime WHO_AM_I só para referência
//   uint8_t who = readReg(WHO_AM_I);
//   Serial.print("WHO_AM_I raw: 0x");
//   Serial.println(who, HEX);
//   Serial.println("Pronto. Rode calibrateOffsets() parado para calibrar (opcional).");
// }

// void loop() {
//   // leitura
//   uint8_t buf[14];
//   readRegs(ACCEL_XOUT_H, 14, buf);

//   int16_t ax = (buf[0] << 8) | buf[1];
//   int16_t ay = (buf[2] << 8) | buf[3];
//   int16_t az = (buf[4] << 8) | buf[5];
//   int16_t temp = (buf[6] << 8) | buf[7];
//   int16_t gx = (buf[8] << 8) | buf[9];
//   int16_t gy = (buf[10] << 8) | buf[11];
//   int16_t gz = (buf[12] << 8) | buf[13];

//   // aplica offsets de calibração
//   long ax_c = (long)ax - ax_off;
//   long ay_c = (long)ay - ay_off;
//   long az_c = (long)az - az_off;
//   long gx_c = (long)gx - gx_off;
//   long gy_c = (long)gy - gy_off;
//   long gz_c = (long)gz - gz_off;

//   // converte para unidades
//   float a_g = (float)ax_c / accel_lsb_per_g();
//   float b_g = (float)ay_c / accel_lsb_per_g();
//   float c_g = (float)az_c / accel_lsb_per_g();

//   float g_dps_x = (float)gx_c / gyro_lsb_per_dps();
//   float g_dps_y = (float)gy_c / gyro_lsb_per_dps();
//   float g_dps_z = (float)gz_c / gyro_lsb_per_dps();

//   float temperature = (temp / 340.0) + 36.53; // fórmula do datasheet

//   Serial.print("AX(g): "); Serial.print(a_g, 3);
//   Serial.print(" AY(g): "); Serial.println(b_g, 3);
//   // Serial.print(" AZ(g): "); Serial.print(c_g, 3);

//   // Serial.print("  GX(dps): "); Serial.print(g_dps_x, 2);
//   // Serial.print(" GY(dps): "); Serial.print(g_dps_y, 2);
//   // Serial.print(" GZ(dps): "); Serial.print(g_dps_z, 2);

//   // Serial.print("  T(C): "); Serial.println(temperature, 2);

//   delay(10);
// }

// ----- Calibração simples: segura o módulo imóvel e roda -----
void calibrateOffsets(uint16_t samples = 500) {
  long sax=0, say=0, saz=0, sgx=0, sgy=0, sgz=0;
  Serial.println("Calibrando... mantenha módulo estático.");
  for (uint16_t i=0;i<samples;i++) {
    uint8_t b[14];
    readRegs(ACCEL_XOUT_H, 14, b);
    int16_t ax = (b[0] << 8) | b[1];
    int16_t ay = (b[2] << 8) | b[3];
    int16_t az = (b[4] << 8) | b[5];
    int16_t gx = (b[8] << 8) | b[9];
    int16_t gy = (b[10] << 8) | b[11];
    int16_t gz = (b[12] << 8) | b[13];

    sax += ax; say += ay; saz += az;
    sgx += gx; sgy += gy; sgz += gz;
    delay(5);
  }
  ax_off = sax / samples;
  ay_off = say / samples;
  az_off = (saz / samples) - (long)accel_lsb_per_g(); // subtrai 1g no z se estiver em repouso
  gx_off = sgx / samples;
  gy_off = sgy / samples;
  gz_off = sgz / samples;
  Serial.println("Calibração feita. Offsets definidos.");
}