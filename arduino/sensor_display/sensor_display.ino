/*
  MTRN3100 sensor display diagnostic.

  Shows MPU6050 X/Y/Z angles and three VL6180X distance readings on the
  SSD1306 I2C OLED from the course controller PCB.

  Schematic notes:
    OLED SSD1306: SDA A4, SCL A5
    MPU6050:      SDA A4, SCL A5
    VL6180X 1:    GPIO0/XSHUT A0
    Physical right VL6180X: GPIO0/XSHUT A1
    Physical front VL6180X: GPIO0/XSHUT A2

  Required Arduino libraries:
    MPU6050_light by rfetick
    VL6180X by Pololu
*/

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <VL6180X.h>

// ------------------------- PCB Mapping -------------------------
const uint8_t LIDAR_LEFT_XSHUT = A0;   // TOF1GPO on the schematic
const uint8_t LIDAR_FRONT_XSHUT = A2;  // Physical front sensor on this robot
const uint8_t LIDAR_RIGHT_XSHUT = A1;  // Physical right sensor on this robot

const uint8_t ADDR_LEFT = 0x54;
const uint8_t ADDR_FRONT = 0x56;
const uint8_t ADDR_RIGHT = 0x55;

const uint8_t OLED_ADDR_PRIMARY = 0x3C;
const uint8_t OLED_ADDR_SECONDARY = 0x3D;

MPU6050 mpu(Wire);

VL6180X lidarLeft;
VL6180X lidarFront;
VL6180X lidarRight;

bool oledOk = false;
bool imuOk = false;
bool lidarLeftOk = false;
bool lidarFrontOk = false;
bool lidarRightOk = false;

unsigned long lastDisplayMs = 0;
unsigned long lastSerialMs = 0;
bool layoutDrawn = false;

// -------------------- Tiny SSD1306 Text Driver -------------------
// Minimal 5x7 ASCII font, columns left-to-right, bit0 at the top.
const uint8_t FONT_5X7[] PROGMEM = {
  0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x5F,0x00,0x00, 0x00,0x07,0x00,0x07,0x00, 0x14,0x7F,0x14,0x7F,0x14,
  0x24,0x2A,0x7F,0x2A,0x12, 0x23,0x13,0x08,0x64,0x62, 0x36,0x49,0x55,0x22,0x50, 0x00,0x05,0x03,0x00,0x00,
  0x00,0x1C,0x22,0x41,0x00, 0x00,0x41,0x22,0x1C,0x00, 0x14,0x08,0x3E,0x08,0x14, 0x08,0x08,0x3E,0x08,0x08,
  0x00,0x50,0x30,0x00,0x00, 0x08,0x08,0x08,0x08,0x08, 0x00,0x60,0x60,0x00,0x00, 0x20,0x10,0x08,0x04,0x02,
  0x3E,0x51,0x49,0x45,0x3E, 0x00,0x42,0x7F,0x40,0x00, 0x42,0x61,0x51,0x49,0x46, 0x21,0x41,0x45,0x4B,0x31,
  0x18,0x14,0x12,0x7F,0x10, 0x27,0x45,0x45,0x45,0x39, 0x3C,0x4A,0x49,0x49,0x30, 0x01,0x71,0x09,0x05,0x03,
  0x36,0x49,0x49,0x49,0x36, 0x06,0x49,0x49,0x29,0x1E, 0x00,0x36,0x36,0x00,0x00, 0x00,0x56,0x36,0x00,0x00,
  0x08,0x14,0x22,0x41,0x00, 0x14,0x14,0x14,0x14,0x14, 0x00,0x41,0x22,0x14,0x08, 0x02,0x01,0x51,0x09,0x06,
  0x32,0x49,0x79,0x41,0x3E, 0x7E,0x11,0x11,0x11,0x7E, 0x7F,0x49,0x49,0x49,0x36, 0x3E,0x41,0x41,0x41,0x22,
  0x7F,0x41,0x41,0x22,0x1C, 0x7F,0x49,0x49,0x49,0x41, 0x7F,0x09,0x09,0x09,0x01, 0x3E,0x41,0x49,0x49,0x7A,
  0x7F,0x08,0x08,0x08,0x7F, 0x00,0x41,0x7F,0x41,0x00, 0x20,0x40,0x41,0x3F,0x01, 0x7F,0x08,0x14,0x22,0x41,
  0x7F,0x40,0x40,0x40,0x40, 0x7F,0x02,0x0C,0x02,0x7F, 0x7F,0x04,0x08,0x10,0x7F, 0x3E,0x41,0x41,0x41,0x3E,
  0x7F,0x09,0x09,0x09,0x06, 0x3E,0x41,0x51,0x21,0x5E, 0x7F,0x09,0x19,0x29,0x46, 0x46,0x49,0x49,0x49,0x31,
  0x01,0x01,0x7F,0x01,0x01, 0x3F,0x40,0x40,0x40,0x3F, 0x1F,0x20,0x40,0x20,0x1F, 0x3F,0x40,0x38,0x40,0x3F,
  0x63,0x14,0x08,0x14,0x63, 0x07,0x08,0x70,0x08,0x07, 0x61,0x51,0x49,0x45,0x43, 0x00,0x7F,0x41,0x41,0x00,
  0x02,0x04,0x08,0x10,0x20, 0x00,0x41,0x41,0x7F,0x00, 0x04,0x02,0x01,0x02,0x04, 0x40,0x40,0x40,0x40,0x40,
  0x00,0x01,0x02,0x04,0x00, 0x20,0x54,0x54,0x54,0x78, 0x7F,0x48,0x44,0x44,0x38, 0x38,0x44,0x44,0x44,0x20,
  0x38,0x44,0x44,0x48,0x7F, 0x38,0x54,0x54,0x54,0x18, 0x08,0x7E,0x09,0x01,0x02, 0x0C,0x52,0x52,0x52,0x3E,
  0x7F,0x08,0x04,0x04,0x78, 0x00,0x44,0x7D,0x40,0x00, 0x20,0x40,0x44,0x3D,0x00, 0x7F,0x10,0x28,0x44,0x00,
  0x00,0x41,0x7F,0x40,0x00, 0x7C,0x04,0x18,0x04,0x78, 0x7C,0x08,0x04,0x04,0x78, 0x38,0x44,0x44,0x44,0x38,
  0x7C,0x14,0x14,0x14,0x08, 0x08,0x14,0x14,0x18,0x7C, 0x7C,0x08,0x04,0x04,0x08, 0x48,0x54,0x54,0x54,0x20,
  0x04,0x3F,0x44,0x40,0x20, 0x3C,0x40,0x40,0x20,0x7C, 0x1C,0x20,0x40,0x20,0x1C, 0x3C,0x40,0x30,0x40,0x3C,
  0x44,0x28,0x10,0x28,0x44, 0x0C,0x50,0x50,0x50,0x3C, 0x44,0x64,0x54,0x4C,0x44, 0x00,0x08,0x36,0x41,0x00,
  0x00,0x00,0x7F,0x00,0x00, 0x00,0x41,0x36,0x08,0x00, 0x10,0x08,0x08,0x10,0x08
};

class TinySSD1306 {
public:
  bool begin() {
    address = detectAddress();
    bool acknowledged = (address != 0);
    if (!acknowledged) {
      address = OLED_ADDR_PRIMARY;
    }

    const uint8_t initCommands[] = {
      0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
      0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x12,
      0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6,
      0x2E, 0xAF
    };

    for (uint8_t i = 0; i < sizeof(initCommands); ++i) {
      command(initCommands[i]);
    }
    clear();
    return acknowledged;
  }

  void clear() {
    for (uint8_t page = 0; page < 8; ++page) {
      setCursor(0, page);
      for (uint8_t col = 0; col < 128; ++col) {
        data(0x00);
      }
    }
    setCursor(0, 0);
  }

  void drawString(uint8_t col, uint8_t row, const char *text) {
    setCursor(col * 8, row);
    while (*text && col < 16) {
      drawChar(*text++);
      col++;
    }
  }

  void clearRow(uint8_t row) {
    setCursor(0, row);
    for (uint8_t col = 0; col < 128; ++col) {
      data(0x00);
    }
  }

  uint8_t i2cAddress() const {
    return address;
  }

private:
  uint8_t address = 0;

  uint8_t detectAddress() {
    Wire.beginTransmission(OLED_ADDR_PRIMARY);
    if (Wire.endTransmission() == 0) {
      return OLED_ADDR_PRIMARY;
    }
    Wire.beginTransmission(OLED_ADDR_SECONDARY);
    if (Wire.endTransmission() == 0) {
      return OLED_ADDR_SECONDARY;
    }
    return 0;
  }

  void command(uint8_t value) {
    Wire.beginTransmission(address);
    Wire.write(0x00);
    Wire.write(value);
    Wire.endTransmission();
  }

  void data(uint8_t value) {
    Wire.beginTransmission(address);
    Wire.write(0x40);
    Wire.write(value);
    Wire.endTransmission();
  }

  void setCursor(uint8_t x, uint8_t page) {
    command(0xB0 | (page & 0x07));
    command(0x00 | (x & 0x0F));
    command(0x10 | (x >> 4));
  }

  void drawChar(char c) {
    if (c < 32 || c > 126) {
      c = ' ';
    }
    uint16_t offset = (uint16_t)(c - 32) * 5;
    for (uint8_t i = 0; i < 5; ++i) {
      data(pgm_read_byte(&FONT_5X7[offset + i]));
    }
    data(0x00);
    data(0x00);
    data(0x00);
  }
};

TinySSD1306 oled;

// -------------------------- Helpers ----------------------------
void printI2CScan() {
  Serial.println(F("I2C scan:"));
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; ++addr) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("  0x"));
      if (addr < 16) {
        Serial.print('0');
      }
      Serial.println(addr, HEX);
      found++;
    }
  }

  if (found == 0) {
    Serial.println(F("  no I2C devices found"));
  }
}

void setAllLidarsOff() {
  pinMode(LIDAR_LEFT_XSHUT, OUTPUT);
  pinMode(LIDAR_FRONT_XSHUT, OUTPUT);
  pinMode(LIDAR_RIGHT_XSHUT, OUTPUT);

  digitalWrite(LIDAR_LEFT_XSHUT, LOW);
  digitalWrite(LIDAR_FRONT_XSHUT, LOW);
  digitalWrite(LIDAR_RIGHT_XSHUT, LOW);
  delay(20);
}

bool startLidar(VL6180X &sensor, uint8_t xshutPin, uint8_t newAddress, const __FlashStringHelper *name) {
  digitalWrite(xshutPin, HIGH);
  delay(50);

  sensor.setTimeout(80);
  sensor.init();
  sensor.configureDefault();
  sensor.setAddress(newAddress);
  delay(10);

  Serial.print(name);
  Serial.print(F(" VL6180X ready @0x"));
  Serial.println(newAddress, HEX);
  return true;
}

uint16_t readLidarMm(VL6180X &sensor, bool ok) {
  if (!ok) {
    return 9999;
  }

  uint16_t mm = sensor.readRangeSingleMillimeters();
  if (sensor.timeoutOccurred()) {
    return 9998;
  }

  return mm;
}

void drawStatusLine(uint8_t row, const char *label, bool ok) {
  char buf[17];
  snprintf(buf, sizeof(buf), "%-5s %s", label, ok ? "OK" : "FAIL");
  oled.drawString(0, row, buf);
}

void drawCleanString(uint8_t col, uint8_t row, const char *text) {
  oled.clearRow(row);
  oled.drawString(col, row, text);
}

void drawLayoutOnce() {
  if (layoutDrawn) {
    return;
  }

  oled.clear();
  oled.drawString(0, 0, "MTRN3100 SENS");
  oled.drawString(0, 1, "X:");
  oled.drawString(0, 2, "Y:");
  oled.drawString(0, 3, "Z:");
  oled.drawString(0, 5, "L:");
  oled.drawString(0, 6, "F:");
  oled.drawString(0, 7, "R:");
  layoutDrawn = true;
}

void drawReadings(float ax, float ay, float az, uint16_t leftMm, uint16_t frontMm, uint16_t rightMm) {
  char buf[17];
  char value[9];

  drawLayoutOnce();

  dtostrf(ax, 7, 1, value);
  snprintf(buf, sizeof(buf), "%s deg", value);
  drawCleanString(2, 1, buf);
  dtostrf(ay, 7, 1, value);
  snprintf(buf, sizeof(buf), "%s deg", value);
  drawCleanString(2, 2, buf);
  dtostrf(az, 7, 1, value);
  snprintf(buf, sizeof(buf), "%s deg", value);
  drawCleanString(2, 3, buf);

  snprintf(buf, sizeof(buf), "%4u mm", leftMm);
  drawCleanString(2, 5, buf);
  snprintf(buf, sizeof(buf), "%4u mm", frontMm);
  drawCleanString(2, 6, buf);
  snprintf(buf, sizeof(buf), "%4u mm", rightMm);
  drawCleanString(2, 7, buf);
}

// --------------------------- Setup -----------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("MTRN3100 sensor display diagnostic"));

  Wire.begin();
  Wire.setClock(100000);

  oledOk = oled.begin();
  if (!oledOk) {
    Serial.println(F("OLED did not ACK at 0x3C or 0x3D; trying 0x3C anyway."));
  } else {
    Serial.print(F("OLED ready @0x"));
    Serial.println(oled.i2cAddress(), HEX);
  }
  oled.clear();
  oled.drawString(0, 0, "OLED starting");

  setAllLidarsOff();

  byte mpuStatus = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(mpuStatus);
  imuOk = (mpuStatus == 0);

  oled.clear();
  oled.drawString(0, 0, "Init sensors");
  drawStatusLine(1, "OLED", oledOk);
  drawStatusLine(2, "IMU", imuOk);

  if (imuOk) {
    oled.drawString(0, 4, "Keep still...");
    Serial.println(F("Calculating MPU6050 offsets; keep robot still."));
    delay(1000);
    mpu.calcOffsets(true, true);
    Serial.println(F("MPU6050 calibration done."));
  }

  lidarLeftOk = startLidar(lidarLeft, LIDAR_LEFT_XSHUT, ADDR_LEFT, F("Left"));
  lidarFrontOk = startLidar(lidarFront, LIDAR_FRONT_XSHUT, ADDR_FRONT, F("Front"));
  lidarRightOk = startLidar(lidarRight, LIDAR_RIGHT_XSHUT, ADDR_RIGHT, F("Right"));

  printI2CScan();

  oled.clear();
  drawStatusLine(0, "OLED", oledOk);
  drawStatusLine(1, "IMU", imuOk);
  drawStatusLine(2, "LEFT", lidarLeftOk);
  drawStatusLine(3, "FRONT", lidarFrontOk);
  drawStatusLine(4, "RIGHT", lidarRightOk);
  oled.drawString(0, 7, "Starting...");
  delay(1200);
}

// ---------------------------- Loop -----------------------------
void loop() {
  if (imuOk) {
    mpu.update();
  }

  unsigned long now = millis();
  if (now - lastDisplayMs >= 250) {
    lastDisplayMs = now;

    float ax = imuOk ? mpu.getAngleX() : 0.0f;
    float ay = imuOk ? mpu.getAngleY() : 0.0f;
    float az = imuOk ? mpu.getAngleZ() : 0.0f;

    uint16_t leftMm = readLidarMm(lidarLeft, lidarLeftOk);
    uint16_t frontMm = readLidarMm(lidarFront, lidarFrontOk);
    uint16_t rightMm = readLidarMm(lidarRight, lidarRightOk);

    drawReadings(ax, ay, az, leftMm, frontMm, rightMm);

    if (now - lastSerialMs >= 600) {
      lastSerialMs = now;
      Serial.print(F("ang X/Y/Z = "));
      Serial.print(ax, 1);
      Serial.print(F(", "));
      Serial.print(ay, 1);
      Serial.print(F(", "));
      Serial.print(az, 1);
      Serial.print(F(" deg | lidar L/F/R = "));
      Serial.print(leftMm);
      Serial.print(F(", "));
      Serial.print(frontMm);
      Serial.print(F(", "));
      Serial.print(rightMm);
      Serial.println(F(" mm"));
    }
  }
}
