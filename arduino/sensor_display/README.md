# Sensor Display Diagnostic

Open `sensor_display.ino` in Arduino IDE and upload it to the Arduino Nano.

## What It Shows

- MPU6050 angles: X, Y, Z in degrees
- VL6180X distances: left, front, right in millimetres
- Serial Monitor debug at `115200` baud, including an I2C scan

## Libraries

Install these from Arduino Library Manager:

- `MPU6050_light` by rfetick
- `VL6180X` by Pololu

The OLED code is driven directly through `Wire`, so it does not need `U8g2`,
`Adafruit_SSD1306`, or `SSD1306Ascii`.

## Course PCB Wiring From The Schematic

- OLED SSD1306: `A4 SDA`, `A5 SCL`, `+5`, `GND`
- MPU6050: `A4 SDA`, `A5 SCL`, `+5`, `GND`
- Left VL6180X GPIO0/XSHUT: `A0`
- Front VL6180X GPIO0/XSHUT: `A2`
- Right VL6180X GPIO0/XSHUT: `A1`

The three VL6180X sensors all start at I2C address `0x29`, so the sketch turns all of them off first, then enables them one by one and changes their addresses to:

- Left: `0x54`
- Front: `0x56`
- Right: `0x55`

## If The OLED Is Still Dark

1. Check the OLED is plugged into `J13` in the right orientation.
2. Check the board has `+5V` and `GND`; the course OLED is powered from `+5V`.
3. Open Serial Monitor at `115200` and read the I2C scan.
4. If the scan does not show an OLED address, usually `0x3C`, the OLED is not electrically connected.
5. If the scan shows `0x3C` but the screen is dark, the module may not be SSD1306-compatible even though the 2025 schematic labels it as SSD1306.
