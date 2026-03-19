#include <Wire.h>

#define SDA_PIN 8
#define SCL_PIN 9
#define ST25DV_ADDR 0x53   // User memory (E2=0,E1=0)

void writeBlock(uint16_t block, uint8_t *data)
{
  uint16_t addr = block * 4;

  Wire.beginTransmission(ST25DV_ADDR);
  Wire.write(addr >> 8);
  Wire.write(addr & 0xFF);

  for(int i=0;i<4;i++)
    Wire.write(data[i]);

  Wire.endTransmission();
  delay(6);
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  Serial.println("Formatting ST25DV as NFC Type 5 tag...");

  // ---- BLOCK 0 : Capability Container ----
  uint8_t cc[4] = {0xE1, 0x40, 0x40, 0x00};
  writeBlock(0, cc);

  // ---- BLOCK 1-4 : NDEF URL ----
  uint8_t block1[4] = {0x03, 0x11, 0xD1, 0x01};
  uint8_t block2[4] = {0x0D, 0x55, 0x03, 'd'};
  uint8_t block3[4] = {'a','r','s','h'};
  uint8_t block4[4] = {'.','a','p','p'};
  uint8_t block5[4] = {0xFE,0x00,0x00,0x00};

  writeBlock(1, block1);
  writeBlock(2, block2);
  writeBlock(3, block3);
  writeBlock(4, block4);
  writeBlock(5, block5);

  Serial.println("Done! Tap phone now.");
}

void loop(){}