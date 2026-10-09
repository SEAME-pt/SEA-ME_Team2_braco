#include "M5Atom.h"

// 5x5, lido linha por linha (índices 0 a 24)
const uint8_t smile[25] = {
  0, 1, 0, 1, 0,
  0, 0, 0, 0, 0,
  1, 0, 0, 0, 1,
  0, 1, 1, 1, 0,
  0, 0, 0, 0, 0
};

const uint32_t COLOR = 0xffff00;  // amarelo

void setup() {
  M5.begin(true, false, true);
  delay(50);
  M5.dis.setBrightness(20);       // 0 a 255, baixo para não ofuscar
}

void loop() {
  for (int i = 0; i < 25; i++) {
    M5.dis.drawpix(i, smile[i] ? COLOR : 0x000000);
  }
  delay(1000);
}
