
#include <PCD8544.h>

PCD8544 display(7, 6, 5, 4, 3);

void setup() {
  display.init();
}

float pos = 0;
void loop() {
  
  display.clearBuffer();

  // Coordinates
  display.strokeRect(0, 0, 83, 47);
  display.strokeLine(0, 23, 84, 23);
  display.strokeLine(41, 0, 41, 47);

  for (uint8_t i = 0; i < PCD8544_SCREEN_WIDTH; i++) {
    display.setPixel(i, 20 * sin(5 * (i * PI / 180 + pos)) + 23);
  }
  pos+= PI / 300.0;
  display.update();
  delay(50);
}
