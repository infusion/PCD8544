
#include <PCD8544.h>

#define SITES 5

PCD8544 display(7, 6, 5, 4, 3);

void setup() {
  display.init();
}

float rot = 0, px = 0, py = 0;
void loop() {

  display.clearBuffer();

  for (uint8_t i = 0; i <= SITES; i++) {
    float alpha = rot + 2.0 * PI / SITES * i;

    float x = 42 + cos(alpha) * 35;
    float y = 30 + sin(alpha) * 15;

    if (i > 0) {
      display.strokeLine(x, y, px, py, HIGH);
      display.strokeLine(x, y, PCD8544_SCREEN_WIDTH / 2, 0, HIGH);
    }
    px = x;
    py = y;
  }
  rot+= PI / 120.0;
  display.draw();
  delay(10);
}
