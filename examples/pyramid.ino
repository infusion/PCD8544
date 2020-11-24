
#include <PCD8544.h>

#define SITES 5

PCD8544 display(7, 6, 5, 4, 3);

void setup() {
  display.init();
}

uint16_t rot = 0;
void loop() {

  display.clearBuffer();

  float alpha = rot / 30.0 - PI * 2.0 / SITES;

  float px = PCD8544_SCREEN_WIDTH / 2 + cos(alpha) * 35;
  float py = PCD8544_SCREEN_HEIGHT / 2 + sin(alpha) * 15 + 5;

  for (uint8_t i = 0; i < SITES; i++) {
    alpha = rot / 30.0 + 2.0 * PI * i / SITES;

    float x = PCD8544_SCREEN_WIDTH / 2 + cos(alpha) * 35;
    float y = PCD8544_SCREEN_HEIGHT / 2 + sin(alpha) * 15 + 5;

    display.strokeLine(x, y, px, py, HIGH);
    display.strokeLine(x, y, PCD8544_SCREEN_WIDTH / 2, 0, HIGH);

    px = x;
    py = y;
  }
  rot++;
  display.draw();
  delay(10);
}
