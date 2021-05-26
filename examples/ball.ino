#include <PCD8544.h>

PCD8544 display(7, 6, 5, 4, 3);

float radius = 6;
float x = random(radius, PCD8544_SCREEN_WIDTH - 2 * radius);
float y = random(radius, PCD8544_SCREEN_HEIGHT - 2 * radius);
float dx = random(0, 10) / 5.0 - 1.;
float dy = random(0, 10) / 5.0 - 1.;

void setup() {

  display.init();
}

void loop() {

  display.clearBuffer();

  x+= dx;
  y+= dy;

  if (x >= PCD8544_SCREEN_WIDTH - radius) {
    x = PCD8544_SCREEN_WIDTH - radius;
    dx = -dx;
  } else if (x <= radius) {
    x = radius;
    dx = -dx;
  }

  if (y >= PCD8544_SCREEN_HEIGHT - radius) {
    y = PCD8544_SCREEN_HEIGHT - radius;
    dy = -dy;
  } else if (y <= radius) {
    y = radius;
    dy = -dy;
  }
 
  display.fillCircle(x, y, radius, 1);
  display.update();

  delay(50);
}
