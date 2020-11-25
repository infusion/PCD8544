#include <PCD8544.h>

PCD8544 display(7, 6, 5, 4, 3);

float radius = 6;
float x = random(R, 84 - 2 * radius);
float y = random(R, 48 - 2 * radius);
float dx = random(0, 10) / 10.0 - 0.5;
float dy = random(0, 10) / 10.0 - 0.5;

void setup() {

  display.init();
}

void loop() {

  display.clearBuffer();

  x+= dx;
  y+= dy;

  if (x >= 84 - radius) {
    X = 84 - radius;
    dx = -dx;
  } else if (x <= radius) {
    x = radius;
    dx = -dx;
  }

  if (y >= 48 - radius) {
    y = 48 - radius;
    dy = -dy;
  } else if (y <= radius) {
    y = radius;
    dy = -dy;
  }
 
  display.strokeCircle(x, y, radius, 1);
  display.draw();

  delay(50);
}

