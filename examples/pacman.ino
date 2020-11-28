
#include <PCD8544.h>

PCD8544 display(7, 6, 5, 4, 3);

void setup() {
  display.init();
}

// Pacman size + position
const uint8_t X = 17;
const uint8_t Y = 24;
const uint8_t R = 15;

float alpha = 0;
float da = +.2;

const float maxAlpha = 40 / 180.0 * PI; // Max mouth open angle

void loop() {
  
  display.clearBuffer();

  // Draw pacman
  display.fillCircle(X, Y, R);

  alpha+= da;

  // Toggle angle of the mouth
  if (alpha >= maxAlpha) {
    alpha = maxAlpha;
    da = -da;
  } else if (alpha < 0) {
    alpha = 0;
    da = -da;
  }

  // Draw mouth
  float slope = alpha; // ~tan(alpha) for [0, 40°]
  float y = 0;
  for (uint8_t x = X; x <= X + R; x++) {
    display.strokeLine(x, Y + y, x, Y - y, 0);
    y+= slope;
  }

  display.draw(); 

  delay(100);
}

