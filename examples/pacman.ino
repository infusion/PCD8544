
#include <PCD8544.h>

#include "fonts.h"

PCD8544 display(7, 6, 5, 4, 3);

void setup() {
  display.init();
  display.setFont(font);
}

// Pacman size + position
const uint8_t X = 15;
const uint8_t Y = 24;
const uint8_t R = 15;

const float maxAlpha = 40 / 180.0 * PI; // Max mouth open angle
float alpha = 0;
float da = +.2;

int16_t textPos = 0;
String text = "Welcome on the Nokia 5110 Display ... visit www.xarg.org ;-)";

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

  textPos-= 2;
  
  if (textPos < - (int16_t) (text.length() * 6)) {
    textPos = 84;
  }

  display.print(text, textPos, Y - 4, FONT_MODE_TRANSPARENT);

  display.draw(); 

  delay(100);
}

