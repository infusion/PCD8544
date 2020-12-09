
#include <PCD8544.h>

#include "fonts.h"

PCD8544 display(7, 6, 5, 4, 3);

String text = "You are great!";

void setup() {
  display.init();
  display.setFont(font);
}

int16_t pos = 0;

void loop() {

  display.clearBuffer();

  uint8_t len = text.length();

  for (uint8_t i = 0; i < len; i++) {
    display.print(text[i], i * 6, (24 - 8) + 10.0 * sin((i + pos) * PI / 20.0));
  }
  
  display.update();
  pos++;
  delay(50);
}
