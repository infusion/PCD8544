#include <PCD8544.h>

#include "fonts.h"

PCD8544 display(7, 6, 5, 4, 3);

void setup() {

  display.init();
  display.setFont(YOUR_FONT);
}

void loop() {

  display.clearBuffer();
  
  display.print("Hello World!", 10, 20);
  
  display.draw();
  
  delay(500);
}

