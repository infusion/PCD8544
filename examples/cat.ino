
#include <PCD8544.h>

PCD8544 display(7, 6, 5, 4, 3);

// Generated with https://www.xarg.org/tools/lcd-image-generator/
const PROGMEM uint8_t image[] = {
0xb5, 0x4a, 0xb5, 0x4a, 0xb5, 0x4a, 0xb5, 0x4b, 0x15, 0xe0, 0x14, 0xea, 0x10, 0xe4, 0x0a, 0xe0, 
0x0a, 0xe0, 0x08, 0xa2, 0x48, 0xa0, 0x05, 0x50, 0x03, 0xa4, 0x03, 0x0c, 0x9b, 0x26, 0xd9, 0x77, 
0x8c, 0x3b, 0xf7, 0x5e, 0xfd, 0xff, 0xbd, 0x43, 0xbe, 0xf5, 0xff, 0xff, 0xb5, 0xff, 0x7f, 0xf5, 
0x5f, 0xff, 0xfd, 0xff, 0x7a, 0x95, 0xff, 0xfd, 0xff, 0x6d, 0xfa, 0x17, 0xfd, 0x83, 0x7e, 0x29, 
0x17, 0x08, 0x4f, 0x05, 0x02, 0x53, 0x81, 0x29, 0x80, 0x51, 0x80, 0x24, 0x40, 0xaa, 0x00, 0xa4, 
0x50, 0x80, 0x34, 0x01, 0x1b, 0xa4, 0x5b, 0x84, 0x7b, 0x04, 0xeb, 0x14, 0xa2, 0x14, 0x49, 0x32, 
0x4d, 0xb2, 0x4d, 0x92, 0x7d, 0xa4, 0x59, 0xa6, 0x59, 0xa4, 0x19, 0xe7, 0x08, 0x52, 0x05, 0x88, 
0x20, 0x02, 0x50, 0x05, 0xa0, 0x0a, 0x55, 0x05, 0xfb, 0xdf, 0xff, 0xbf, 0x46, 0xd9, 0xa7, 0x5d, 
0xf6, 0xed, 0xbb, 0x55, 0xeb, 0xbf, 0x42, 0xfd, 0x07, 0xfc, 0x7f, 0xff, 0xbf, 0x45, 0x12, 0xa9, 
0x42, 0x14, 0x02, 0x48, 0x00, 0x22, 0x89, 0x24, 0x01, 0x6c, 0x82, 0x28, 0xd2, 0x04, 0x7a, 0x81, 
0x2e, 0xd0, 0x0b, 0x64, 0x09, 0x52, 0x05, 0xf0, 0xaa, 0x55, 0xaa, 0x50, 0x87, 0x58, 0x22, 0x4d, 
0xb2, 0x04, 0xa1, 0x0a, 0x40, 0x09, 0x92, 0x25, 0x5e, 0x31, 0x5f, 0x20, 0x0b, 0x94, 0x03, 0x28, 
0x05, 0x52, 0x00, 0x54, 0x00, 0xa9, 0x02, 0x44, 0x10, 0xa5, 0x00, 0x2b, 0x40, 0xab, 0x53, 0x84, 
0x0b, 0x37, 0xfd, 0xff, 0xd7, 0xfe, 0xfb, 0xff, 0x6f, 0xf4, 0xbb, 0x1e, 0x83, 0x28, 0x43, 0xc8, 
0x12, 0x41, 0x14, 0x42, 0x00, 0xd5, 0x00, 0x2a, 0x81, 0x54, 0x00, 0x24, 0x01, 0x2a, 0x80, 0x15, 
0x22, 0x5d, 0xb2, 0x4d, 0x19, 0x42, 0x14, 0x01, 0x4a, 0x00, 0x55, 0xaa, 0xac, 0x53, 0xac, 0x51, 
0xa6, 0x59, 0xa2, 0x19, 0xa2, 0x4c, 0x21, 0x4a, 0x04, 0x50, 0x02, 0xa0, 0x49, 0x90, 0xc2, 0xa8, 
0x80, 0xd0, 0x82, 0x80, 0xa9, 0x00, 0x44, 0x81, 0x52, 0xa4, 0xc9, 0xd1, 0xea, 0xc4, 0xd0, 0xe1, 
0xd6, 0xc9, 0x16, 0x28, 0x41, 0x88, 0x12, 0x65, 0x8b, 0x16, 0x6b, 0x97, 0x43, 0xbd, 0x02, 0x50, 
0x04, 0xa9, 0x92, 0xcd, 0xe2, 0xc1, 0xc8, 0xd5, 0xc0, 0x96, 0xa1, 0x0a, 0xa0, 0x04, 0x90, 0x01, 
0x80, 0x24, 0xc0, 0xc8, 0xa2, 0x84, 0xa0, 0x0a, 0xa0, 0x04, 0xa0, 0x09, 0xf6, 0x09, 0xf6, 0xa9, 
0xa8, 0x57, 0xb8, 0x47, 0xb8, 0x47, 0xb8, 0x43, 0xbc, 0x43, 0xac, 0x51, 0xaa, 0x55, 0xa8, 0xd2, 
0x7d, 0x9b, 0x6f, 0x93, 0x6f, 0x95, 0x2b, 0xd4, 0x69, 0x81, 0x72, 0x84, 0x49, 0x9f, 0x3f, 0xbf, 
0x7f, 0x7f, 0x7d, 0x3f, 0xbf, 0x3f, 0xbf, 0x7d, 0x52, 0x84, 0x10, 0xe5, 0x08, 0x61, 0x0a, 0xd4, 
0x22, 0xd4, 0x08, 0xa1, 0x52, 0x3e, 0xbf, 0x3f, 0x7f, 0x7f, 0x7d, 0x7f, 0x7f, 0xbf, 0x3f, 0x89, 
0x52, 0x80, 0x55, 0xa1, 0x29, 0x57, 0x8b, 0x53, 0x2f, 0x5e, 0x9e, 0x7a, 0xd4, 0x21, 0xde, 0x20, 
0xdd, 0x23, 0xdc, 0x2b, 0x59, 0xb7, 0xc8, 0x37, 0xec, 0x93, 0x7c, 0xa3, 0xdc, 0x33, 0xcc, 0xb3, 
0x4c, 0xb1, 0x4e, 0xb1, 0x4e, 0x99, 0x76, 0x88, 0x77, 0x88, 0x6f, 0x52, 0x8d, 0x71, 0x07, 0xa9, 
0x1f, 0xa6, 0x0d, 0x2e, 0x91, 0x0e, 0x11, 0xa6, 0x18, 0xa2, 0x44, 0x09, 0x26, 0xff, 0xfa, 0xf7, 
0xe8, 0xd7, 0xe8, 0xd6, 0xe9, 0xf6, 0xeb, 0x0e, 0x51, 0x42, 0x2c, 0x20, 0x55, 0x02, 0x2d, 0x02, 
0x5e, 0x06, 0x9b, 0x2c, 0x47, 0x1a, 0x45, 0x9a, 0x29, 0x96, 0x2c, 0xd9, 0x2a, 0x55, 0xac, 0x51, 
0xae, 0x51, 0xae, 0x51, 0xae, 0x51, 0xac, 0x53};

void setup() {

  display.init();
}

void loop() {

  display.updateImage(image);

  delay(2000);
  display.setDisplayMode(PCD8544_DISPLAY_INVERTED);

  delay(1000);
  display.setDisplayMode(PCD8544_DISPLAY_NORMAL);
}
