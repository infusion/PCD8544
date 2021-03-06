/**
 * Philips PCD8544 LCD Driver v0.0.1 08/06/2014
 * https://www.xarg.org/2014/06/how-to-use-a-nokia-5110-graphical-display-with-arduino/
 *
 * Copyright (c) 2014, Robert Eisele (robert@xarg.org)
 * Dual licensed under the MIT or GPL Version 2 licenses.
 **/

#ifndef _PCD8544_H
#define _PCD8544_H

#if defined(ARDUINO_ARCH_SAMD) || defined(__SAM3X8E__)

// Arduino Due

#define PROGMEM

#include "Arduino.h"

#define GETPROGBYTE(data, x)   data[x]

#define cbi(reg, bitmask) *reg&=~bitmask
#define sbi(reg, bitmask) *reg|= bitmask

typedef volatile uint32_t PinReg;
typedef uint32_t PinMask;

#elif defined(ARDUINO_ARCH_AVR) ||    \
    defined(__AVR_ATmega328P__) ||  \
    defined(__AVR_ATmega1280__) ||  \
    defined(__AVR_ATmega2560__) ||  \
    defined(__AVR_ATmega32U4__)

// Arduino Uno
// Arduino Duemilanove
// Arduino Mega 2560
// Arduino Leonardo
// Arduino Decimilia and older

#ifdef __AVR__
#include <avr/pgmspace.h>
#endif

#include "Arduino.h"

#include <SPI.h>

#define GETPROGBYTE(data, x) pgm_read_byte(data + x)

#define cbi(reg, bitmask) *reg&=~bitmask
#define sbi(reg, bitmask) *reg|= bitmask

typedef volatile uint8_t PinReg;
typedef uint8_t PinMask;

#elif defined(__PIC32MX__)

// PIC32

#define PROGMEM

#include "WProgram.h"

#define GETPROGBYTE(data, x)   data[x]

#define cbi(reg, bitmask) (*(reg + 1)) = bitmask
#define sbi(reg, bitmask) (*(reg + 2)) = bitmask

typedef volatile uint32_t PortReg;
typedef uint8_t PortMask;

#else
typedef volatile uint32_t PinReg;
typedef uint32_t PinMask;
#endif

#ifndef _BV
#define _BV(x) (1 << (x))
#endif


#define PCD8544_SCREEN_WIDTH           84
#define PCD8544_SCREEN_HEIGHT          48

#define PCD8544_BUFFER_LEN             (PCD8544_SCREEN_WIDTH * PCD8544_SCREEN_HEIGHT / 8)

#define PCD8544_BASIC_INSTRUCTION      0x00
#define PCD8544_EXTENDED_INSTRUCTION   0x01

#define PCD8544_HORIZONTAL_ADDRESS     0x00
#define PCD8544_VERTICAL_ADDRESS       0x02

#define PCD8544_ADDRESSING             PCD8544_HORIZONTAL_ADDRESS

#define PCD8544_POWER_UP               0x00
#define PCD8544_POWER_DOWN             0x04

#define PCD8544_FUNCTIONSET            0x20
#define PCD8544_DISPLAYCONTROL         0x08
#define PCD8544_SETYADDR               0x40
#define PCD8544_SETXADDR               0x80

#define PCD8544_SETTEMP                0x04
#define PCD8544_SETBIAS                0x10
#define PCD8544_SETVOP                 0x80

#define PCD8544_SPI_CLOCK_DIV          SPI_CLOCK_DIV4

#define PCD8544_BUFFER(x, y)           PCD8544_BUFFER_LINE(x, (y / 8))
#define PCD8544_BUFFER_LINE(x, l)      buffer[(uint16_t(l) * PCD8544_SCREEN_WIDTH + x)]

typedef enum {
    PCD8544_DISPLAY_BLANK = 0,
    PCD8544_DISPLAY_NORMAL = 4,
    PCD8544_DISPLAY_ALL_ON = 1,
    PCD8544_DISPLAY_INVERTED = 5
} pcd8544_display_t;

typedef enum {
    FONT_MODE_DEFAULT = 0,
    FONT_MODE_TRANSPARENT = 1,
    FONT_MODE_INVERTED = 2,
    FONT_MODE_AUTO_LINEBREAK = 4,
    FONT_MODE_IGNORE_NEWLINE = 8
} pcd8544_fontmode_t;

class PCD8544 {

private:

    // Pin numbers
    int8_t _rst, _sce, _dc, _din, _sclk;

    // Pin references + masks (unwrapped digitalWrite() function for speed)
    PinReg *P_RST, *P_SCE, *P_DC, *P_DIN, *P_SCLK;
    PinMask B_RST, B_SCE, B_DC, B_DIN, B_SCLK;

    // TODO: ship a funny default image
    uint8_t buffer[PCD8544_SCREEN_WIDTH * PCD8544_SCREEN_HEIGHT / 8] = {0};

    uint8_t *_font;

    inline void _start_data();
    inline void _write_data(uint8_t data);
    inline void _end_data();
    inline void _command(uint8_t data);

    inline bool _hasHardwareSPI();

public:

    // Software SPI with explicit CS pin
    PCD8544(int8_t rst, int8_t sce, int8_t dc, int8_t din, int8_t sclk) : _rst(rst), _sce(sce), _dc(dc), _din(din), _sclk(sclk), _font(NULL) {
    }

    // Software SPI with CS tied to ground. Saves a pin but the other pins can't be shared with other hardware
    PCD8544(int8_t rst, int8_t dc, int8_t din, int8_t sclk) : _rst(rst), _sce(-1), _dc(dc), _din(din), _sclk(sclk), _font(NULL) {
    }

    // Hardware SPI with hardware controlled SCK (SCLK) and MOSI (DIN). SCE is controlled by IO pin
    PCD8544(int8_t rst, int8_t sce, int8_t dc) : _rst(rst), _sce(sce), _dc(dc), _din(-1), _sclk(-1), _font(NULL) {
    }

    void init(uint8_t contrast = 60, uint8_t bias = 0x03, uint8_t tempCoeff = 0x02);

    // Contrast: 0-127
    void setContrast(uint8_t contrast);

    // Bias: 0-7
    void setBias(uint8_t bias);

    // Temp: 0-3
    void setTempCoeff(uint8_t temp);

    void setPower(bool on);

    void setDisplayMode(pcd8544_display_t mode);

    // Drawing Methods

    void setPixel(uint8_t x, uint8_t y, uint8_t color = 1);

    uint8_t getPixel(uint8_t x, uint8_t y);

    void strokeRect(uint8_t x1, uint8_t y1, uint8_t width, uint8_t height, uint8_t color = 1);

    void fillRect(uint8_t x1, uint8_t y1, uint8_t width, uint8_t height, uint8_t color = 1);

    void strokeCircle(uint8_t x0, uint8_t y0, uint8_t radius, uint8_t color = 1);

    void fillCircle(uint8_t x0, uint8_t y0, uint8_t radius, uint8_t color = 1);

    void strokeLine(int8_t x1, int8_t y1, int8_t x2, int8_t y2, uint8_t color = 1);

    // Text Methods

    void setFont(uint8_t *font);

    void print(char c, int16_t x, int16_t y, pcd8544_fontmode_t mode = FONT_MODE_DEFAULT);

    void print(char *c, int16_t x, int16_t y, pcd8544_fontmode_t mode = FONT_MODE_DEFAULT);

    void print(String st, int16_t x, int16_t y, pcd8544_fontmode_t mode = FONT_MODE_DEFAULT);

    // Invoke Methods

    void update();

    void updateImage(const uint8_t *image);

    void clearBuffer();

    void clear();

};

#endif
