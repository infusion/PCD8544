
#include "PCD8544.h"

void PCD8544::init(uint8_t contrast, uint8_t bias, uint8_t tempCoeff) {

    if (_hasHardwareSPI()) {
        // Setup hardware SPI.
        SPI.begin();
        SPI.setClockDivider(PCD8544_SPI_CLOCK_DIV);
        SPI.setDataMode(SPI_MODE0);
        SPI.setBitOrder(MSBFIRST);
    } else {
        // Setup software SPI.

        pinMode(_din, OUTPUT);
        pinMode(_sclk, OUTPUT);

        // Set software SPI ports and masks
        P_DIN = portOutputRegister(digitalPinToPort(_din));
        B_DIN = digitalPinToBitMask(_din);
        P_SCLK = portOutputRegister(digitalPinToPort(_sclk));
        B_SCLK = digitalPinToBitMask(_sclk);
    }

    P_RST = portOutputRegister(digitalPinToPort(_rst));
    B_RST = digitalPinToBitMask(_rst);
    P_SCE = portOutputRegister(digitalPinToPort(_sce));
    B_SCE = digitalPinToBitMask(_sce);
    P_DC = portOutputRegister(digitalPinToPort(_dc));
    B_DC = digitalPinToBitMask(_dc);

    pinMode(_dc, OUTPUT);
    pinMode(_rst, OUTPUT);

    if (_sce > 0)
        pinMode(_sce, OUTPUT);

    // toggle RST low to reset LCD to a known state
    resetLCD;

    // Set presets 
    _command(PCD8544_FUNCTIONSET | PCD8544_EXTENDED_INSTRUCTION | PCD8544_ADDRESSING);
    _command(PCD8544_SETVOP | contrast); // Set Vop
    _command(PCD8544_SETTEMP | tempCoeff); // 0x00, 0x04?
    _command(PCD8544_SETBIAS | bias); // 0x13 or 0x14: 1:48 or 1:65?

    // Set display to Normal
    _command(PCD8544_FUNCTIONSET | PCD8544_BASIC_INSTRUCTION | PCD8544_ADDRESSING);
    _command(PCD8544_DISPLAYCONTROL | PCD8544_DISPLAY_NORMAL);

    _updateBoundingBox(0, 0, PCD8544_SCREEN_WIDTH - 1, PCD8544_SCREEN_HEIGHT - 1);

    // Clear Screen
    draw();
}

void PCD8544::setContrast(uint8_t contrast) {

    if (contrast > 0x7F) {
        contrast = 0x7F;
    }

    _command(PCD8544_FUNCTIONSET | PCD8544_EXTENDED_INSTRUCTION | PCD8544_ADDRESSING); // enter extended instructions
    _command(PCD8544_SETVOP | contrast);
    _command(PCD8544_FUNCTIONSET | PCD8544_BASIC_INSTRUCTION | PCD8544_ADDRESSING); // enter normal instructions
}

void PCD8544::setBias(uint8_t bias) {
    _command(PCD8544_FUNCTIONSET | PCD8544_EXTENDED_INSTRUCTION | PCD8544_ADDRESSING); // enter extended instructions
    _command(PCD8544_SETBIAS | bias);
    _command(PCD8544_FUNCTIONSET | PCD8544_BASIC_INSTRUCTION | PCD8544_ADDRESSING); // enter normal instructions
}

void PCD8544::setTempCoeff(uint8_t temp) {
    _command(PCD8544_FUNCTIONSET | PCD8544_EXTENDED_INSTRUCTION | PCD8544_ADDRESSING); // enter extended instructions
    _command(PCD8544_SETTEMP | temp);
    _command(PCD8544_FUNCTIONSET | PCD8544_BASIC_INSTRUCTION | PCD8544_ADDRESSING); // enter normal instructions
}

void PCD8544::setDisplayMode(pcd8544_display_t mode) {
    // In basic instruction mode!
    _command(PCD8544_DISPLAYCONTROL | mode);
}

void PCD8544::setPixel(uint8_t x, uint8_t y, uint8_t color) {

    if (x >= PCD8544_SCREEN_WIDTH || y >= PCD8544_SCREEN_HEIGHT)
        return;

    if (color == 1)
        PCD8544_BUFFER(x, y)|= (1 << y % 8);
    else if (color == 0)
        PCD8544_BUFFER(x, y)&=~(1 << y % 8);
    else
        PCD8544_BUFFER(x, y)^= (1 << y % 8);

    _updateBoundingBox(x, y, x, y);
}

uint8_t PCD8544::getPixel(uint8_t x, uint8_t y) {

    if (x >= PCD8544_SCREEN_WIDTH || y >= PCD8544_SCREEN_HEIGHT)
        return 0;

    return (PCD8544_BUFFER(x, y) >> (y % 8)) % 2;
}

void PCD8544::strokeRect(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color) {

    if (x1 > x2) {
        uint8_t t = x2;
        x2 = x1;
        x1 = t;
    }

    if (y1 > y2) {
        uint8_t t = y2;
        y2 = y1;
        y1 = t;
    }

    if (color == 1) {

        for (uint8_t x = x1; x <= x2 && x < PCD8544_SCREEN_WIDTH; x++) {
            PCD8544_BUFFER(x, y1)|= (1 << y1 % 8);
            PCD8544_BUFFER(x, y2)|= (1 << y2 % 8);
        }

        for (uint8_t y = y1 + 1; y < y2 && y < PCD8544_SCREEN_HEIGHT; y++) {
            PCD8544_BUFFER(x1, y)|= (1 << y % 8);
            PCD8544_BUFFER(x2, y)|= (1 << y % 8);
        }

    } else if (color == 0) {

        for (uint8_t x = x1; x <= x2 && x < PCD8544_SCREEN_WIDTH; x++) {
            PCD8544_BUFFER(x, y1)&= ~(1 << y1 % 8);
            PCD8544_BUFFER(x, y2)&= ~(1 << y2 % 8);
        }

        for (uint8_t y = y1 + 1; y < y2 && y < PCD8544_SCREEN_HEIGHT; y++) {
            PCD8544_BUFFER(x1, y)&= ~(1 << y % 8);
            PCD8544_BUFFER(x2, y)&= ~(1 << y % 8);
        }

    } else {

        for (uint8_t x = x1; x <= x2 && x < PCD8544_SCREEN_WIDTH; x++) {
            PCD8544_BUFFER(x, y1)^= (1 << y1 % 8);
            PCD8544_BUFFER(x, y2)^= (1 << y2 % 8);
        }

        for (uint8_t y = y1 + 1; y < y2 && y < PCD8544_SCREEN_HEIGHT; y++) {
            PCD8544_BUFFER(x1, y)^= (1 << y % 8);
            PCD8544_BUFFER(x2, y)^= (1 << y % 8);
        }
    }

    _updateBoundingBox(x1, y1, x2, y2);
}

void PCD8544::strokeCircle(uint8_t x0, uint8_t y0, uint8_t radius, uint8_t color) {

    // https://en.wikipedia.org/wiki/Midpoint_circle_algorithm

    int8_t x = radius - 1;
    int8_t y = 0;
    int8_t dx = 1;
    int8_t dy = 1;
    int8_t err = dx - (radius << 1);

    while (x >= y) {
        setPixel(x0 + x, y0 + y, color);
        setPixel(x0 + y, y0 + x, color);
        setPixel(x0 - y, y0 + x, color);
        setPixel(x0 - x, y0 + y, color);
        setPixel(x0 - x, y0 - y, color);
        setPixel(x0 - y, y0 - x, color);
        setPixel(x0 + y, y0 - x, color);
        setPixel(x0 + x, y0 - y, color);

        if (err <= 0) {
            y++;
            err += dy;
            dy += 2;
        }
        if (err > 0) {
            x--;
            dx += 2;
            err += dx - (radius << 1);
        }
    }
}

void PCD8544::strokeLine(int8_t x1, int8_t y1, int8_t x2, int8_t y2, uint8_t color) {

    // Bresenham's Algorithm

    int8_t dx = x2 - x1, sx = x1 < x2 ? 1 : -1;
    int8_t dy = y2 - y1, sy = y1 < y2 ? 1 : -1;

    if (dx < 0) dx = -dx;
    if (dy < 0) dy = -dy;

    int8_t ert, err = (dx > dy ? dx : -dy) / 2;

    for (;;) {
        setPixel(x1, y1, color);
        if (x1 == x2 && y1 == y2) break;
        ert = err;
        if (ert >-dx) {
            err -= dy;
            x1 += sx;
        }
        if (ert < dy) {
            err += dx;
            y1 += sy;
        }
    }
}

void PCD8544::draw() {

#ifdef PCD8544_USE_BOUNDINGBOX

    for (uint8_t y = boundMinY; y <= boundMaxY; y += 8) {

        uint8_t l = y / 8;

        _command(PCD8544_SETYADDR | l);
        _command(PCD8544_SETXADDR | boundMinX);

        _start_data();
        for (uint8_t x = boundMinX; x <= boundMaxX; x++) {
            _write_data(PCD8544_BUFFER_LINE(x, l));
        }
        _end_data();
    }

    // Invalidate next draw() call
    boundMinY = 255;
    boundMaxY = 0;

#else
    _command(PCD8544_SETYADDR);
    _command(PCD8544_SETXADDR);

    _start_data();
    for (uint16_t i = 0; i < PCD8544_BUFFER_LEN; i++) {
        _write_data(buffer[i]);
    }
    _end_data();
#endif
}









void PCD8544::drawImage(const uint8_t *image) {

    _command(PCD8544_SETYADDR);
    _command(PCD8544_SETXADDR);

    _start_data();
    for (uint16_t i = 0; i < PCD8544_BUFFER_LEN; i++) {
        _write_data(GETPROGBYTE(image, i));
    }
    _end_data();
}

void PCD8544::clearBuffer() {

    for (uint16_t i = 0; i < PCD8544_BUFFER_LEN; i++) {
        buffer[i] = 0;
    }
    _updateBoundingBox(0, 0, PCD8544_SCREEN_WIDTH - 1, PCD8544_SCREEN_HEIGHT - 1);
}

void PCD8544::clear() {

    _command(PCD8544_SETYADDR);
    _command(PCD8544_SETXADDR);

    _start_data();
    for (uint16_t i = 0; i < PCD8544_BUFFER_LEN; i++) {
        buffer[i] = 0;
        _write_data(0);
    }
    _end_data();

#ifdef PCD8544_USE_BOUNDINGBOX
    // Invalidate next display() call
    boundMinY = 255;
    boundMaxY = 0;
#endif
}


// Private methods

inline void PCD8544::_start_data() {

    sbi(P_DC, B_DC);

    if (_sce > 0) {
        cbi(P_SCE, B_SCE); // SCE LOW, start transmission
    }
}

inline void PCD8544::_write_data(uint8_t data) {

    if (_hasHardwareSPI()) {
        // Hardware SPI
        SPI.transfer(data);
    } else {
        // Software SPI
        for (uint8_t c = 128; c; c >>= 1) {
            cbi(P_SCLK, B_SCLK);

            if (data & c)
                sbi(P_DIN, B_DIN);
            else
                cbi(P_DIN, B_DIN);

            sbi(P_SCLK, B_SCLK);
        }
    }
}

inline void PCD8544::_end_data() {

    if (_sce > 0) {
        sbi(P_SCE, B_SCE); // SCE HIGH, end transmission 
    }
}

inline void PCD8544::_command(uint8_t data) {

    cbi(P_DC, B_DC); // send command

    if (_sce > 0) {
        cbi(P_SCE, B_SCE); // SCE LOW, start transmission
    }

    // Write byte
    _write_data(data);

    if (_sce > 0) {
        sbi(P_SCE, B_SCE); // SCE HIGH, end transmission
    }
}

inline bool PCD8544::_hasHardwareSPI() {
    return _din == -1 && _sclk == -1;
}

inline void PCD8544::_updateBoundingBox(uint8_t xmin, uint8_t ymin, uint8_t xmax, uint8_t ymax) {
#ifdef PCD8544_USE_BOUNDINGBOX
    if (xmin < boundMinX) boundMinX = xmin;
    if (xmax > boundMaxX) boundMaxX = xmax;
    if (ymin < boundMinY) boundMinY = ymin;
    if (ymax > boundMaxY) boundMaxY = ymax;
#endif
}