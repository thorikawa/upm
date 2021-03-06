/*
 * Author: Yevgeniy Kiveisha <yevgeniy.kiveisha@intel.com>
 * Copyright (c) 2014 Intel Corporation.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#pragma once

#include <string>
#include "i2clcd.h"
#include "ssd.h"

namespace upm
{
const uint8_t DISPLAY_CMD_SET_NORMAL_1308 = 0xA6;

/**
 * @library i2clcd
 * @sensor ssd1308
 * @comname SSD1308 OLED Display
 * @altname Grove OLED Display 0.96"
 * @type display
 * @man seeed adafruit
 * @web http://garden.seeedstudio.com/images/4/46/SSD1308_1.0.pdf
 * @web http://www.seeedstudio.com/wiki/Grove_-_OLED_Display_0.96%22
 * @con i2c
 *
 * @brief C++ API for SSD1308 i2c controlled OLED displays
 *
 * The SSD1308 is a 128x64 Dot matrix OLED/PLED segment driver with
 * controller. This implementation was tested using the Grove LED 128×64
 * Display module which is an OLED monochrome display.
 *
 * @image html ssd1308.jpeg
 * @snippet ssd1308-oled.cxx Interesting
 */
class SSD1308 : public I2CLcd
{
  public:
    /**
     * SSD1308 Constructor, calls libmraa initialisation functions
     *
     * @param bus i2c bus to use
     * @param address the slave address the lcd is registered on
     */
    SSD1308(int bus, int address = 0x3C);
    /**
     * SSD1308 Destructor
     */
    ~SSD1308();
    /**
     * Draw an image, see examples/python/make_oled_pic.py for an
     * explanation on how the pixels are mapped to bytes
     *
     * @param data the buffer to read
     * @param bytes the amount of bytes to read from the pointer
     * @return Result of operation
     */
    mraa_result_t draw(uint8_t* data, int bytes);
    /**
     * Write a string to LCD
     *
     * @param msg The std::string to write to display, note only ascii
     *     chars are supported
     * @return Result of operation
     */
    mraa_result_t write(std::string msg);
    /**
     * Set cursor to a coordinate
     *
     * @param row The row to set cursor to
     * @param column The column to set cursor to
     * @return Result of operation
     */
    mraa_result_t setCursor(int row, int column);
    /**
     * Clear display from characters
     *
     * @return Result of operatio
     */
    mraa_result_t clear();
    /**
     * Return to coordinate 0,0
     *
     * @return Result of operation
     */
    mraa_result_t home();

  private:
    mraa_result_t writeChar(uint8_t value);
    mraa_result_t setNormalDisplay();
    mraa_result_t setAddressingMode(displayAddressingMode mode);
};
}
