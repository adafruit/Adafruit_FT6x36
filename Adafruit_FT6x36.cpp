/*!
 * @file Adafruit_FT6x36.cpp
 *
 * @mainpage Adafruit FT6x36 Capacitive Touch Driver
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for Adafruit's FT6x36 driver for the
 * Arduino platform. It is designed specifically to work with the
 * FT6x36/FT6236/FT6336 capacitive touch controller.
 *
 * These chips use I2C to communicate.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section author Author
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.
 * Adapted for FT63x6 based of excellent code from 
 * https://github.com/strange-v/FT6X36
 *
 * @section license License
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#include "Adafruit_FT6x36.h"

/**************************************************************************/
/*!
    @brief  Instantiates a new FT6x36 class
*/
/**************************************************************************/
Adafruit_FT6x36::Adafruit_FT6x36(void) {}

/**************************************************************************/
/*!
    @brief  Setups the I2C interface and hardware, identifies if chip is found
    @param  thresh Optional threshold for touch detection
    @param  wire Pointer to Wire object for I2C communication
    @returns True if chip is found and initialized, false otherwise
*/
/**************************************************************************/
bool Adafruit_FT6x36::begin(uint8_t thresh, TwoWire *wire) {
  if (i2c_dev)
    delete i2c_dev;
  i2c_dev = new Adafruit_I2CDevice(FT6X36_I2C_ADDR, wire);
  if (!i2c_dev->begin())
    return false;

  Adafruit_BusIO_Register vendor_id =
      Adafruit_BusIO_Register(i2c_dev, FT6X36_REG_PANEL_ID, 1);
  if (vendor_id.read() != FT6X36_VENDID)
    return false;

  Adafruit_BusIO_Register chip_id =
      Adafruit_BusIO_Register(i2c_dev, FT6X36_REG_CHIPID, 1);
  uint8_t id = chip_id.read();
  if ((id != FT6206_CHIPID) && (id != FT6236_CHIPID) && (id != FT6336_CHIPID))
    return false;

  Adafruit_BusIO_Register thresh_reg =
      Adafruit_BusIO_Register(i2c_dev, FT6X36_REG_THRESHHOLD, 1);
  thresh_reg.write(thresh);

  Adafruit_BusIO_Register rate_reg =
      Adafruit_BusIO_Register(i2c_dev, FT6X36_REG_TOUCHRATE_ACTIVE, 1);
  rate_reg.write(0x0E);

  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the firmware version
    @returns Version number read from chip
*/
/**************************************************************************/
uint8_t Adafruit_FT6x36::getVersion(void) {
  Adafruit_BusIO_Register version_reg =
      Adafruit_BusIO_Register(i2c_dev, FT6X36_REG_FIRMWARE_VERSION, 1);
  return version_reg.read();
}

/**************************************************************************/
/*!
    @brief  Gets the chip ID
    @returns Chip ID read from FT6X36_REG_CHIPID register
*/
/**************************************************************************/
uint8_t Adafruit_FT6x36::getChipID(void) {
  Adafruit_BusIO_Register chip_id =
      Adafruit_BusIO_Register(i2c_dev, FT6X36_REG_CHIPID, 1);
  return chip_id.read();
}



/**************************************************************************/
/*!
    @brief  Reads all touch data registers in one transaction
    @returns True if read was successful, false otherwise
*/
/**************************************************************************/
bool Adafruit_FT6x36::readData(void) {
  uint8_t data[16];
  
  Adafruit_BusIO_Register touch_array =
      Adafruit_BusIO_Register(i2c_dev, 0x00, 16);
  
  if (!touch_array.read(data, 16)) {
    return false;
  }

  // Process touch points
  for (uint8_t i = 0; i < 2; i++) {
    const uint8_t offset = 6 * i;
    _touchX[i] = ((uint16_t)(data[FT6X36_REG_P1_XH + offset] & 0x0F) << 8) | 
                   data[FT6X36_REG_P1_XL + offset];
    _touchY[i] = ((uint16_t)(data[FT6X36_REG_P1_YH + offset] & 0x0F) << 8) | 
                   data[FT6X36_REG_P1_YL + offset];
    _touchEvent[i] = data[FT6X36_REG_P1_XH + offset] >> 6;
  }
  
  return true;
}

/**************************************************************************/
/*!
    @brief  Returns number of active touch points
    @returns Number of touch points (0-2)
*/
/**************************************************************************/
uint8_t Adafruit_FT6x36::touched(void) {
  if (!readData()) {
    return 0;
  }
  return _touches;
}

/**************************************************************************/
/*!
    @brief  Returns touch point coordinates
    @param  x1 Pointer to store X coordinate of first touch point
    @param  y1 Pointer to store Y coordinate of first touch point
    @param  x2 Optional pointer to store X coordinate of second touch point
    @param  y2 Optional pointer to store Y coordinate of second touch point
*/
/**************************************************************************/
void Adafruit_FT6x36::getTouchPoints(uint16_t *x1, uint16_t *y1, 
                                    uint16_t *x2, uint16_t *y2) {
  if (!readData()) {
    return;
  }
  
  if (_touches > 0) {
    *x1 = _touchX[0];
    *y1 = _touchY[0];
  }
  
  if (_touches > 1 && x2 && y2) {
    *x2 = _touchX[1];
    *y2 = _touchY[1];
  }
}

/**************************************************************************/
/*!
    @brief  Gets the last touch event type for a given touch point
    @param  touch_num Touch point index (0 or 1)
    @returns Touch event type as FT6X36_RawEvent
*/
/**************************************************************************/
FT6X36_RawEvent Adafruit_FT6x36::getLastTouchEvent(uint8_t touch_num) {
  if (touch_num > 1) return FT6X36_RawEvent::NO_EVENT;
  return static_cast<FT6X36_RawEvent>(_touchEvent[touch_num]);
}


/**************************************************************************/
/*!
    @brief  Gets the touch sensitivity threshold
    @returns Touch threshold value
*/
/**************************************************************************/
uint8_t Adafruit_FT6x36::getThreshold(void) {
  Adafruit_BusIO_Register thresh_reg =
      Adafruit_BusIO_Register(i2c_dev, FT6X36_REG_THRESHHOLD, 1);
  return thresh_reg.read();
}

/**************************************************************************/
/*!
    @brief  Sets the touch sensitivity threshold
    @param  thresh Threshold value
*/
/**************************************************************************/
void Adafruit_FT6x36::setThreshold(uint8_t thresh) {
  Adafruit_BusIO_Register thresh_reg =
      Adafruit_BusIO_Register(i2c_dev, FT6X36_REG_THRESHHOLD, 1);
  thresh_reg.write(thresh);
}
