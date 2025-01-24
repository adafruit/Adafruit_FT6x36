/*!
 * @file Adafruit_FT6x36.h
 *
 * This is a library for the FT6x36 Capacitive Touch Screen Controller family
 * 
 * These controllers use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the touch controller
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section author Author
 * Written by Limor Fried/Ladyada for Adafruit Industries.
 * 
 * @section license License
 * MIT license, all text above must be included in any redistribution
 */

#ifndef ADAFRUIT_FT6X36_LIBRARY
#define ADAFRUIT_FT6X36_LIBRARY

#include "Arduino.h"
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_BusIO_Register.h>

#define FT6X36_I2C_ADDR                0x38    //!< I2C address
#define FT6X36_REG_DEVICE_MODE          0x00    //!< Device mode register
#define FT6X36_REG_GESTURE_ID           0x01    //!< Gesture ID register
#define FT6X36_REG_NUM_TOUCHES          0x02    //!< Number of touches register
#define FT6X36_REG_P1_XH                0x03    //!< First touch X position high byte
#define FT6X36_REG_P1_XL                0x04    //!< First touch X position low byte
#define FT6X36_REG_P1_YH                0x05    //!< First touch Y position high byte
#define FT6X36_REG_P1_YL                0x06    //!< First touch Y position low byte
#define FT6X36_REG_P1_WEIGHT            0x07    //!< First touch weight
#define FT6X36_REG_P1_MISC              0x08    //!< First touch area
#define FT6X36_REG_P2_XH                0x09    //!< Second touch X position high byte
#define FT6X36_REG_P2_XL                0x0A    //!< Second touch X position low byte
#define FT6X36_REG_P2_YH                0x0B    //!< Second touch Y position high byte
#define FT6X36_REG_P2_YL                0x0C    //!< Second touch Y position low byte
#define FT6X36_REG_P2_WEIGHT            0x0D    //!< Second touch weight
#define FT6X36_REG_P2_MISC              0x0E    //!< Second touch area
#define FT6X36_REG_THRESHHOLD           0x80    //!< Touch threshold
#define FT6X36_REG_FILTER_COEF          0x85    //!< Filter coefficient
#define FT6X36_REG_CTRL                 0x86    //!< Control register
#define FT6X36_REG_TIME_ENTER_MONITOR   0x87    //!< Enter monitor time
#define FT6X36_REG_TOUCHRATE_ACTIVE     0x88    //!< Touch rate in active mode
#define FT6X36_REG_TOUCHRATE_MONITOR    0x89    //!< Touch rate in monitor mode
#define FT6X36_REG_RADIAN_VALUE         0x91    //!< Radian value
#define FT6X36_REG_OFFSET_LEFT_RIGHT    0x92    //!< Left/right offset
#define FT6X36_REG_OFFSET_UP_DOWN       0x93    //!< Up/down offset
#define FT6X36_REG_DISTANCE_LEFT_RIGHT  0x94    //!< Left/right distance
#define FT6X36_REG_DISTANCE_UP_DOWN     0x95    //!< Up/down distance
#define FT6X36_REG_DISTANCE_ZOOM        0x96    //!< Zoom distance
#define FT6X36_REG_LIB_VERSION_H        0xA1    //!< Library version high byte
#define FT6X36_REG_LIB_VERSION_L        0xA2    //!< Library version low byte
#define FT6X36_REG_CHIPID               0xA3    //!< Chip ID
#define FT6X36_REG_INTERRUPT_MODE       0xA4    //!< Interrupt mode
#define FT6X36_REG_POWER_MODE           0xA5    //!< Power mode
#define FT6X36_REG_FIRMWARE_VERSION     0xA6    //!< Firmware version
#define FT6X36_REG_PANEL_ID             0xA8    //!< Panel ID
#define FT6X36_REG_STATE                0xBC    //!< State

// Power modes
#define FT6X36_PMODE_ACTIVE             0x00    //!< Active mode
#define FT6X36_PMODE_MONITOR            0x01    //!< Monitor mode
#define FT6X36_PMODE_STANDBY            0x02    //!< Standby mode
#define FT6X36_PMODE_HIBERNATE          0x03    //!< Hibernate mode

#define FT6X36_VENDID            0x11    //!< Expected vendor ID value
#define FT6206_CHIPID            0x06    //!< FT6206 chip ID
#define FT6236_CHIPID            0x36    //!< FT6236 chip ID
#define FT6336_CHIPID            0x64    //!< FT6336 chip ID

#define FT6X36_DEFAULT_THRESHOLD  22     //!< Default touch threshold

/*!
    @brief Raw touch events directly from the controller
*/
enum class FT6X36_RawEvent {
  PRESS_DOWN, //!< Initial touch contact
  LIFT_UP,    //!< Touch release
  CONTACT,    //!< Continuous touch
  NO_EVENT    //!< No touch event
};

/*!
    @brief Processed touch events with gesture recognition
*/
enum class FT6X36_Event {
  NONE,       //!< No event
  TOUCH_START, //!< Touch initiated
  TOUCH_MOVE,  //!< Touch point moved
  TOUCH_END,   //!< Touch released
  TAP,         //!< Quick touch and release
  DRAG_START,  //!< Start of drag gesture
  DRAG_MOVE,   //!< Drag in progress
  DRAG_END     //!< End of drag gesture
};

/*!
    @brief  Class that stores state and functions for interacting with FT6x36 
            capacitive touch controller family
*/
class Adafruit_FT6x36 {
 public:
  Adafruit_FT6x36(void);
  
  bool begin(uint8_t thresh = FT6X36_DEFAULT_THRESHOLD, 
            TwoWire *wire = &Wire);

  // Basic touch functions  
  uint8_t touched(void);
  void getTouchPoints(uint16_t *x1, uint16_t *y1, uint16_t *x2 = NULL, uint16_t *y2 = NULL);
  FT6X36_RawEvent getLastTouchEvent(uint8_t touch_num = 0);
  
  // Configuration
  void setThreshold(uint8_t threshold);
  uint8_t getThreshold(void);
  uint8_t getVersion(void);
  uint8_t getChipID(void);
  
  // Debug
  void printDebugInfo(void);           ///< Print debug register values

 private:
  Adafruit_I2CDevice *i2c_dev = NULL;  ///< Pointer to I2C bus interface
  uint16_t _touches;                   ///< Number of touches detected
  uint16_t _touchX[2];                 ///< Touch X coordinates
  uint16_t _touchY[2];                 ///< Touch Y coordinates
  uint8_t _touchEvent[2];              ///< Touch event types

  bool readData(void);                 ///< Read touch data from controller
};

#endif // ADAFRUIT_FT6X36_LIBRARY
