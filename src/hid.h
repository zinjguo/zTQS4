#ifndef HID_HEADER_FILE_H
#define HID_HEADER_FILE_H

#include <arduino.h>
#include <Adafruit_TinyUSB.h>

#define TUD_HID_REPORT_DESC_GAMEPAD_16BIT_AXIS(...)                                                     \
  HID_USAGE_PAGE(HID_USAGE_PAGE_DESKTOP),                                                               \
      HID_USAGE(HID_USAGE_DESKTOP_GAMEPAD),                                                             \
      HID_COLLECTION(HID_COLLECTION_APPLICATION),                                                       \
      __VA_ARGS__                                 /* 16 bit X, Y, Z, Rz, Rx, Ry (min -127, max 127 ) */ \
      HID_USAGE_PAGE(HID_USAGE_PAGE_DESKTOP),                                                           \
      HID_USAGE(HID_USAGE_DESKTOP_X),                                                                   \
      HID_USAGE(HID_USAGE_DESKTOP_Y),                                                                   \
      HID_USAGE(HID_USAGE_DESKTOP_Z),                                                                   \
      HID_USAGE(HID_USAGE_DESKTOP_RZ),                                                                  \
      HID_USAGE(HID_USAGE_DESKTOP_RX),                                                                  \
      HID_USAGE(HID_USAGE_DESKTOP_RY),                                                                  \
      HID_LOGICAL_MIN_N(0x8000, 2),                                                                     \
      HID_LOGICAL_MAX_N(0x7FFF, 2),                                                                     \
      HID_REPORT_COUNT(6),                                                                              \
      HID_REPORT_SIZE(16),                                                                              \
      HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE), /* 8 bit DPad/Hat Button Map  */               \
      HID_USAGE_PAGE(HID_USAGE_PAGE_DESKTOP),                                                           \
      HID_USAGE(HID_USAGE_DESKTOP_HAT_SWITCH),                                                          \
      HID_LOGICAL_MIN(1),                                                                               \
      HID_LOGICAL_MAX(8),                                                                               \
      HID_PHYSICAL_MIN(0),                                                                              \
      HID_PHYSICAL_MAX_N(315, 2),                                                                       \
      HID_REPORT_COUNT(1),                                                                              \
      HID_REPORT_SIZE(8),                                                                               \
      HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE), /* 32 bit Button Map */                        \
      HID_USAGE_PAGE(HID_USAGE_PAGE_BUTTON),                                                            \
      HID_USAGE_MIN(1),                                                                                 \
      HID_USAGE_MAX(32),                                                                                \
      HID_LOGICAL_MIN(0),                                                                               \
      HID_LOGICAL_MAX(1),                                                                               \
      HID_REPORT_COUNT(32),                                                                             \
      HID_REPORT_SIZE(1),                                                                               \
      HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),                                                \
      HID_COLLECTION_END


#define TUD_HID_REPORT_DESC_MOUSE_ZFSB(...) \
  HID_USAGE_PAGE ( HID_USAGE_PAGE_DESKTOP      )                   ,\
  HID_USAGE      ( HID_USAGE_DESKTOP_MOUSE     )                   ,\
  HID_COLLECTION ( HID_COLLECTION_APPLICATION  )                   ,\
    /* Report ID if any */\
    __VA_ARGS__ \
    HID_USAGE      ( HID_USAGE_DESKTOP_POINTER )                   ,\
    HID_COLLECTION ( HID_COLLECTION_PHYSICAL   )                   ,\
      HID_USAGE_PAGE  ( HID_USAGE_PAGE_BUTTON  )                   ,\
        HID_USAGE_MIN   ( 1                                      ) ,\
        HID_USAGE_MAX   ( 5                                      ) ,\
        HID_LOGICAL_MIN ( 0                                      ) ,\
        HID_LOGICAL_MAX ( 1                                      ) ,\
        /* Left, Right, Middle, Backward, Forward buttons */ \
        HID_REPORT_COUNT( 5                                      ) ,\
        HID_REPORT_SIZE ( 1                                      ) ,\
        HID_INPUT       ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ) ,\
        /* 3 bit padding */ \
        HID_REPORT_COUNT( 1                                      ) ,\
        HID_REPORT_SIZE ( 3                                      ) ,\
        HID_INPUT       ( HID_CONSTANT                           ) ,\
      HID_USAGE_PAGE  ( HID_USAGE_PAGE_DESKTOP )                   ,\
        /* X, Y position [-127, 127] */ \
        HID_USAGE       ( HID_USAGE_DESKTOP_X                    ) ,\
        HID_USAGE       ( HID_USAGE_DESKTOP_Y                    ) ,\
        HID_LOGICAL_MIN_N(0x8001, 2),                                                                     \
        HID_LOGICAL_MAX_N(0x7FFF, 2),                                                                     \
        HID_REPORT_COUNT( 2                                      ) ,\
        HID_REPORT_SIZE ( 16                                      ) ,\
        HID_INPUT       ( HID_DATA | HID_VARIABLE | HID_RELATIVE ) ,\
        /* Verital wheel scroll [-127, 127] */ \
        HID_USAGE       ( HID_USAGE_DESKTOP_WHEEL                )  ,\
        HID_LOGICAL_MIN ( 0x81                                   )  ,\
        HID_LOGICAL_MAX ( 0x7f                                   )  ,\
        HID_REPORT_COUNT( 1                                      )  ,\
        HID_REPORT_SIZE ( 8                                      )  ,\
        HID_INPUT       ( HID_DATA | HID_VARIABLE | HID_RELATIVE )  ,\
      HID_USAGE_PAGE  ( HID_USAGE_PAGE_CONSUMER ), \
       /* Horizontal wheel scroll [-127, 127] */ \
        HID_USAGE_N     ( HID_USAGE_CONSUMER_AC_PAN, 2           ), \
        HID_LOGICAL_MIN ( 0x81                                   ), \
        HID_LOGICAL_MAX ( 0x7f                                   ), \
        HID_REPORT_COUNT( 1                                      ), \
        HID_REPORT_SIZE ( 8                                      ), \
        HID_INPUT       ( HID_DATA | HID_VARIABLE | HID_RELATIVE ), \
    HID_COLLECTION_END                                            , \
  HID_COLLECTION_END \




uint8_t const desc_hid_report[] =
{
  TUD_HID_REPORT_DESC_GAMEPAD_16BIT_AXIS(),
};

uint8_t const desc_hid_report_mouse[] =
  {
      TUD_HID_REPORT_DESC_MOUSE_ZFSB(),
};

// HID Report Descriptor for settings_hid
const uint8_t settings_hid_report_descriptor[] = {
  0x06, 0x00, 0xFF,  // Usage Page (Vendor Defined 0xFF00)
  0x09, 0x01,        // Usage (0x01)
  0xA1, 0x01,        // Collection (Application)
  0x85, 0x02,        //   Report ID (2)
  0x09, 0x02,        //   Usage (0x02)
  0x15, 0x00,        //   Logical Minimum (0)
  0x26, 0xFF, 0x00,  //   Logical Maximum (255)
  0x75, 0x08,        //   Report Size (8)
  0x95, 0x3F,        //   Report Count (63)
  0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
  0xC0,              // End Collection
};

// Joystick report strcut
typedef struct TU_ATTR_PACKED
{
  int16_t x = 0;        ///< Delta x  movement of left analog-stick
  int16_t y = 0;        ///< Delta y  movement of left analog-stick
  int16_t z = 0;        ///< Delta z  movement of right analog-joystick
  int16_t rz = 0;       ///< Delta Rz movement of right analog-joystick
  int16_t rx = 0;       ///< Delta Rx movement of analog left trigger
  int16_t ry = 0;       ///< Delta Ry movement of analog right trigger
  uint8_t hat = 0;      ///< Buttons mask for currently pressed buttons in the DPad/hat
  uint32_t buttons = 0; ///< Buttons mask for currently pressed buttons
} hid_joystick_report_t;

// Define the HID mouse report structure
typedef struct TU_ATTR_PACKED {
    uint8_t buttons; // 8 bits for buttons (left, right, middle)
    int16_t x;        // 16 bits for X position (relative movement)
    int16_t y;        // 16 bits for Y position (relative movement)
    int8_t wheel;    // 8 bits for vertical wheel (relative movement)
    int8_t  pan;     // using AC Pan
} hid_mouse_zfsb_report_t;




#endif // HID_HEADER_FILE_H