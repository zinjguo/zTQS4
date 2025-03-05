int CURRENT_VERSION = 1;

#define I2C_SDA 4
#define I2C_SCL 5

#include <Arduino.h>
#include <bitset>
#include <stdexcept>
#include <Smoothed.h>
#include <Wire.h>
#include <EEPROM.h>
#include "Adafruit_TinyUSB.h"
#include "hid.h"
#include <structs.h>

#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;

int grip1 = 0x08;
int grip2 = 0x09;

int clockIndex = 0;

const int AXIS1_INDEX = 64;
const int AXIS2_INDEX = 80;
const int AXIS3_INDEX = 96;
const int AXIS4_INDEX = 112;

uint16_t axis1 = 0;
uint16_t axis2 = 0;
uint16_t axis3 = 0;
uint16_t axis4 = 0;

uint16_t axisRx = 0;
uint16_t axisRy = 0;
uint16_t axisRxRaw = 0;
uint16_t axisRyRaw = 0;

bool btns[64];

byte grip1Data[12];
uint16_t grip1Axis[4];
bool grip1Btns[32];

byte grip2Data[12];
uint16_t grip2Axis[4];
bool grip2Btns[32];

userSettings settings;

Smoothed<long> smoothedAxis1;


Adafruit_USBD_HID joystick_hid(desc_hid_report, sizeof(desc_hid_report), HID_ITF_PROTOCOL_NONE, 2, true);
Adafruit_USBD_HID mouse_hid(desc_hid_report_mouse, sizeof(desc_hid_report_mouse), HID_ITF_PROTOCOL_NONE, 4, false);


hid_joystick_report_t JoystickData;

hid_mouse_zfsb_report_t MouseData;
bool mouseModeEnabled = false;
bool mouseBindPressed = false;
long lastMouseBindPressed = 0;
static unsigned long lastMouseUpdate = 0;
unsigned long mouseUpdateRate = 4;
Smoothed<long> mouseSmoothedX;
Smoothed<long> mouseSmoothedY;
static float mouseXAccumulator = 0.0;
static float mouseYAccumulator = 0.0;
static float mouseWheelAccumulator = 0.0;


void setup()
{
  EEPROM.begin(512);
  settings = EEPROM.get(0, settings);

  if (settings.version != CURRENT_VERSION)
  {
    settings = getDefaultSettings(CURRENT_VERSION);
    EEPROM.put(0, settings);
    EEPROM.commit();
  }

  analogReadResolution(12);

  Serial.begin(115200);

  delay(2000);

  Serial.println("Starting...");
  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.begin();
  Serial.println("Started I2C...");

  delay(2000);
  smoothedAxis1.begin(SMOOTHED_AVERAGE, 60);

  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }


  USBDevice.setID(0x2E8A, 0x0402);
  USBDevice.setProductDescriptor("zTQS Mk. IV");
  joystick_hid.setReportCallback(get_report_callback, set_report_callback);

  joystick_hid.begin();
  mouse_hid.begin();

}

void loop()
{
  getGripsData();

  smoothedAxis1.add(analogRead(26));
  axis1 = smoothedAxis1.get();

  axisRxRaw = ads.readADC_SingleEnded(0);
  axisRyRaw = ads.readADC_SingleEnded(1);

  printJoyData();
}


void getGripsData(){
  Wire.requestFrom(grip1, 12);

  while (Wire.available())
  {
    for (int i = 0; i < 12; i++)
    {
      grip1Data[i] = Wire.read();
    }
  }


  Wire.requestFrom(grip2, 12);

  while (Wire.available())
  {
    for (int i = 0; i < 12; i++)
    {
      grip2Data[i] = Wire.read();
    }
  }


  // parse grip data to dataToShift array
  parseExpGripBytes(grip1Data, grip1Btns, grip1Axis);
  parseExpGripBytes(grip2Data, grip2Btns, grip2Axis);

  for (int i = 0; i < 32; i++) {
    btns[i] = grip1Btns[i];
  }

  for (int i = 32; i < 64; i++) {
    btns[i] = grip2Btns[i - 32];
  }

  axis1 = grip1Axis[0];
  axis2 = grip1Axis[1];
  axis3 = grip1Axis[2];

}

void parseExpGripBytes(byte data[], bool gripBtns[], uint16_t gripAxis[])

{
  // read the first 4 bytes as buttons
  for (int i = 0; i < 4; i++)
  {
    byte b = data[i];
    for (int j = 0; j < 8; j++)
    {
      gripBtns[i * 8 + j] = (b >> j) & 1;
    }
  }

  // read the next 8 bytes as axis
  int axis_offset = 4;
  for (int i = 0; i < 4; i++)
  {
    gripAxis[i] = data[4 + i * 2] | (data[4 + i * 2 + 1] << 8);
  }

  for (int i = 0; i < 32; i++)
  {
    Serial.print(gripBtns[i]);
  }
  Serial.println();
}



void printJoyData()
{
  for (int i = 0; i < 64; i++){
    if(btns[i]) {
      Serial.print(i);
      Serial.print(" ");
    }
  }

  Serial.print("  Axis1: ");
  Serial.print(axis1);
  Serial.print(" ");

  Serial.print("  Axis2: ");
  Serial.print(axis2);
  Serial.print(" ");

  Serial.print("  Axis3: ");
  Serial.print(axis3);
  Serial.print(" ");

  Serial.print("  AxisRx: ");
  Serial.print(axisRxRaw);
  Serial.print(" ");

  erial.print("  AxisRy: ");
  Serial.print(axisRyRaw);
  Serial.print(" ");
  Serial.println();

  delay(10);
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t get_report_callback(uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen)
{
  Serial.print("GET_REPORT, id: ");
  Serial.print(report_id);
  Serial.print(", type: ");
  Serial.print(report_type);
  Serial.print(", length: ");
  Serial.print(reqlen);
  Serial.print(", Buff: ");
  Serial.print(buffer[0]);
  Serial.print(" ");
  Serial.print(buffer[1]);
  Serial.println();
  // not used in this example
  (void)report_id;
  (void)report_type;
  (void)buffer;
  (void)reqlen;
  return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void set_report_callback(uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize)
{
  Serial.print("SET_REPORT, id: ");
  Serial.print(report_id);
  Serial.print(", type: ");
  Serial.print(report_type);
  Serial.print(", bufsize: ");
  Serial.print(bufsize);
  Serial.print(", Buff: ");
  Serial.print(buffer[0]);
  Serial.print(" ");
  Serial.print(buffer[1]);
  Serial.print(" ");
  Serial.print(buffer[2]);
  Serial.print(" ");
  Serial.print(buffer[3]);
  Serial.print(" ");
  Serial.print(buffer[4]);
  Serial.print(" ");
  Serial.println();


  // This example doesn't use multiple report and report ID
  (void)report_id;
  (void)report_type;

  // echo back anything we received from host
}
