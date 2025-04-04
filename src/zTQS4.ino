#define I2C_SDA 4
#define I2C_SCL 5

#include <Arduino.h>
#include <Smoothed.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_TinyUSB.h>
#include <hid.h>
#include <utils.h>
#include <structs.h>

#include <Adafruit_ADS1X15.h>
int CURRENT_VERSION = 1;

userSettings settings;

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

bool pBtns[64];
bool lBtns[32];

byte grip1Data[12];
uint16_t grip1Axis[4];
bool grip1Btns[32];

byte grip2Data[12];
uint16_t grip2Axis[4];
bool grip2Btns[32];



Smoothed<long> smoothedAxis1;


Adafruit_USBD_HID joystick_hid(desc_hid_report, sizeof(desc_hid_report), HID_ITF_PROTOCOL_NONE, 2, false);
//Adafruit_USBD_HID mouse_hid(desc_hid_report_mouse, sizeof(desc_hid_report_mouse), HID_ITF_PROTOCOL_NONE, 4, false);


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


#define CALIBRATION_POINTS 21  // Calibration at 0%, 10%, ... 100%
#define WAIT_DURATION 4000   // 4 seconds wait (non-blocking)

struct CalibrationPoint {
  int rawValue;       // Value from the secondary potentiometer
  int correctedValue; // Value from the main potentiometer (reference)
};

CalibrationPoint correctionTable[CALIBRATION_POINTS];

enum CalibrationState {
  WAIT_FOR_START,
  SHOW_TARGET,
  WAIT_FOR_SETTLE,
  COMPLETE
};

CalibrationState calState = COMPLETE;
int currentPoint = 0;
unsigned long waitStartTime = 0;
unsigned long lastPrintTime = 0;  // For updating display frequently

int CALIBRATION_TABLE_ADDR = 512;


float rXAlpha = 0.3;  // Adjust between 0.1 - 0.3 for best results
float rXFiltered = 0;;
float rYAlpha = .3;
float rYFiltered = 0;
float xAlpha = .4;
float xFiltered = 0;
float yAlpha = .4;
float yFiltered = 0;
float zAlpha = .2;
float zFiltered = 0;

#define MEDIAN_FILTER_SIZE 10

int16_t medianFilter(int16_t newValue, int16_t *buffer, int &index) {
  buffer[index] = newValue;
  index = (index + 1) % MEDIAN_FILTER_SIZE;

  int16_t sorted[MEDIAN_FILTER_SIZE];
  memcpy(sorted, buffer, sizeof(sorted));
  for (int i = 0; i < MEDIAN_FILTER_SIZE - 1; i++) {
    for (int j = i + 1; j < MEDIAN_FILTER_SIZE; j++) {
      if (sorted[i] > sorted[j]) {
        int16_t temp = sorted[i];
        sorted[i] = sorted[j];
        sorted[j] = temp;
      }
    }
  }
  return sorted[MEDIAN_FILTER_SIZE / 2];
}

int16_t xBuffer[MEDIAN_FILTER_SIZE] = {0};
int xIndex = 0;
int16_t yBuffer[MEDIAN_FILTER_SIZE] = {0};
int yIndex = 0;

void setup()
{
  EEPROM.begin(1024);
  EEPROM.get(CALIBRATION_TABLE_ADDR, correctionTable);

  analogReadResolution(12);

  Serial.begin(115200);


  Serial.println("Starting...");
  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.begin();
  Serial.println("Started I2C...");

  smoothedAxis1.begin(SMOOTHED_AVERAGE, 60);

  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }

  if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);
  }

  USBDevice.setID(0x2E8A, 0x0410);
  USBDevice.setProductDescriptor("zTQS Mk IV");
  USBDevice.setManufacturerDescriptor("zStudio");
  //joystick_hid.setReportCallback(get_report_callback, set_report_callback);

  joystick_hid.begin();
  //mouse_hid.begin();
  while (!TinyUSBDevice.mounted())
    delay(1);


  settings.version = CURRENT_VERSION;
  
  //TDC X Axis
  settings.xMin = 315;
  settings.xMax =670;
  settings.xCenter = 500;
  settings.xDz = 30;

  //TDC Y Axis
  settings.yMin = 300;
  settings.yMax = 670;
  settings.yCenter = 480;
  settings.yDz = 30;

  //ANT Elev wheel
  settings.zMin = 410;
  settings.zMax = 640;
  settings.zCenter = 512;
  settings.zDz = 10;
  
  //outboard throttle
  settings.rYMin = 6760;
  settings.rYMax = 13290;
  
  //inboard throttle
  settings.rXMin = 6850;
  settings.rXMax = 13300;

  settings.invertX = true;
  settings.invertY = true;

  settings.invertRX = true;
  settings.invertRY = true;

  //startCalibration();


}

int16_t filter(int16_t raw, float alpha, float &filtered) {
  return filtered = (alpha * raw) + ((1 - alpha) * filtered);
}

void loop()
{

  if (calState != COMPLETE) {
    updateCalibration();
    return;
  }

 
  getGripsData();
  
  axisRxRaw = ads.readADC_SingleEnded(0);
  axisRyRaw = ads.readADC_SingleEnded(1);

  int16_t rawX = mapAxis(axis2, settings.xMin, settings.xMax, settings.xCenter, settings.xDz, false);
  int16_t rawY = mapAxis(axis3, settings.yMin, settings.yMax, settings.yCenter, settings.yDz, false);

  JoystickData.x = filter(medianFilter(rawX, xBuffer, xIndex), xAlpha, xFiltered);
  if (settings.invertX) {
    JoystickData.x = -JoystickData.x;
  }

  JoystickData.y = filter(medianFilter(rawY, yBuffer, yIndex), yAlpha, yFiltered);
  if (settings.invertY) {
    JoystickData.y = -JoystickData.y;
  }
  
  JoystickData.z = filter(mapAxis(axis1, settings.zMin, settings.zMax, settings.zCenter, settings.zDz, false), zAlpha, zFiltered);
  JoystickData.rx = filter(mapAxis(axisRxRaw, settings.rXMin, settings.rXMax, false), rXAlpha, rXFiltered);
  if (settings.invertRX) {
    JoystickData.rx = -JoystickData.rx;
  }
  
  //JoystickData.ry = filter(mapAxis(axisRyRaw, settings.rYMin, settings.rYMax, false) , rYAlpha, rYFiltered);
  
  JoystickData.ry = filter(interpolate(mapAxis(axisRyRaw, settings.rYMin, settings.rYMax, false)), rYAlpha, rYFiltered);

  if (settings.invertRY) {
    JoystickData.ry = -JoystickData.ry;
  }

  // Check if ry and rx are within 3 percent of each other
  if (abs(JoystickData.ry - JoystickData.rx) <= 0.3 * max(abs(JoystickData.ry), abs(JoystickData.rx))) {
    //JoystickData.ry = JoystickData.rx;
  }


//printJoyData();

  mapBtns();

  JoystickData.buttons = boolArrayToUint32(lBtns);

  if (!joystick_hid.ready())
  {
    return;
  }else {
    joystick_hid.sendReport(0, &JoystickData, sizeof(JoystickData));
  }


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
    pBtns[i] = grip1Btns[i];
  }

  for (int i = 32; i < 64; i++) {
    pBtns[i] = grip2Btns[i - 32];
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


  // for (int i = 0; i < 32; i++)
  // {
  //   Serial.print(gripBtns[i]);
  // }
  // Serial.println();
}



void printJoyData()
{
  for (int i = 0; i < 64; i++){
    if(pBtns[i]) {
      Serial.print(i);
      Serial.print(" ");
    }
  }

  Serial.print("X: ");Serial.print(JoystickData.x); Serial.print("("); Serial.print(axis2); Serial.print(")");
  Serial.print(" Y: "); Serial.print(JoystickData.y); Serial.print("(");Serial.print(axis3);Serial.print(")");
  Serial.print(" Z: "); Serial.print(JoystickData.z); Serial.print("("); Serial.print(axis1); Serial.print(")");;
  Serial.print(" RX: ");Serial.print(JoystickData.rx);Serial.print("(");Serial.print(axisRxRaw);Serial.print(")");
  Serial.print(" Ry: "); Serial.print(JoystickData.ry);Serial.print("(");Serial.print(axisRyRaw);Serial.print(")");
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


int16_t mapAxis(uint16_t raw, uint16_t fromMin, uint16_t fromMax, uint16_t center, uint16_t deadzone, bool debug) {
  const int16_t toMin = -32767;
  const int16_t toMax = 32767;
  int16_t toCenter = (toMin + toMax) / 2; // This will be ~0

  // Calculate lower and upper boundaries of the deadzone.
  uint16_t lowerDead = (center > deadzone) ? center - deadzone : 0;
  uint16_t upperDead = center + deadzone;

  // If raw is within the deadzone around the center, return the target center.
  if (raw >= lowerDead && raw <= upperDead) {
      if (debug) { Serial.println("Returning center"); }
      return toCenter;
  }
  // For raw values below the deadzone: map from [fromMin, lowerDead] -> [toMin, toCenter]
  else if (raw < lowerDead) {
      uint16_t effectiveRange = lowerDead - fromMin;
      if (effectiveRange == 0) effectiveRange = 1; // avoid division by zero
      uint16_t lowerInput = raw - fromMin;
      int32_t mapped = toMin + ((int32_t) lowerInput * (toCenter - toMin)) / effectiveRange;
      if (debug) { Serial.println(mapped); }
      // Clamp the result
      if (raw < fromMin) {
          mapped = toMin;
      }

      return (int16_t) mapped;
  }
  // For raw values above the deadzone: map from [upperDead, fromMax] -> [toCenter, toMax]
  else { // raw > upperDead
      uint16_t effectiveRange = fromMax - upperDead;
      if (effectiveRange == 0) effectiveRange = 1; // avoid division by zero
      uint16_t upperInput = raw - upperDead;
      int32_t mapped = toCenter + ((int32_t) upperInput * (toMax - toCenter)) / effectiveRange;
      // Clamp the result
      if (debug) { Serial.println(mapped); }
      if (raw > fromMax) {
          mapped = toMax;
      }
      return (int16_t) mapped;
  }
}

// Overloaded version when center and deadzone are not provided.
// In this case, center will be the midpoint between fromMin and fromMax, and deadzone is zero.
int16_t mapAxis(uint16_t raw, uint16_t fromMin, uint16_t fromMax, bool debug) {
  uint16_t center = (fromMin + fromMax) / 2;
  return mapAxis(raw, fromMin, fromMax, center, 0, debug);
}

void mapBtns(){
  //cancel out midldle button clicks for RKJXM hat switches
  if (pBtns[0] || pBtns[1] || pBtns[2] || pBtns[3]) {
    pBtns[4] = false;
  }

  if (pBtns[5] || pBtns[6] || pBtns[7] || pBtns[8]) {
    pBtns[9] = false;
  }

  if (pBtns[37] || pBtns[38] || pBtns[39] || pBtns[40]) {
    pBtns[41] = false;
  }

  lBtns[ 0] = pBtns[ 1];  lBtns[ 1] = pBtns[ 2];  lBtns[ 2] = pBtns[47];  lBtns[ 3] = NULL;  
  lBtns[ 4] = pBtns[49];  lBtns[ 5] = pBtns[48];  lBtns[ 6] = pBtns[41];  lBtns[ 7] = pBtns[ 4];
  lBtns[ 8] = pBtns[ 1];  lBtns[ 9] = pBtns[ 2];  lBtns[10] = pBtns[ 3];  lBtns[11] = pBtns[ 0];
  lBtns[12] = pBtns[ 9];  lBtns[13] = pBtns[ 7];  lBtns[14] = pBtns[ 8];  lBtns[15] = pBtns[ 5];
  lBtns[16] = pBtns[ 6];  lBtns[17] = pBtns[17];  lBtns[18] = NULL;       lBtns[19] = pBtns[15];
  lBtns[20] = pBtns[16];  lBtns[21] = pBtns[18];  lBtns[22] = NULL;       lBtns[23] = NULL;
  lBtns[24] = pBtns[40];  lBtns[25] = NULL;       lBtns[26] = NULL;       lBtns[27] = NULL;
  lBtns[28] = NULL;       lBtns[29] = pBtns[39];  lBtns[30] = pBtns[38];   lBtns[31] = pBtns[37];
  
  //engine cut off buttons
  
  if(settings.invertRX){
    if (axisRxRaw > settings.rXMax + 100) {
      lBtns[0] = true;
    } else {
      lBtns[0] = false;
    }
  }else {

    if (axisRxRaw < settings.rXMin - 100) {
      lBtns[0] = true;
    } else {
      lBtns[0] = false;
    }
  }

  if (settings.invertRY) {
    if (axisRyRaw > settings.rYMax + 100) {
      lBtns[1] = true;
    } else {
      lBtns[1] = false;
    }
  } else {

    if (axisRyRaw < settings.rYMin - 100) {
      lBtns[1] = true;
    } else {
      lBtns[1] = false;
    }
  }

  //ouboard grip toggle state
  if (!pBtns[47] && !pBtns[49]) {
    lBtns[3] = true;
  }else {
    lBtns[3] = false;
  }

  //Speedbrake toggle state
  if (!pBtns[15] && !pBtns[17]) {
    lBtns[18] = true;
  }else {
    lBtns[18] = false;
  }

  // TDC axes to btn 
  if (axis2 < settings.xMin + 100) {
    lBtns[28] = true;
  }

  if (axis2 > settings.xMax - 100) {
    lBtns[26] = true;
  }

  if (axis3 < settings.yMin + 100) {
    lBtns[27] = true;
  }

  if (axis3 > settings.yMax - 100) {
    lBtns[25] = true;
  }

  // ANT Elev wheel to btn
  if (axis1 < settings.zMin + 100) {
    lBtns[22] = true;
  }
  if (axis1 > settings.zMax - 100) {
    lBtns[23] = true;
  }
  
}


void startCalibration() {
  Serial.println("Starting Calibration Routine...");
  currentPoint = 0;
  calState = SHOW_TARGET;
}



void updateCalibration() {
  unsigned long currentMillis = millis();
  // Continuously read main pot value (mapped value)
  int16_t mainVal = mapAxis(ads.readADC_SingleEnded(0), settings.rXMin, settings.rXMax, false);
  long percentage = ((long)mainVal + 32768L) * 100L / 65535L;

  // For this calibration routine, the target percent for each calibration point
  int targetPercent = currentPoint * 100 / (CALIBRATION_POINTS - 1);
  const int tolerance = 1;  // Acceptable tolerance in percentage points
  
  // Print calibration status every 200ms
  if (currentMillis - lastPrintTime >= 200) {
    Serial.print("Calibration Point ");
    Serial.print(currentPoint);
    Serial.print(" / ");
    Serial.print(CALIBRATION_POINTS - 1);
    Serial.print(" - Target: ");
    Serial.print(targetPercent);
    Serial.print("%, current reading: ");
    Serial.print(mainVal);
    Serial.print(" (");
    Serial.print(percentage);
    Serial.println("%)");
    lastPrintTime = currentMillis;
  }
  
  // Static flag to prevent multiple recordings at the same calibration point
  static bool pointRecorded = false;
  
  // When main pot reading percentage is within tolerance of the target...
  if (!pointRecorded && (abs(percentage - targetPercent) <= tolerance)) {
    // Record the calibration values
    int16_t secondaryVal = mapAxis(ads.readADC_SingleEnded(1), settings.rYMin, settings.rYMax, false);
    correctionTable[currentPoint].correctedValue = mainVal;
    correctionTable[currentPoint].rawValue = secondaryVal;
    
    Serial.print("Recorded Calibration Point ");
    Serial.print(currentPoint);
    Serial.print(": Main = ");
    Serial.print(mainVal);
    Serial.print(" | Secondary = ");
    Serial.println(secondaryVal);
    
    pointRecorded = true;
    currentPoint++;  // Advance to the next calibration point
    
    if (currentPoint >= CALIBRATION_POINTS) {
      calState = COMPLETE;
      Serial.println("Calibration Complete!");
      for (int i = 0; i < CALIBRATION_POINTS; i++) {
        Serial.print("Point ");
        Serial.print(i);
        Serial.print(": Raw = ");
        Serial.print(correctionTable[i].rawValue);
        Serial.print(" | Corrected = ");
        Serial.println(correctionTable[i].correctedValue);
      }
      
      EEPROM.put(CALIBRATION_TABLE_ADDR, correctionTable);
      EEPROM.commit();
      Serial.println("Calibration data saved to EEPROM.");
      Serial.print("Calibration data size: ");
      Serial.println(sizeof(correctionTable));
    }
  }
  
  // Reset the flag once the reading leaves the tolerance zone
  if (pointRecorded && (abs(percentage - targetPercent) > tolerance)) {
    pointRecorded = false;
  }
}

int interpolate(int input) {
  for (int i = 0; i < CALIBRATION_POINTS - 1; i++) {
    if (input >= correctionTable[i].rawValue && input <= correctionTable[i + 1].rawValue) {
      float ratio = (float)(input - correctionTable[i].rawValue) /
                    (correctionTable[i + 1].rawValue - correctionTable[i].rawValue);
      return correctionTable[i].correctedValue + ratio *
             (correctionTable[i + 1].correctedValue - correctionTable[i].correctedValue);
    }
  }
  return input;
}