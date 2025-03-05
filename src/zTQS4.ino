#define DATA_PIN 3
#define CLOCK_PIN 4
#define LATCH_PIN 5

#define I2C_SDA 0
#define I2C_SCL 1

bool dataToShift[128];
#include <Arduino.h>
#include <bitset>
#include <stdexcept>
#include <Smoothed.h>
#include <Wire.h>

int sendSize = sizeof(dataToShift);

bool sending = false;
int sendIndex = 0;
int bitIndex = 0;
bool lastLatch;
int latchState;

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

bool btns[64];

byte grip1Data[12];
uint16_t grip1Axis[4];
bool grip1Btns[32];

byte grip2Data[12];
uint16_t grip2Axis[4];
bool grip2Btns[32];

Smoothed<long> smoothedAxis1;

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
    gripAxis[i] = data[(i * 2) + 3] << 8 | data[i + 4];
  }
}

void setup()
{
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, INPUT);
  pinMode(LATCH_PIN, INPUT); // Changed to INPUT
  analogReadResolution(12);

  Serial.begin(115200);

  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.begin();
  smoothedAxis1.begin(SMOOTHED_AVERAGE, 60);
}

void loop()
{
  getGripsData();

  // smoothedAxis1.add(analogRead(26));
  // axis1 = smoothedAxis1.get();
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

  printJoyData();
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
  Serial.print(millis());
  Serial.println();
  delay(10);
}
