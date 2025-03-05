#include <arduino.h>

#ifndef UTIL_HEADER_FILE_H
#define UTIL_HEADER_FILE_H

#define ROWS 21
#define COLS 2

double scale(float input, float factor)
{
    int sign = 1;

    if (input < 0)
    {
        sign = -1;
    }

    double result = std::pow(std::abs(input), factor);
    result = result * sign;
    return result;
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/// @brief map a setting value from 0-255 to a float value between 0.25 and 3. 85 input will return 1
/// @param inputValue
/// @return
float mapSettingIntToFloat(int inputValue)
{
    return ((float)inputValue / 255) * 3.75 + .25;
}

bool getBit8(uint8_t number, int index)
{
    if (index < 0 || index >= 8)
    {
        // Index out of range
        return false;
    }
    return (number >> (7 - index)) & 0x01;
}

bool getBit(uint64_t number, int index)
{
    if (index < 0 || index >= 64)
    {
        // Index out of range
        return false;
    }
    return (number >> (63 - index)) & 0x01;
}

uint32_t boolArrayToUint32(bool boolArray[32])
{
    uint32_t result = 0;

    for (int i = 0; i < 32; i++)
    {
        if (boolArray[i])
        {
            result |= (1u << i); // Set the i-th bit to 1 if the boolean is true.
        }
    }

    return result;
}

float clampFloat(float val, float min, float max)
{
    if (val < min)
    {
        return min;
    }
    else if (val > max)
    {
        return max;
    }
    else
    {
        return val;
    }
}


// struct Point
// {
//     float input;
//     float output;
// };

// Define your control points here. Make sure they are in increasing order.
// First elemnt of the array is the input, second element is the output.
int16_t defaultControlPointsX[ROWS][COLS] = {
    {-2047, -2047},
    {-1842, -1842},
    {-1637, -1637},
    {-1423, -1423},
    {-1228, -1228},
    {-1023, -1023},
    {-818, -818},
    {-614, -614},
    {-408, -408},
    {-204, -204},
    {0, 0},
    {204, 204},
    {408, 408},
    {614, 614},
    {818, 818},
    {1023, 1024},
    {1228, 1228},
    {1423, 1423},
    {1637, 1637},
    {1842, 1842},
    {2047, 2047},
    };

int16_t defaultControlPointsY[ROWS][COLS] = {
    {-2047, -2047},
    {-1842, -1842},
    {-1637, -1637},
    {-1423, -1423},
    {-1228, -1228},
    {-1023, -1023},
    {-818, -818},
    {-614, -614},
    {-408, -408},
    {-204, -204},
    {0, 0},
    {204, 204},
    {408, 408},
    {614, 614},
    {818, 818},
    {1023, 1024},
    {1228, 1228},
    {1423, 1423},
    {1637, 1637},
    {1842, 1842},
    {2047, 2047},
    };

float appplyCurve(float input, int16_t controlPoints[ROWS][COLS])
{
    // Find the two control points x is between.
    for (int i = 0; i < ROWS - 1; i++)
    {
        if (input >= controlPoints[i][0] && input <= controlPoints[i + 1][0])
        {
            // Perform linear interpolation between the two points.
            float t = (input - controlPoints[i][0]) / (controlPoints[i + 1][0] - controlPoints[i][0]);
            return controlPoints[i][1] * (1 - t) + controlPoints[i + 1][1] * t;
        }
    }

    // If x is outside the range of the control points, return the closest one.
    if (input < controlPoints[0][0])
    {
        return controlPoints[0][1];
    }
    else
    {
        return controlPoints[ROWS - 1][1];
    }
}



// Serialize a 2D array into a 1D array
void serializeCurvesArray(int16_t array[ROWS][COLS], int16_t serialized[ROWS*COLS]) {
  for (int i = 0; i < ROWS; i++) {
    for (int j = 0; j < COLS; j++) {
      serialized[i * COLS + j] = array[i][j];
    }
  }
}

// Unserialize a 1D array into a 2D array
void unserializeCurvesArray(int16_t array[ROWS][COLS], int16_t serialized[ROWS*COLS]) {
  for (int i = 0; i < ROWS; i++) {
    for (int j = 0; j < COLS; j++) {
      array[i][j] = serialized[i * COLS + j];
    }
  }
}


#endif // UTIL_HEADER_FILE_H


