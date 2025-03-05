#ifndef STRUCTS_HEADER_FILE_H
#define STRUCTS_HEADER_FILE_H
#include <Arduino.h>


struct userSettings
{
    uint16_t version;
    uint16_t xMin;
    uint16_t xMax;
    uint16_t xCenter;
    uint16_t xDz;
    uint16_t yMin;
    uint16_t yMax;
    uint16_t yCenter;
    uint16_t yDz;
    uint16_t zMin;
    uint16_t zMax;
    uint16_t zCenter;
    uint16_t zDz;
    uint16_t rXMin;
    uint16_t rXMax;
    uint16_t rXCenter;
    uint16_t rXDz;
    uint16_t rYMin;
    uint16_t rYMax;
    uint16_t rYCenter;
    uint16_t rYDz;
    bool invertX;
    bool invertY;
    bool invertZ;
    bool invertRX;
    bool invertRY;
    bool invertRZ;
    bool circularDzXY;
};

userSettings getDefaultSettings(uint16_t CURRENT_VERSION){
    userSettings settings;
    settings.version = CURRENT_VERSION;;
    settings.xMin = 0;
    settings.xMax = 1023;
    settings.xCenter = 512;
    settings.xDz = 0;
    settings.yMin = 0;
    settings.yMax = 1023;
    settings.yCenter = 512;
    settings.yDz = 0;
    settings.zMin = 0;
    settings.zMax = 1023;
    settings.zCenter = 512;
    settings.zDz = 0;
    settings.rXMin = 0;
    settings.rXMax = 1023;
    settings.rXCenter = 512;
    settings.rXDz = 0;
    settings.rYMin = 0;
    settings.rYMax = 1023;
    settings.rYCenter = 512;
    settings.rYDz = 0;
    settings.invertX = false;
    settings.invertY = false;
    settings.invertZ = false;
    settings.invertRX = false;
    settings.invertRY = false;
    settings.invertRZ = false;
    settings.circularDzXY = false;
    return settings;
}

#endif // STRUCTS_HEADER_FILE_H