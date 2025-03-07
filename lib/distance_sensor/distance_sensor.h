#ifndef DISTANCE_SENSOR_H
#define DISTANCE_SENSOR_H

#include <Arduino.h>
#define SAMPLES 25

class DistanceSensor {
public:
    DistanceSensor(uint8_t SensorPin, long SensorModel);
    int getDistance();
private:
    uint8_t _sensorPin;
    long _model;
    void sort(int a[], int size);
};

#endif