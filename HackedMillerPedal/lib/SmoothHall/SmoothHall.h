#ifndef SMOOTH_HALL_H
#define SMOOTH_HALL_H

#include <Arduino.h>

enum FilterType {
    MOVING_AVERAGE = 1,
    EXPONENTIAL_SMOOTHING = 2,
    ADAPTIVE_SMOOTHING = 4,
    KALMAN_FILTER = 8,
    OVERSAMPLE_ONLY = 16
};

class SmoothHall {
public:
    SmoothHall(uint8_t analogPin, uint8_t numSamples = 10, float alpha = 0.2, bool enableCalibration = true,
               uint16_t calibrationSamples = 100, uint16_t oversampleCount = 10,
               float initialProcessNoise = 1.0, float initialMeasurementNoise = 0.01, float initialErrorEstimate = 1.0,
               float adaptiveMinAlpha = 0.05, float adaptiveMaxAlpha = 0.5, float adaptiveScalingFactor = 100.0);

    void calibrate();
    float read();                      // Returns value based on configured filter stack
    float readFilterType(FilterType type);  // Retrieve a specific filter's output

    int readMovingAverage();
    float readExponentialSmoothing();
    float readAdaptiveSmoothing();
    float readKalmanFilter();

    void setFilterStack(uint8_t filters);

private:
    uint8_t _analogPin;
    uint8_t _numSamples;
    float _alpha;
    bool _enableCalibration;
    uint16_t _calibrationSamples;
    uint16_t _oversampleCount;

    int *_hallReadings;
    int _bufferIndex;
    int _total;
    int _hallBaseline;

    float _expSmoothedValue;
    float _adaptiveAlpha;
    float _adaptiveMinAlpha;
    float _adaptiveMaxAlpha;
    float _adaptiveScalingFactor;

    // Kalman Variables
    float _kalmanEstimate;
    float _kalmanError;
    float _processNoise;
    float _measurementNoise;
    float _prevMeasurement;

    uint8_t _filterStack;

    int readSensor();
    int oversampleSensor();
};

#endif