#include "SmoothHall.h"

SmoothHall::SmoothHall(uint8_t analogPin, uint8_t numSamples, float alpha, bool enableCalibration,
                       uint16_t calibrationSamples, uint16_t oversampleCount,
                       float initialProcessNoise, float initialMeasurementNoise, float initialErrorEstimate,
                       float adaptiveMinAlpha, float adaptiveMaxAlpha, float adaptiveScalingFactor)
    : _analogPin(analogPin), _numSamples(numSamples), _alpha(alpha), _enableCalibration(enableCalibration),
      _calibrationSamples(calibrationSamples), _oversampleCount(oversampleCount),
      _bufferIndex(0), _total(0), _hallBaseline(0), _expSmoothedValue(0),
      _adaptiveAlpha(alpha), _adaptiveMinAlpha(adaptiveMinAlpha), _adaptiveMaxAlpha(adaptiveMaxAlpha),
      _adaptiveScalingFactor(adaptiveScalingFactor), _kalmanEstimate(0),
      _kalmanError(initialErrorEstimate), _processNoise(initialProcessNoise), _measurementNoise(initialMeasurementNoise),
      _prevMeasurement(0), _filterStack(EXPONENTIAL_SMOOTHING) {

    _hallReadings = new int[_numSamples]();
}

// Reads sensor value from the analog input pin
int SmoothHall::readSensor() {
    return analogRead(_analogPin);
}

// Oversampling: Take multiple readings and average them
int SmoothHall::oversampleSensor() {
    long sum = 0;
    for (uint16_t i = 0; i < _oversampleCount; i++) {
        sum += readSensor();
        delayMicroseconds(50);  // Small delay to reduce ADC noise
    }
    return sum / _oversampleCount;
}

// Calibration or Oversampling
void SmoothHall::calibrate() {
    if (_enableCalibration == true) {
        long baselineSum = 0;
        Serial.println("Calibrating... Ensure no magnetic field is present.");
        for (uint16_t i = 0; i < _calibrationSamples; i++) {
            baselineSum += readSensor();
            delay(10);
        }
        _hallBaseline = baselineSum / _calibrationSamples;
        Serial.print("Calibration complete! Baseline: ");
        Serial.println(_hallBaseline);
    } else if(_enableCalibration == false) {
        Serial.println("Calibration bypassed. Using oversampling instead.");
        _hallBaseline = oversampleSensor();
    }

    for (uint8_t i = 0; i < _numSamples; i++) {
        _hallReadings[i] = readSensor() - _hallBaseline;
        _total += _hallReadings[i];
    }

    _expSmoothedValue = _total / _numSamples;
    _kalmanEstimate = _expSmoothedValue;
}

void SmoothHall::setFilterStack(uint8_t filters) {
    _filterStack = filters;
}

int SmoothHall::readMovingAverage() {
    int rawValue = readSensor();
    int normalizedValue = rawValue - _hallBaseline;

    _total -= _hallReadings[_bufferIndex];
    _hallReadings[_bufferIndex] = normalizedValue;
    _total += normalizedValue;
    _bufferIndex = (_bufferIndex + 1) % _numSamples;

    return _total / _numSamples;
}

float SmoothHall::readExponentialSmoothing() {
    int rawValue = readSensor();
    int normalizedValue = rawValue - _hallBaseline;

    _expSmoothedValue = (_alpha * normalizedValue) + ((1 - _alpha) * _expSmoothedValue);

    return _expSmoothedValue;
}

// Adaptive Smoothing: Dynamically adjusts alpha
float SmoothHall::readAdaptiveSmoothing() {
    int rawValue = readSensor();
    int normalizedValue = rawValue - _hallBaseline;
    float delta = abs(normalizedValue - _expSmoothedValue);

    _adaptiveAlpha = constrain(delta / _adaptiveScalingFactor, _adaptiveMinAlpha, _adaptiveMaxAlpha);
    _expSmoothedValue = (_adaptiveAlpha * normalizedValue) + ((1 - _adaptiveAlpha) * _expSmoothedValue);

    return _expSmoothedValue;
}

// Kalman Filter with Auto-Tuning
float SmoothHall::readKalmanFilter() {
    int rawValue = readSensor();
    int normalizedValue = rawValue - _hallBaseline;

    float measurementDelta = abs(normalizedValue - _prevMeasurement);
    _processNoise = constrain(measurementDelta * 0.01, 0.0001, 2.0);

    float kalmanGain = _kalmanError / (_kalmanError + _measurementNoise);
    _kalmanEstimate = _kalmanEstimate + kalmanGain * (normalizedValue - _kalmanEstimate);

    _kalmanError = (1 - kalmanGain) * _kalmanError + _processNoise;
    _prevMeasurement = normalizedValue;

    return _kalmanEstimate;
}
// Read using the configured filter stack
float SmoothHall::read() {
    float value;
    if (_enableCalibration) {
        value = readSensor() - _hallBaseline;
    } else {
        value = oversampleSensor() - _hallBaseline;
    }

    if (_filterStack & MOVING_AVERAGE) {
        value = readMovingAverage();
    }
    if (_filterStack & EXPONENTIAL_SMOOTHING) {
        value = readExponentialSmoothing();
    }
    if (_filterStack & ADAPTIVE_SMOOTHING) {
        value = readAdaptiveSmoothing();
    }
    if (_filterStack & KALMAN_FILTER) {
        value = readKalmanFilter();
    }

    return value;
}

// Get a specific filter type
float SmoothHall::readFilterType(FilterType type) {
    switch (type) {
        case MOVING_AVERAGE:
            return readMovingAverage();
        case EXPONENTIAL_SMOOTHING:
            return readExponentialSmoothing();
        case ADAPTIVE_SMOOTHING:
            return readAdaptiveSmoothing();
        case KALMAN_FILTER:
            return readKalmanFilter();
        case OVERSAMPLE_ONLY:
            return oversampleSensor();
        default:
            return read();  // Default to the configured filter stack
    }
}