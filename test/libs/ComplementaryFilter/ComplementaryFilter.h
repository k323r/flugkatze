/*
#define ACCELEROMETER_SENSITIVITY 8192.0
#define GYROSCOPE_SENSITIVITY 65.536

#define M_PI 3.14159265359

#define dt 0.01							// 10 ms sample rate!

void ComplementaryFilter(short accData[3], short gyrData[3], float *pitch, float *roll)
{
    float pitchAcc, rollAcc;

    // Integrate the gyroscope data -> int(angularSpeed) = angle
    *pitch += ((float)gyrData[0] / GYROSCOPE_SENSITIVITY) * dt; // Angle around the X-axis
    *roll -= ((float)gyrData[1] / GYROSCOPE_SENSITIVITY) * dt;    // Angle around the Y-axis

    // Compensate for drift with accelerometer data if !bullshit
    // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
    int forceMagnitudeApprox = abs(accData[0]) + abs(accData[1]) + abs(accData[2]);
    if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
    {
	// Turning around the X axis results in a vector on the Y-axis
        pitchAcc = atan2f((float)accData[1], (float)accData[2]) * 180 / M_PI;
        *pitch = *pitch * 0.98 + pitchAcc * 0.02;

	// Turning around the Y axis results in a vector on the X-axis
        rollAcc = atan2f((float)accData[0], (float)accData[2]) * 180 / M_PI;
        *roll = *roll * 0.98 + rollAcc * 0.02;
    }
}
*/

#ifndef ComplementaryFilter_h
#define ComplementaryFilter_h

#define COMPLEMENTARY_LIB_VERSION "0.0.1"

#define M_PI 3.14159265359

#include "Arduino.h"

class ComplementaryFilter
{
public:
    ComplementaryFilter(void);
    explicit ComplementaryFilter(const uint8_t);
    ~ComplementaryFilter();

    void clear();
    void addValue(const double);
    void fillValue(const double, const uint8_t);

    double getAverage() const;      // does iterate over all elements.

	// return statistical characteristics of the running average
    double GetStandardDeviation() const;
	double GetStandardError() const;

	// returns min/max added to the data-set since last clear
    double getMin() const { return _min; };
    double getMax() const { return _max; };

	// returns min/max from the values in the internal buffer
    double GetMinInBuffer() const;
    double GetMaxInBuffer() const;

	// return true if buffer is full
	bool BufferIsFull() const;

    double getElement(uint8_t idx) const;

    uint8_t getSize() const { return _size; }
    uint8_t getCount() const { return _cnt; }



protected:
    float _weightA;
    float _weightB;
};

#endif
// END OF FILE
