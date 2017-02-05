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


#include "ComplementaryFilter.h"
#include <stdlib.h>
#include <math.h>

ComplementaryFilter::ComplementaryFilter(const uint8_t size)
{
    _size = size;
    _ar = (double*) malloc(_size * sizeof(double));
    if (_ar == NULL) _size = 0;
    clear();
}

ComplementaryFilter::~ComplementaryFilter()
{
    if (_ar != NULL) free(_ar);
}

// resets all counters
void ComplementaryFilter::clear()
{
    _cnt = 0;
    _idx = 0;
    _sum = 0.0;
    _min = NAN;
    _max = NAN;
    for (uint8_t i = 0; i < _size; i++)
    {
        _ar[i] = 0.0; // keeps addValue simpler
    }
}

// adds a new value to the data-set
void ComplementaryFilter::addValue(const double value)
{
    if (_ar == NULL) return;  // allocation error
    _sum -= _ar[_idx];
    _ar[_idx] = value;
    _sum += _ar[_idx];
    _idx++;
    if (_idx == _size) _idx = 0;  // faster than %

    // handle min max
    if (_cnt == 0) _min = _max = value;
    else if (value < _min) _min = value;
    else if (value > _max) _max = value;

    // update count as last otherwise if( _cnt == 0) above will fail
    if (_cnt < _size) _cnt++;
}

// returns the average of the data-set added sofar
double ComplementaryFilter::getAverage() const
{
    if (_cnt == 0) return NAN;
    double sum = 0;
    for (uint8_t i = 0; i < _cnt; i++)
    {
        sum += _ar[i];
    }
    return sum / _cnt;
}

double ComplementaryFilter::getFastAverage() const
{
    if (_cnt == 0) return NAN;
    return _sum / _cnt;
}

// returns the max value in the buffer
double ComplementaryFilter::GetMinInBuffer() const
{
    if (_cnt == 0) return NAN;
    double min = _ar[0];
    for (uint8_t i = 1; i < _cnt; i++)
    {
        if (min > _ar[i]) min = _ar[i];
    }
    return min;
}

double ComplementaryFilter::GetMaxInBuffer() const
{
    if (_cnt == 0) return NAN;
    double max = _ar[0];
    for (uint8_t i = 1; i < _cnt; i++)
    {
        if (max < _ar[i]) max = _ar[i];
    }
    return max;
}

// return true if buffer is full
bool ComplementaryFilter::BufferIsFull() const
{
    if (_cnt == _size) return true;
    return false;
}

// returns the value of an element if exist, NAN otherwise
double ComplementaryFilter::getElement(uint8_t idx) const
{
    if (idx >=_cnt ) return NAN;
    return _ar[idx];
}

// Return standard deviation of running average. If buffer is empty, return NAN.
double ComplementaryFilter::GetStandardDeviation() const
{
	if (_cnt == 0) return NAN;
    double temp = 0;
	double average = getFastAverage();
    for (uint8_t i = 0; i < _cnt; i++)
    {
		temp += pow((_ar[i] - average),2);
    }
	temp = sqrt(temp/(_cnt - 1));
    return temp;
}

// Return standard error of running average. If buffer is empty, return NAN.
double ComplementaryFilter::GetStandardError() const //++
{
	double temp = GetStandardDeviation();
	if(temp==NAN) return NAN;
	double n;
	if(_cnt>=30) n= _cnt;
	else n= _cnt - 1;
	temp = temp/sqrt(n);
	return temp;
}

// fill the average with a value
// the param number determines how often value is added (weight)
// number should preferably be between 1 and size
void ComplementaryFilter::fillValue(const double value, const uint8_t number)
{
    clear(); // TODO conditional?  if (clr) clear();

    for (uint8_t i = 0; i < number; i++)
    {
        addValue(value);
    }
}
// END OF FILE
