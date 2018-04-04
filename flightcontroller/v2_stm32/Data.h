#ifndef _DATA_H_
#define _DATA_H_
#endif

#pragma once

#include <Wire.h>

class Data {
    public:
      Data();

      void send();
      
      float ax = 0;
      float ay = 0;
      float az = 0;

      float temp = 0;

      float roll = 0;
      float pitch = 0;
      float yaw = 0;
};
