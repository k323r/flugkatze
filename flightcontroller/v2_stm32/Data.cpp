#include "Data.h"

Data::Data() {
  
}

void Data::send(){
  Serial2.write('S');                    // starting byte to ensure data integrity
  Serial2.write((uint8_t *) &ax, sizeof(ax));
  Serial2.write((uint8_t *) &ay, sizeof(ay)); 
  Serial2.write((uint8_t *) &az, sizeof(az));
  Serial2.write((uint8_t *) &roll, sizeof(roll));
  Serial2.write((uint8_t *) &pitch, sizeof(pitch));
  Serial2.write((uint8_t *) &yaw, sizeof(yaw));
  Serial2.write('E');                    // end byte to ensure data integrity
}
