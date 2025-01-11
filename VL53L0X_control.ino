#include <DFRobot_VL53L0X.h>
#include "cobs.h"




int number_of_sensors = 4;


DFRobot_VL53L0X sensors[4];

//dummy values for now, 
//these are the digital pins 
//that need to be enabled for the lidar xshut
int pin_list[4] = {7, 7, 7, 7}; 


void setup_VL53L0X() {
  //join i2c bus (address optional for master)
  Wire.begin();
  for (int i = 0; i < number_of_sensors; i++){
    //Set I2C sub-device address
    sensors[i].begin(0x50 + i);
    //Set to Back-to-back mode and high precision mode
    sensors[i].setMode(sensors[i].eContinuous, sensors[i].eHigh);
    //Laser rangefinder begins to work
    sensors[i].start();
  }

  
 
}
/**
*@brief returns value of n lidar sensor in inches
**/
float get_value_of_VL53L0X(int n) 
{

  float mm;

  //Get the distance
  mm = sensors[n].getDistance();

  if (mm < 40){
    mm = 0;
  } 

  return (mm / 25.4);

}

void read_VL53L0X_to_brain(){

  

  uint8_t buf[4]; 
  uint8_t output_buf[40];
  
  float val = get_value_of_VL53L0X(0);

  struct cobs_encode_result result;
  
  memcpy(buf, &val, sizeof(float));


  result = cobs_encode(output_buf, 40, buf, 4); //encode the data into COBS format
   
  serial_write_to_brain_buffer(output_buf, result.out_len);
}



