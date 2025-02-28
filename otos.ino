//Install the Arduino libraries:  Sparkfun toolkit, Sparkfun OTOS library

#include "SparkFun_Qwiic_OTOS_Arduino_Library.h"
#include "Wire.h"


#define FIELD_WIDTH_INCHES 144
#define HALF_FIELD_WIDTH_INCHES 72
#define DISTANCE_BETWEEN_MID_LIDARS 10
#define WIDTH 14.5
#define LENGTH 14.75
#define PI 3.1415926535897932384626433832795
#define SIDE_LIDAR_DIST_TO_CENTER 8 //negative if toward the front of the bot

// Create an Optical Tracking Odometry Sensor object
QwiicOTOS otos;

bool OTOS_DEBUG = true;
bool DATA_DEBUG = false;

void setup_otos()
{
  
  Serial.println("Qwiic OTOS Example 1 - Basic Readings");


  Wire.begin();

  // Attempt to begin the sensor
  while (otos.begin() == false)
  {
    Serial.println("OTOS not connected, check your wiring and I2C address!");
    delay(1000);
  }

  Serial.println("OTOS connected!");

  Serial.println("Ensure the OTOS is flat and stationary to calibrate the IMU");

  Serial.println("Calibrating IMU...");

  // Calibrate the IMU, which removes the accelerometer and gyroscope offsets
  otos.calibrateImu();

  // Reset the tracking algorithm - this resets the position to the origin,
  // but can also be used to recover from some rare tracking errors
  otos.resetTracking();
  otos.setLinearScalar(1.02161f);

}

float process_side_lidar(float distance, float heading_rad){
  float opp = (SIDE_LIDAR_DIST_TO_CENTER)*std::tan(heading_rad);
  return distance + opp; //opp can be negative 
}

void offset_otos_with_lidar(bool is_red_alliance, float left_lidar, float mid_left_lidar, float mid_right_lidar, float right_lidar){
  
  sfe_otos_pose2d_t offsetPose;
  
  float mid_lidar = (mid_left_lidar + mid_right_lidar) / 2.0;
  float delta_dist = mid_right_lidar - mid_left_lidar;

  float angle = std::atan2(delta_dist, DISTANCE_BETWEEN_MID_LIDARS);
  float heading;

  if (is_red_alliance){
    heading = (angle + PI); 
  } else{
    heading = (angle);
  }
  
  
  //offsetPose.h = angle;
  float dist_to_wall;
  
  bool is_left_side;

  if (left_lidar < right_lidar){

    dist_to_wall = left_lidar;
    is_left_side = true;

  } else {

    is_left_side = false;
    dist_to_wall = right_lidar;

  }

  dist_to_wall = process_side_lidar(dist_to_wall, heading);

  float adjacent_y = dist_to_wall + (WIDTH/2.0);
  float adjacent_x = mid_lidar + (LENGTH/2.0);
  
  
  float new_x = HALF_FIELD_WIDTH_INCHES - (std::cos(angle) * adjacent_x); //width of half the field - deltaX
  float new_y = HALF_FIELD_WIDTH_INCHES - (std::cos(angle) * adjacent_y); //same but deltaY
  

  if (is_red_alliance){
    offsetPose.x = new_x;
    //heading = (angle + PI) * 180 / PI 
  } else{
    offsetPose.x = -new_x;
    //heading = (angle) * 180 / PI 
  }


  if (is_left_side){
    offsetPose.y = -new_y;
  } else{
    offsetPose.y = new_y;
  }
  offsetPose.h = heading * 180 / PI;

  reinit_otos();
  delay(5);
  otos.setPosition(offsetPose);

}

void reinit_otos(){
  otos.calibrateImu();

  otos.resetTracking();
  otos.setLinearScalar(10.0f/(10.236585f));
}


void get_otos_position(uint8_t* buf) {
  // Get the latest position, which includes the x and y coordinates, plus the
  // heading angle
  sfe_otos_pose2d_t myPosition;
  otos.getPosition(myPosition);
  
  //myPosition.h = 90;
  //myPosition.y = 20;
  //myPosition.x = 10;

  //memcpy(buf, &myPosition.x, sizeof(float));        // Copy X value to buffer
  //memcpy(buf + sizeof(float), &myPosition.y, sizeof(float)); // Copy Y value to buffer
  //memcpy(buf + (2 * sizeof(float)), &myPosition.h, sizeof(float));
  
  uint8_t* byteBuf = buf; 

  float values[] = {myPosition.x, myPosition.y, myPosition.h};
  for (size_t i = 0; i < 3; i++) {
      float* floatPtr = (float*)(byteBuf + i * sizeof(float));
      *floatPtr = values[i];
  }

  // Wait a bit so we don't spam the serial port
  //delayMicroseconds(1000);
  delay(1);
  

  // Alternatively, you can comment out the print and delay code above, and
  // instead use the following code to rapidly refresh the data
  // DID THIS
  if (true){
    Serial.print(myPosition.x);
    Serial.print("\t");
    Serial.print(myPosition.y);
    Serial.print("\t");
    Serial.println(myPosition.h);
  }
  
  
}

void set_otos_pos(float x, float y, float h){
  sfe_otos_pose2d_t position = {x, y, h};
  otos.setPosition(position);
}

void send_otos_data(){
  uint8_t temp_buf[20]; // 4 bytes each for each dimension (xyh)
  uint8_t output_buf[40];
  
  struct cobs_encode_result result;
  
  get_otos_position(temp_buf);
  
  if (OTOS_DEBUG && DATA_DEBUG){
    Serial.println("Data to send:");
    Serial.print("[");
    for (int i; i < 20; i++){
      Serial.print(temp_buf[i]);
      Serial.print(", ");
    }
    Serial.print("]");
    Serial.println();
  }
  


  result = cobs_encode(output_buf, 40, temp_buf, 20); //encode the data into COBS format
   
  serial_write_to_brain_buffer(output_buf, result.out_len);  //write the COBS packet to the VEX brain 

}




