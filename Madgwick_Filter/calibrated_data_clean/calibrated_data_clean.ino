/****************************************************************

 ***************************************************************/
#include "ICM_20948.h"
#include <MadgwickAHRS.h>

Madgwick filter;

// Mag calibration values are calculated via calibration sketch results.
// These values must be determined for each baord/environment.

// Offsets applied to raw x/y/z mag values
float mag_offsets[3]            = { 3.27F, 11.04F, -14.24F };

// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { {  0.8936,  -0.028,  -0.001 },
                                    {  -0.028,  1.068, 0.022 },
                                    {  -0.001,  0.022,  1.002 } };

float mag_field_strength        = 52.08F;

// Offsets applied to compensate for gyro zero-drift error for x/y/z
float gyro_zero_offsets[3]      = { 0.0F, 0.0F, 0.0F };

//
//#define USE_SPI       // Uncomment this to use SPI
//

#define SERIAL_PORT Serial

#define SPI_PORT SPI    // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 2        // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT Wire  // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL   1     // The value of the last bit of the I2C address. 
                        // On the SparkFun 9DoF IMU breakout the default is 1, and when 
                        // the ADR jumper is closed the value becomes 0

#ifdef USE_SPI
  ICM_20948_SPI myICM;  // If using SPI create an ICM_20948_SPI object
#else
  ICM_20948_I2C myICM;  // Otherwise create an ICM_20948_I2C object
#endif


void setup() {
  filter.begin(30);
  SERIAL_PORT.begin(115200);
  while(!SERIAL_PORT){};

#ifdef USE_SPI
    SPI_PORT.begin();
#else
    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);
#endif
  
  bool initialized = false;
  while( !initialized ){

#ifdef USE_SPI
    myICM.begin( CS_PIN, SPI_PORT ); 
#else
    myICM.begin( WIRE_PORT, AD0_VAL );
#endif

    //SERIAL_PORT.print( F("Initialization of the sensor returned: ") );
    //SERIAL_PORT.println( myICM.statusString() );
    if( myICM.status != ICM_20948_Stat_Ok ){
      //SERIAL_PORT.println( "Trying again..." );
      delay(500);
    }else{
      initialized = true;
    }
  }
}

void loop() {
  
if( myICM.dataReady() ){
    myICM.getAGMT();

  // Apply mag offset compensation (base values in uTesla)
  float x = myICM.magX() - mag_offsets[0];
  float y = myICM.magY() - mag_offsets[1];
  float z = myICM.magZ() - mag_offsets[2];

  // Apply mag soft iron error compensation
  float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
  float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
  float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

  // Apply gyro zero-rate error compensation
  float gx = myICM.gyrX() + gyro_zero_offsets[0];
  float gy = myICM.gyrY() + gyro_zero_offsets[1];
  float gz = myICM.gyrX() + gyro_zero_offsets[2];

  // Update the filter
  filter.update(gx, gy, gz, myICM.accX()*0.001, myICM.accY()*0.001, myICM.accZ()*0.001, mx, my, mz);

  // Print the orientation filter output
  // Note: To avoid gimbal lock you should read quaternions not Euler
  // angles, but Euler angles are used here since they are easier to
  // understand looking at the raw values. See the ble fusion sketch for
  // and example of working with quaternion data.
  float roll = filter.getRoll();
  float pitch = filter.getPitch();
  float yaw = filter.getYaw();
  /*
  roll *= 180.0f / PI;
  pitch *= 180.0f / PI;
  yaw *= 180.0f / PI;
  
  if(roll<0){
    roll+=360;
  }
  if(pitch<0){
    pitch+=360;
  }
  if(yaw<0){
    yaw+=360;
  }
  */
  //roll = 360-((360/(roll+360))*360);
  //pitch = 360-((360/(pitch+360))*360);
  //yaw = 360-((360/(yaw+360))*360);
  
  yaw -= 7; //magnetic declination in GGBrown is 7 degrees and 0 minutes
  Serial.print(millis());
  //Serial.print(" - Orientation: ");
  Serial.print(" ");
  Serial.print(roll);
  Serial.print(" ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.println(yaw);

  delay(10);
}else{
    //Serial.println("Waiting for data");
    delay(500);
  }
}
