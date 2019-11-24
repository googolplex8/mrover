#include "ICM_20948.h"  // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include <MadgwickAHRS.h>

Madgwick filterIMU;
Madgwick filterMAG;

//#define USE_SPI       // Uncomment this to use SPI

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
  filterMAG.begin(30);
  filterIMU.begin(30);
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

    SERIAL_PORT.print( F("Initialization of the sensor returned: ") );
    SERIAL_PORT.println( myICM.statusString() );
    if( myICM.status != ICM_20948_Stat_Ok ){
      SERIAL_PORT.println( "Trying again..." );
      delay(500);
    }else{
      initialized = true;
    }
  }
}

void loop() {
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float rollIMU, pitchIMU, headingIMU;
  float rollMAG, pitchMAG, headingMAG;
  
  if( myICM.dataReady() ){
    myICM.getAGMT();                // The values are only updated when you call 'getAGMT'
//  printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
//  printScaledAGMT( myICM.agmt);   // This function takes into account the scale settings from when the measurement was made to calculate the values with units

    //raw data in gravity and degrees/second
    ax = myICM.accX()*0.001;
    ay = myICM.accY()*0.001;
    az = myICM.accZ()*0.001;
    gx = myICM.gyrX();
    gy = myICM.gyrX();
    gz = myICM.gyrX();
    mx = myICM.magX();
    my = myICM.magY();
    mz = myICM.magZ();
     
    // update the filter, which computes orientation
    filterIMU.updateIMU(gx, gy, gz, ax, ay, az);
    filterMAG.update(gx, gy, gz, ax, ay, az, mx, my, mz);

    // print the heading, pitch and roll for IMU
    rollIMU = filterIMU.getRoll();
    pitchIMU = filterIMU.getPitch();
    headingIMU = filterIMU.getYaw();
    Serial.print("Orientation (IMU): ");
    Serial.print(headingIMU);
    Serial.print(" ");
    Serial.print(pitchIMU);
    Serial.print(" ");
    Serial.println(rollIMU);

    // print the heading, pitch and roll for MAG
    rollMAG = filterMAG.getRoll();
    pitchMAG = filterMAG.getPitch();
    headingMAG = filterMAG.getYaw();
    Serial.print("Orientation (MAG): ");
    Serial.print(headingMAG);
    Serial.print(" ");
    Serial.print(pitchMAG);
    Serial.print(" ");
    Serial.println(rollMAG);
    Serial.println();

    delay(30);
    //originally 30
  }else{
    Serial.println("Waiting for data");
    delay(500);
  }
  
}
