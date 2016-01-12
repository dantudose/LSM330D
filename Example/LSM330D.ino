/*****************************************************************
LSM330D_Simple.ino
LSM330D Library Simple Example Code
by Dan Tudose
Original Creation Date: January 10, 2016

The LSM330D is a versatile 6DOF sensor. It has a built-in
accelerometer, gyroscope, and temperature sensor. Very cool! Plus it
functions over either SPI or I2C.

This Arduino sketch is a demo of the simple side of the
LSM330D library. It'll demo the following:
* How to create a LSM330D object, using a constructor (global
  variables section).
* How to use the begin() function of the LSM330D class.
* How to read the gyroscope and accelerometer
  using the readGryo(), readAccel() functions and the
  gx, gy, gz, ax, ay, az variables.
* How to calculate actual acceleration, rotation speed 
  field strength using the calcAccel() and calcGyro()
  functions.
* How to use the data from the LSM330D to calculate orientation
  and heading.

Hardware setup: This library supports communicating with the
LSM330D over either I2C or SPI. If you're using I2C, these are
the only connections that need to be made:
	LSM330D --------- Arduino
	 SCL ---------- SCL (A5 on older 'Duinos')
	 SDA ---------- SDA (A4 on older 'Duinos')
	 VDD ------------- 3.3V
	 GND ------------- GND
(CSG, CSA, DEN_G, and SDO_G, SDO_A should all be pulled high.)
  
If you're using SPI, here is an example hardware setup:
	      LSM330D --------- Arduino
          CSG -------------- 9
          CSA ------------- 10
          SDOG ------------- 12
          SDOA ------------ 12 (tied to SDOG)
          SCL -------------- 13
          SDA -------------- 11
          VDD -------------- 3.3V
          GND -------------- GND
	
The LSM330D has a maximum voltage of 3.6V. Make sure you power it
off the 3.3V rail! And either use level shifters between SCL
and SDA or just use a 3.3V Arduino Pro.	  

Development environment specifics:
	IDE: Arduino 1.0.5
	Hardware Platform: Arduino Pro 3.3V/8MHz
	
This code is beerware. If you see me at the local, and you've found our code helpful, 
please buy me a round!

Distributed as-is; no warranty is given.
*****************************************************************/

// The LSM330D requires both the SPI and Wire libraries.
// Unfortunately, you'll need to include both in the Arduino
// sketch, before including the LSM330D library.

#include <Wire.h>
#include <LSM330D.h>

///////////////////////
// Example I2C Setup //
///////////////////////
// Comment out this section if you're using SPI
// SDO_A and SDO_G are both high, so our addresses are:
#define LSM330D_A  LSM330D_A_ADDR1 
#define LSM330D_G  LSM330D_G_ADDR1 
// Create an instance of the LSM330D library called `dof` the
// parameters for this constructor are:
// [SPI or I2C Mode declaration],[gyro I2C address],[accel I2C add.]
LSM330D dof(MODE_I2C, LSM330D_G, LSM330D_A);


// Do you want to print calculated values or raw ADC ticks read
// from the sensor? Comment out ONE of the two #defines below
// to pick:
#define PRINT_CALCULATED
//#define PRINT_RAW

#define PRINT_SPEED 500 // 500 ms between prints

void setup()
{
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);
  delay(3000);
  
  Serial.begin(115200); // Start serial at 115200 bps
  // Use the begin() function to initialize the LSM330D library.
  // You can either call it with no parameters (the easy way):
  uint16_t status = dof.begin();
  // Or call it with declarations for sensor scales and data rates:  
  //uint16_t status = dof.begin(dof.G_SCALE_2000DPS, 
  //                            dof.A_SCALE_6G);
  
  // begin() returns a 16-bit value which includes both the gyro 
  // and accelerometers WHO_AM_I response. You can check this to
  // make sure communication was successful.
  Serial.print("LSM330D WHO_AM_I's returned: 0x");
  Serial.println(status, HEX);
  Serial.println("Should be 0x00D4");
  Serial.println();
}

void loop()
{
  
  printAccel(); // Print "A: ax, ay, az"
  printGyro();  // Print "G: gx, gy, gz"
  
  // Print the heading and orientation for fun!
  printOrientation(dof.calcAccel(dof.ax), dof.calcAccel(dof.ay), dof.calcAccel(dof.az));
  Serial.println();
  
  delay(PRINT_SPEED);
}

void printGyro()
{
  // To read from the gyroscope, you must first call the
  // readGyro() function. When this exits, it'll update the
  // gx, gy, and gz variables with the most current data.
  dof.readGyro();
  
  // Now we can use the gx, gy, and gz variables as we please.
  // Either print them as raw ADC values, or calculated in DPS.
  Serial.print("G: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcGyro helper function to convert a raw ADC value to
  // DPS. Give the function the value that you want to convert.
  Serial.print(dof.calcGyro(dof.gx), 2);
  Serial.print(", ");
  Serial.print(dof.calcGyro(dof.gy), 2);
  Serial.print(", ");
  Serial.println(dof.calcGyro(dof.gz), 2);
#elif defined PRINT_RAW
  Serial.print(dof.gx);
  Serial.print(", ");
  Serial.print(dof.gy);
  Serial.print(", ");
  Serial.println(dof.gz);
#endif
}

void printAccel()
{
  // To read from the accelerometer, you must first call the
  // readAccel() function. When this exits, it'll update the
  // ax, ay, and az variables with the most current data.
  dof.readAccel();
  
  // Now we can use the ax, ay, and az variables as we please.
  // Either print them as raw ADC values, or calculated in g's.
  Serial.print("A: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcAccel helper function to convert a raw ADC value to
  // g's. Give the function the value that you want to convert.
  Serial.print(dof.calcAccel(dof.ax), 2);
  Serial.print(", ");
  Serial.print(dof.calcAccel(dof.ay), 2);
  Serial.print(", ");
  Serial.println(dof.calcAccel(dof.az), 2);
#elif defined PRINT_RAW 
  Serial.print(dof.ax);
  Serial.print(", ");
  Serial.print(dof.ay);
  Serial.print(", ");
  Serial.println(dof.az);
#endif

}


// A fun function that does calculations based on the
// acclerometer data. This function will print your LSM330D's
// orientation -- it's roll and pitch angles.
void printOrientation(float x, float y, float z)
{
  float pitch, roll;
  
  pitch = atan2(x, sqrt(y * y) + (z * z));
  roll = atan2(y, sqrt(x * x) + (z * z));
  pitch *= 180.0 / PI;
  roll *= 180.0 / PI;
  
  Serial.print("Pitch, Roll: ");
  Serial.print(pitch, 2);
  Serial.print(", ");
  Serial.println(roll, 2);
}
