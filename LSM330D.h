#ifndef __LSM330D_H__
#define __LSM330D_H__

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
  #include "pins_arduino.h"
#endif


// LSM330D I2C address
// Accelerometer = 001100xb
#define LSM330D_A_ADDR0     0b00011000  // SA0=0(GND)
#define LSM330D_A_ADDR1     0b00011001  // SA0=1(VDD)

// gyro = 110101xb
#define LSM330D_G_ADDR0     0b01101010  // SA0=0(GND)
#define LSM330D_G_ADDR1     0b01101011  // SA0=1(VDD)

// LSM330D Register Map
// Accelerometer registers
#define CTRL_REG1_A     0x20
#define CTRL_REG2_A     0x21
#define CTRL_REG3_A     0x22
#define CTRL_REG4_A     0x23
#define CTRL_REG5_A     0x24

#define OUT_X_L_A       0x28
#define OUT_X_H_A       0x29
#define OUT_Y_L_A       0x2A
#define OUT_Y_H_A       0x2B
#define OUT_Z_L_A       0x2C
#define OUT_Z_H_A       0x2D


#define CTRL_REG6_A 	0x25
#define REFERENCE_A 	0x26
#define STATUS_REG_A	0x27

#define FIFO_CTRL_REG 	0x2E
#define FIFO_SRC_REG 	0x2F
#define INT1_CFG_A 		0x30
#define INT1_SOURCE_A 	0x31
#define INT1_THS_A 		0x32
#define INT1_DURATION_A 0x33
#define INT2_CFG_A 		0x34
#define INT2_SOURCE_A 	0x35
#define INT2_THS_A 		0x36
#define INT2_DURATION_A 0x37
#define CLICK_CFG_A 	0x38
#define CLICK_SRC_A 	0x39
#define CLICK_THS_A 	0x3A
#define TIME_LIMIT_A 	0x3B


#define TIME_LATENCY_A 	0x3C
#define TIME_WINDOW_A 	0x3D
#define Act_THS 		0x3E
#define Act_DUR 		0x3F



// Gyroscope registers
#define WHO_AM_I_G      0x0F
#define WHOAMI_G_VAL    0xD4

#define CTRL_REG1_G     0x20
#define CTRL_REG2_G     0x21
#define CTRL_REG3_G     0x22
#define CTRL_REG4_G     0x23
#define CTRL_REG5_G     0x24

#define REFERENCE_G		0x25
#define OUT_TEMP_G		0x26
#define STATUS_REG_G 	0x27

#define OUT_X_L_G       0x28
#define OUT_X_H_G       0x29
#define OUT_Y_L_G       0x2A
#define OUT_Y_H_G       0x2B
#define OUT_Z_L_G       0x2C
#define OUT_Z_H_G       0x2D

#define FIFO_CTRL_REG_G 0x2E
#define FIFO_SRC_REG_G	0x2F
#define INT1_CFG_G		0x30
#define INT1_SRC_G 		0x31
#define INT1_TSH_XH_G 	0x32
#define INT1_TSH_XL_G 	0x33
#define INT1_TSH_YH_G 	0x34
#define INT1_TSH_YL_G 	0x35
#define INT1_TSH_ZH_G 	0x36
#define INT1_TSH_ZL_G 	0x37
#define INT1_DURATION_G 0x38

// The LSM330D functions over both I2C or SPI. This library supports both.
// But the interface mode used must be sent to the LSM330D constructor. Use
// one of these two as the first parameter of the constructor.
//#define MODE_I2C 1
//#define interface_mode MODE_I2C

enum interface_mode
{
	MODE_SPI,
	MODE_I2C,
};


class LSM330D
{
public:
  
   
	enum gyro_scale
	{
		G_SCALE_250DPS,		// 00:  250 degrees per second
		G_SCALE_500DPS,		// 01:  500 dps
		G_SCALE_2000DPS	// 10:  2000 dps
	};
	// accel_scale defines all possible FSR's of the accelerometer:
	enum accel_scale
	{
		A_SCALE_2G,	// 00:  2g
		A_SCALE_4G,	// 01:  4g
		A_SCALE_8G,	// 10:  8g
		A_SCALE_16G	// 11:  16g
	};
	// gyro_odr defines all possible data rate/bandwidth combos of the gyro:
	enum gyro_odr
	{							// ODR (Hz) --- Cutoff
		G_ODR_95_BW_125  = 0x0, //   95         12.5
		G_ODR_95_BW_25   = 0x1, //   95          25
		// 0x2 and 0x3 define the same data rate and bandwidth
		G_ODR_190_BW_125 = 0x4, //   190        12.5
		G_ODR_190_BW_25  = 0x5, //   190         25
		G_ODR_190_BW_50  = 0x6, //   190         50
		G_ODR_190_BW_70  = 0x7, //   190         70
		G_ODR_380_BW_20  = 0x8, //   380         20
		G_ODR_380_BW_25  = 0x9, //   380         25
		G_ODR_380_BW_50  = 0xA, //   380         50
		G_ODR_380_BW_100 = 0xB, //   380         100
		G_ODR_760_BW_30  = 0xC, //   760         30
		G_ODR_760_BW_35  = 0xD, //   760         35
		G_ODR_760_BW_50  = 0xE, //   760         50
		G_ODR_760_BW_100 = 0xF //   760         100
	};
	// accel_oder defines all possible output data rates of the accelerometer:
	enum accel_odr
	{
		A_POWER_DOWN, 	// Power-down mode (0x0)
		A_ODR_1,		// 1 Hz	(0x1)
		A_ODR_10,		// 10 Hz (0x2)
		A_ODR_25,		// 25 Hz (0x3)
		A_ODR_50,		// 50 Hz (0x4)
		A_ODR_100,		// 100 Hz (0x5)
		A_ODR_200,		// 200 Hz (0x6)
		A_ODR_400,		// 400 Hz (0x7)
		A_ODR_1620,		// 1620 Hz (0x9)
		A_ODR_5376		// 5376 Hz (0x9)
	};

    
	
	// We'll store the gyro, accel, and magnetometer readings in a series of
	// public class variables. Each sensor gets three variables -- one for each
	// axis. Call readGyro(), readAccel(), and readMag() first, before using
	// these variables!
	// These values are the RAW signed 16-bit readings from the sensors.
	int16_t gx, gy, gz; // x, y, and z axis readings of the gyroscope
	int16_t ax, ay, az; // x, y, and z axis readings of the accelerometer
	int16_t temperature;
	
	float abias[3];
    float gbias[3];

	// LSM9DS0 -- LSM9DS0 class constructor
	// The constructor will set up a handful of private variables, and set the
	// communication mode as well.
	// Input:
	//	- interface = Either MODE_SPI or MODE_I2C, whichever you're using
	//				to talk to the IC.
	//	- gAddr = If MODE_I2C, this is the I2C address of the gyroscope.
	// 				If MODE_SPI, this is the chip select pin of the gyro (CSG)
	//	- xmAddr = If MODE_I2C, this is the I2C address of the accel.
	//				If MODE_SPI, this is the cs pin of the accel (CSXM)
	LSM330D(interface_mode interface, uint8_t gAddr, uint8_t xmAddr);
	
	// begin() -- Initialize the gyro, accelerometer, and magnetometer.
	// This will set up the scale and output rate of each sensor. It'll also
	// "turn on" every sensor and every axis of every sensor.
	// Input:
	//	- gScl = The scale of the gyroscope. This should be a gyro_scale value.
	//	- aScl = The scale of the accelerometer. Should be a accel_scale value.
	//	- gODR = Output data rate of the gyroscope. gyro_odr value.
	//	- aODR = Output data rate of the accelerometer. accel_odr value.

	// Output: The function will return an unsigned 16-bit value. The most-sig
	//		bytes of the output are the WHO_AM_I reading of the accel. The
	//		least significant two bytes are the WHO_AM_I reading of the gyro.
	// All parameters have a defaulted value, so you can call just "begin()".
	// Default values are FSR's of:  245DPS, 2g, 2Gs; ODRs of 95 Hz for 
	// gyro, 100 Hz for accelerometer.
	// Use the return value of this function to verify communication.
	uint16_t begin(gyro_scale gScl = G_SCALE_250DPS, 
				accel_scale aScl = A_SCALE_2G,
				gyro_odr gODR = G_ODR_95_BW_125, 
				accel_odr aODR = A_ODR_50);
	
	// readGyro() -- Read the gyroscope output registers.
	// This function will read all six gyroscope output registers.
	// The readings are stored in the class' gx, gy, and gz variables. Read
	// those _after_ calling readGyro().
	void readGyro();
	
	// readAccel() -- Read the accelerometer output registers.
	// This function will read all six accelerometer output registers.
	// The readings are stored in the class' ax, ay, and az variables. Read
	// those _after_ calling readAccel().
	void readAccel();
	
	// readTemp() -- Read the temperature output register.
	// This function will read two temperature output registers.
	// The combined readings are stored in the class' temperature variables. Read
	// those _after_ calling readTemp().
	void readTemp();
	
	// calcGyro() -- Convert from RAW signed 16-bit value to degrees per second
	// This function reads in a signed 16-bit value and returns the scaled
	// DPS. This function relies on gScale and gRes being correct.
	// Input:
	//	- gyro = A signed 16-bit raw reading from the gyroscope.
	float calcGyro(int16_t gyro);
	
	// calcAccel() -- Convert from RAW signed 16-bit value to gravity (g's).
	// This function reads in a signed 16-bit value and returns the scaled
	// g's. This function relies on aScale and aRes being correct.
	// Input:
	//	- accel = A signed 16-bit raw reading from the accelerometer.
	float calcAccel(int16_t accel);
	
	// setGyroScale() -- Set the full-scale range of the gyroscope.
	// This function can be called to set the scale of the gyroscope to 
	// 245, 500, or 200 degrees per second.
	// Input:
	// 	- gScl = The desired gyroscope scale. Must be one of three possible
	//		values from the gyro_scale enum.
	void setGyroScale(gyro_scale gScl);
	
	// setAccelScale() -- Set the full-scale range of the accelerometer.
	// This function can be called to set the scale of the accelerometer to
	// 2, 4, 6, 8, or 16 g's.
	// Input:
	// 	- aScl = The desired accelerometer scale. Must be one of five possible
	//		values from the accel_scale enum.
	void setAccelScale(accel_scale aScl);
	
	// setGyroODR() -- Set the output data rate and bandwidth of the gyroscope
	// Input:
	//	- gRate = The desired output rate and cutoff frequency of the gyro.
	//		Must be a value from the gyro_odr enum (check above, there're 14).
	void setGyroODR(gyro_odr gRate);
	
	// setAccelODR() -- Set the output data rate of the accelerometer
	// Input:
	//	- aRate = The desired output rate of the accel.
	//		Must be a value from the accel_odr enum (check above, there're 11).
	void setAccelODR(accel_odr aRate); 	

    
	// configGyroInt() -- Configure the gyro interrupt output.
	// Triggers can be set to either rising above or falling below a specified
	// threshold. This function helps setup the interrupt configuration and 
	// threshold values for all axes.
	// Input:
	//	- int1Cfg = A 8-bit value that is sent directly to the INT1_CFG_G
	//		register. This sets AND/OR and high/low interrupt gen for each axis
	//	- int1ThsX = 16-bit interrupt threshold value for x-axis
	//	- int1ThsY = 16-bit interrupt threshold value for y-axis
	//	- int1ThsZ = 16-bit interrupt threshold value for z-axis
	//	- duration = Duration an interrupt holds after triggered. This value
	// 		is copied directly into the INT1_DURATION_G register.
	// Before using this function, read about the INT1_CFG_G register and
	// the related INT1* registers in the LMS9DS0 datasheet.
	void configGyroInt(uint8_t int1Cfg, uint16_t int1ThsX = 0,
						  uint16_t int1ThsY = 0, uint16_t int1ThsZ = 0, 
						  uint8_t duration = 0);


    void calLSM330D(float gbias[3], float abias[3]);

 private:
  
	void writeReg(uint8_t addr, uint8_t reg, uint8_t value);
	void readReg(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
  
 
	// xmAddress and gAddress store the I2C address or SPI chip select pin
	// for each sensor.
	uint8_t xmAddress, gAddress;
	// interfaceMode keeps track of whether we're using SPI or I2C to talk
	interface_mode interfaceMode;
	
	// gScale, aScale store the current scale range for each 
	// sensor. Should be updated whenever that value changes.
	gyro_scale gScale;
	accel_scale aScale;
	
	// gRes, aRes store the current resolution for each sensor. 
	// Units of these values would be DPS (or g's or Gs's) per ADC tick.
	// This value is calculated as (sensor scale) / (2^15).
	float gRes, aRes;
	
	// initGyro() -- Sets up the gyroscope to begin reading.
	// This function steps through all five gyroscope control registers.
	// Upon exit, the following parameters will be set:
	//	- CTRL_REG1_G = 0x0F: Normal operation mode, all axes enabled. 
	//		95 Hz ODR, 12.5 Hz cutoff frequency.
	//	- CTRL_REG2_G = 0x00: HPF set to normal mode, cutoff frequency
	//		set to 7.2 Hz (depends on ODR).
	//	- CTRL_REG3_G = 0x88: Interrupt enabled on INT_G (set to push-pull and
	//		active high). Data-ready output enabled on DRDY_G.
	//	- CTRL_REG4_G = 0x00: Continuous update mode. Data LSB stored in lower
	//		address. Scale set to 245 DPS. SPI mode set to 4-wire.
	//	- CTRL_REG5_G = 0x00: FIFO disabled. HPF disabled.
	void initGyro();
	
	// initAccel() -- Sets up the accelerometer to begin reading.
	// This function steps through all accelerometer related control registers.
	// Upon exit these registers will be set as:
	//	- CTRL_REG0_XM = 0x00: FIFO disabled. HPF bypassed. Normal mode.
	//	- CTRL_REG1_XM = 0x57: 100 Hz data rate. Continuous update.
	//		all axes enabled.
	//	- CTRL_REG2_XM = 0x00:  2g scale. 773 Hz anti-alias filter BW.
	//	- CTRL_REG3_XM = 0x04: Accel data ready signal on INT1_XM pin.
	void initAccel();
	
	
	// gReadByte() -- Reads a byte from a specified gyroscope register.
	// Input:
	// 	- subAddress = Register to be read from.
	// Output:
	// 	- An 8-bit value read from the requested address.
	uint8_t gReadByte(uint8_t subAddress);
	
	// gReadBytes() -- Reads a number of bytes -- beginning at an address
	// and incrementing from there -- from the gyroscope.
	// Input:
	// 	- subAddress = Register to be read from.
	// 	- * dest = A pointer to an array of uint8_t's. Values read will be
	//		stored in here on return.
	//	- count = The number of bytes to be read.
	// Output: No value is returned, but the `dest` array will store
	// 	the data read upon exit.
	void gReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count);
	
	// gWriteByte() -- Write a byte to a register in the gyroscope.
	// Input:
	//	- subAddress = Register to be written to.
	//	- data = data to be written to the register.
	void gWriteByte(uint8_t subAddress, uint8_t data);
	
	// xmReadByte() -- Read a byte from a register in the accel/mag sensor
	// Input:
	//	- subAddress = Register to be read from.
	// Output:
	//	- An 8-bit value read from the requested register.
	uint8_t xmReadByte(uint8_t subAddress);
	
	// xmReadBytes() -- Reads a number of bytes -- beginning at an address
	// and incrementing from there -- from the accelerometer/magnetometer.
	// Input:
	// 	- subAddress = Register to be read from.
	// 	- * dest = A pointer to an array of uint8_t's. Values read will be
	//		stored in here on return.
	//	- count = The number of bytes to be read.
	// Output: No value is returned, but the `dest` array will store
	// 	the data read upon exit.
	void xmReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count);
	
	// xmWriteByte() -- Write a byte to a register in the accel/mag sensor.
	// Input:
	//	- subAddress = Register to be written to.
	//	- data = data to be written to the register.
	void xmWriteByte(uint8_t subAddress, uint8_t data);
	
	// calcgRes() -- Calculate the resolution of the gyroscope.
	// This function will set the value of the gRes variable. gScale must
	// be set prior to calling this function.
	void calcgRes();
	
	
	// calcaRes() -- Calculate the resolution of the accelerometer.
	// This function will set the value of the aRes variable. aScale must
	// be set prior to calling this function.
	void calcaRes();
	
	
	///////////////////
	// I2C Functions //
	///////////////////
	// initI2C() -- Initialize the I2C hardware.
	// This function will setup all I2C pins and related hardware.
	void initI2C();
	
	// I2CwriteByte() -- Write a byte out of I2C to a register in the device
	// Input:
	//	- address = The 7-bit I2C address of the slave device.
	//	- subAddress = The register to be written to.
	//	- data = Byte to be written to the register.
	void I2CwriteByte(uint8_t address, uint8_t subAddress, uint8_t data);
	
	// I2CreadByte() -- Read a single byte from a register over I2C.
	// Input:
	//	- address = The 7-bit I2C address of the slave device.
	//	- subAddress = The register to be read from.
	// Output:
	//	- The byte read from the requested address.
	uint8_t I2CreadByte(uint8_t address, uint8_t subAddress);
	
	// I2CreadBytes() -- Read a series of bytes, starting at a register via SPI
	// Input:
	//	- address = The 7-bit I2C address of the slave device.
	//	- subAddress = The register to begin reading.
	// 	- * dest = Pointer to an array where we'll store the readings.
	//	- count = Number of registers to be read.
	// Output: No value is returned by the function, but the registers read are
	// 		all stored in the *dest array given.
	void I2CreadBytes(uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t count); 
 
 
 
};


#endif