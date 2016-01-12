/******************************************************************************
SFE_LSM330D.cpp
SFE_LSM330D Library Source File
by Dan Tudose
derived from Jim Lindblom LSM9DS0 library
Original Creation Date: January 12, 2016 

This file implements all functions of the LSM330D class. Functions here range
from higher level stuff, like reading/writing LSM330D registers to low-level,
hardware reads and writes. Both SPI and I2C handler functions can be found
towards the bottom of this file.

Development environment specifics:
	IDE: Arduino 1.0.5
	Hardware Platform: Arduino Pro 3.3V/8MHz
	
This code is beerware; if you see me or Jim at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

#include "LSM330D.h"
#include <Wire.h> // Wire library is used for I2C


#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

LSM330D::LSM330D(interface_mode interface, uint8_t gAddr, uint8_t xmAddr)
{
	// interfaceMode will keep track of whether we're using SPI or I2C:
	interfaceMode = interface;
	
	// xmAddress and gAddress will store the 7-bit I2C address, if using I2C.
	// If we're using SPI, these variables store the chip-select pins.
	xmAddress = xmAddr;
	gAddress = gAddr;
}

uint16_t LSM330D::begin(gyro_scale gScl, accel_scale aScl,  
						gyro_odr gODR, accel_odr aODR)
{
	// Store the given scales in class variables. These scale variables
	// are used throughout to calculate the actual g's, DPS,and Gs's.
	gScale = gScl;
	aScale = aScl;
	
	
	// Once we have the scale values, we can calculate the resolution
	// of each sensor. That's what these functions are for. One for each sensor
	calcgRes(); // Calculate DPS / ADC tick, stored in gRes variable
	calcaRes(); // Calculate g / ADC tick, stored in aRes variable
	
	// Now, initialize our hardware interface.
	if (interfaceMode == MODE_I2C)	// If we're using I2C
		initI2C();					// Initialize I2C
	
	
	// To verify communication, we can read from the WHO_AM_I register of
	// each device. Store those in a variable so we can return them.
	uint8_t gTest = gReadByte(WHO_AM_I_G);		// Read the gyro WHO_AM_I
	//uint8_t xmTest = xmReadByte(WHO_AM_I_XM);	// Read the accel/mag WHO_AM_I
	
	// Gyro initialization stuff:
	initGyro();	// This will "turn on" the gyro. Setting up interrupts, etc.
	setGyroODR(gODR); // Set the gyro output data rate and bandwidth.
	setGyroScale(gScale); // Set the gyro range
	
	// Accelerometer initialization stuff:
	initAccel(); // "Turn on" all axes of the accel. Set up interrupts, etc.
	setAccelODR(aODR); // Set the accel data rate.
	setAccelScale(aScale); // Set the accel range.
	
	
	// Once everything is initialized, return the WHO_AM_I registers we read:
	return gTest;
}

void LSM330D::initGyro()
{
	
	/* CTRL_REG1_G sets output data rate, bandwidth, power-down and enables
	Bits[7:0]: DR1 DR0 BW1 BW0 PD Zen Xen Yen
	DR[1:0] - Output data rate selection
		00=95Hz, 01=190Hz, 10=380Hz, 11=760Hz
	BW[1:0] - Bandwidth selection (sets cutoff frequency)
		 Value depends on ODR. See datasheet table 21.
	PD - Power down enable (0=power down mode, 1=normal or sleep mode)
	Zen, Xen, Yen - Axis enable (o=disabled, 1=enabled)	*/
	gWriteByte(CTRL_REG1_G, 0x0F); // Normal mode, enable all axes
	
	/* CTRL_REG2_G sets up the HPF
	Bits[7:0]: 0 0 HPM1 HPM0 HPCF3 HPCF2 HPCF1 HPCF0
	HPM[1:0] - High pass filter mode selection
		00=normal (reset reading HP_RESET_FILTER, 01=ref signal for filtering,
		10=normal, 11=autoreset on interrupt
	HPCF[3:0] - High pass filter cutoff frequency
		Value depends on data rate. See datasheet table 26.
	*/
	gWriteByte(CTRL_REG2_G, 0x00); // Normal mode, high cutoff frequency
	
	/* CTRL_REG3_G sets up interrupt and DRDY_G pins
	Bits[7:0]: I1_IINT1 I1_BOOT H_LACTIVE PP_OD I2_DRDY I2_WTM I2_ORUN I2_EMPTY
	I1_INT1 - Interrupt enable on INT_G pin (0=disable, 1=enable)
	I1_BOOT - Boot status available on INT_G (0=disable, 1=enable)
	H_LACTIVE - Interrupt active configuration on INT_G (0:high, 1:low)
	PP_OD - Push-pull/open-drain (0=push-pull, 1=open-drain)
	I2_DRDY - Data ready on DRDY_G (0=disable, 1=enable)
	I2_WTM - FIFO watermark interrupt on DRDY_G (0=disable 1=enable)
	I2_ORUN - FIFO overrun interrupt on DRDY_G (0=disable 1=enable)
	I2_EMPTY - FIFO empty interrupt on DRDY_G (0=disable 1=enable) */
	// Int1 enabled (pp, active low), data read on DRDY_G:
	gWriteByte(CTRL_REG3_G, 0x00); //0x88
	
	/* CTRL_REG4_G sets the scale, update mode
	Bits[7:0] - BDU BLE FS1 FS0 - ST1 ST0 SIM
	BDU - Block data update (0=continuous, 1=output not updated until read
	BLE - Big/little endian (0=data LSB @ lower address, 1=LSB @ higher add)
	FS[1:0] - Full-scale selection
		00=245dps, 01=500dps, 10=2000dps, 11=2000dps
	ST[1:0] - Self-test enable
		00=disabled, 01=st 0 (x+, y-, z-), 10=undefined, 11=st 1 (x-, y+, z+)
	SIM - SPI serial interface mode select
		0=4 wire, 1=3 wire */
	gWriteByte(CTRL_REG4_G, 0x00); // Set scale to 245 dps
	
	/* CTRL_REG5_G sets up the FIFO, HPF, and INT1
	Bits[7:0] - BOOT FIFO_EN - HPen INT1_Sel1 INT1_Sel0 Out_Sel1 Out_Sel0
	BOOT - Reboot memory content (0=normal, 1=reboot)
	FIFO_EN - FIFO enable (0=disable, 1=enable)
	HPen - HPF enable (0=disable, 1=enable)
	INT1_Sel[1:0] - Int 1 selection configuration
	Out_Sel[1:0] - Out selection configuration */
	gWriteByte(CTRL_REG5_G, 0x00);
	
	// Temporary !!! For testing !!! Remove !!! Or make useful !!!
	//configGyroInt(0x2A, 0, 0, 0, 0); // Trigger interrupt when above 0 DPS...
}

void LSM330D::initAccel()
{
	
	xmWriteByte(CTRL_REG1_A, 0x97); //0b10010111
    xmWriteByte(CTRL_REG2_A, 0x00);
    xmWriteByte(CTRL_REG3_A, 0x08); //0b00001000
    xmWriteByte(CTRL_REG4_A, 0x08);  // +/-2G (1mg/digt)
    xmWriteByte(CTRL_REG5_A, 0x40);
	
	/* CTRL_REG0_XM (0x1F) (Default value: 0x00)
	Bits (7-0): BOOT FIFO_EN WTM_EN 0 0 HP_CLICK HPIS1 HPIS2
	BOOT - Reboot memory content (0: normal, 1: reboot)
	FIFO_EN - Fifo enable (0: disable, 1: enable)
	WTM_EN - FIFO watermark enable (0: disable, 1: enable)
	HP_CLICK - HPF enabled for click (0: filter bypassed, 1: enabled)
	HPIS1 - HPF enabled for interrupt generator 1 (0: bypassed, 1: enabled)
	HPIS2 - HPF enabled for interrupt generator 2 (0: bypassed, 1 enabled)   */
	//xmWriteByte(CTRL_REG0_XM, 0x00);
	
	/* CTRL_REG1_XM (0x20) (Default value: 0x07)
	Bits (7-0): AODR3 AODR2 AODR1 AODR0 BDU AZEN AYEN AXEN
	AODR[3:0] - select the acceleration data rate:
		0000=power down, 0001=3.125Hz, 0010=6.25Hz, 0011=12.5Hz, 
		0100=25Hz, 0101=50Hz, 0110=100Hz, 0111=200Hz, 1000=400Hz,
		1001=800Hz, 1010=1600Hz, (remaining combinations undefined).
	BDU - block data update for accel AND mag
		0: Continuous update
		1: Output registers aren't updated until MSB and LSB have been read.
	AZEN, AYEN, and AXEN - Acceleration x/y/z-axis enabled.
		0: Axis disabled, 1: Axis enabled									 */	
	//xmWriteByte(CTRL_REG1_XM, 0x57); // 100Hz data rate, x/y/z all enabled
	
	//Serial.println(xmReadByte(CTRL_REG1_XM));
	/* CTRL_REG2_XM (0x21) (Default value: 0x00)
	Bits (7-0): ABW1 ABW0 AFS2 AFS1 AFS0 AST1 AST0 SIM
	ABW[1:0] - Accelerometer anti-alias filter bandwidth
		00=773Hz, 01=194Hz, 10=362Hz, 11=50Hz
	AFS[2:0] - Accel full-scale selection
		000=+/-2g, 001=+/-4g, 010=+/-6g, 011=+/-8g, 100=+/-16g
	AST[1:0] - Accel self-test enable
		00=normal (no self-test), 01=positive st, 10=negative st, 11=not allowed
	SIM - SPI mode selection
		0=4-wire, 1=3-wire													 */
	//xmWriteByte(CTRL_REG2_XM, 0x00); // Set scale to 2g
	
	/* CTRL_REG3_XM is used to set interrupt generators on INT1_XM
	Bits (7-0): P1_BOOT P1_TAP P1_INT1 P1_INT2 P1_INTM P1_DRDYA P1_DRDYM P1_EMPTY
	*/
	// Accelerometer data ready on INT1_XM (0x04)
	//xmWriteByte(CTRL_REG3_XM, 0x04); 
}



// This is a function that uses the FIFO to accumulate sample of accelerometer and gyro data, average
// them, scales them to  gs and deg/s, respectively, and then passes the biases to the main sketch
// for subtraction from all subsequent data. There are no gyro and accelerometer bias registers to store
// the data as there are in the ADXL345, a precursor to the LSM330D, or the MPU-9150, so we have to
// subtract the biases ourselves. This results in a more accurate measurement in general and can
// remove errors due to imprecise or varying initial placement. Calibration of sensor data in this manner
// is good practice.

void LSM330D::calLSM330D(float * gbias, float * abias)
{  
  uint8_t data[6] = {0, 0, 0, 0, 0, 0};
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  uint16_t samples, ii;
  
  // First get gyro bias
  byte c = gReadByte(CTRL_REG5_G);
  gWriteByte(CTRL_REG5_G, c | 0x40);         // Enable gyro FIFO  
  delay(20);                                 // Wait for change to take effect
  gWriteByte(FIFO_CTRL_REG_G, 0x20 | 0x1F);  // Enable gyro FIFO stream mode and set watermark at 32 samples
  delay(1000);  // delay 1000 milliseconds to collect FIFO samples
  
  samples = (gReadByte(FIFO_SRC_REG_G) & 0x1F); // Read number of stored samples

  for(ii = 0; ii < samples ; ii++) {            // Read the gyro data stored in the FIFO
    int16_t gyro_temp[3] = {0, 0, 0};
    gReadBytes(OUT_X_L_G,  &data[0], 6);
    gyro_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]); // Form signed 16-bit integer for each sample in FIFO
    gyro_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]);
    gyro_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]);

    gyro_bias[0] += (int32_t) gyro_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    gyro_bias[1] += (int32_t) gyro_temp[1]; 
    gyro_bias[2] += (int32_t) gyro_temp[2]; 
  }  

  gyro_bias[0] /= samples; // average the data
  gyro_bias[1] /= samples; 
  gyro_bias[2] /= samples; 
  
  gbias[0] = (float)gyro_bias[0]*gRes;  // Properly scale the data to get deg/s
  gbias[1] = (float)gyro_bias[1]*gRes;
  gbias[2] = (float)gyro_bias[2]*gRes;
  
  c = gReadByte(CTRL_REG5_G);
  gWriteByte(CTRL_REG5_G, c & ~0x40);  // Disable gyro FIFO  
  delay(20);
  gWriteByte(FIFO_CTRL_REG_G, 0x00);   // Enable gyro bypass mode
  

  //  Now get the accelerometer biases
  c = xmReadByte(CTRL_REG5_A);
  xmWriteByte(CTRL_REG5_A, c | 0x40);      // Enable accelerometer FIFO  
  delay(20);                                // Wait for change to take effect
  xmWriteByte(FIFO_CTRL_REG, 0x40 | 0x1F);  // Enable accelerometer FIFO stream mode and set watermark at 32 samples
  delay(1000);  // delay 1000 milliseconds to collect FIFO samples

  samples = (xmReadByte(FIFO_SRC_REG) & 0x1F); // Read number of stored accelerometer samples

   for(ii = 0; ii < samples ; ii++) {          // Read the accelerometer data stored in the FIFO
    int16_t accel_temp[3] = {0, 0, 0};
    xmReadBytes(OUT_X_L_A, &data[0], 6);
    accel_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]);// Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]);
    accel_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]);  

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1]; 
    accel_bias[2] += (int32_t) accel_temp[2]; 
  }  

  accel_bias[0] /= samples; // average the data
  accel_bias[1] /= samples; 
  accel_bias[2] /= samples; 

  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) (1.0/aRes);}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) (1.0/aRes);}
 
  
  abias[0] = (float)accel_bias[0]*aRes; // Properly scale data to get gs
  abias[1] = (float)accel_bias[1]*aRes;
  abias[2] = (float)accel_bias[2]*aRes;

  c = xmReadByte(CTRL_REG5_A);
  xmWriteByte(CTRL_REG5_A, c & ~0x40);    // Disable accelerometer FIFO  
  delay(20);
  xmWriteByte(FIFO_CTRL_REG, 0x00);       // Enable accelerometer bypass mode
}

void LSM330D::readAccel()
{
	uint8_t temp[6]; // We'll read six bytes from the accelerometer into temp	
	xmReadBytes(OUT_X_L_A, temp, 6); // Read 6 bytes, beginning at OUT_X_L_A
	ax = (temp[1] << 8) | temp[0]; // Store x-axis values into ax
	ay = (temp[3] << 8) | temp[2]; // Store y-axis values into ay
	az = (temp[5] << 8) | temp[4]; // Store z-axis values into az
}


void LSM330D::readTemp()
{
	int8_t temp; // We'll read two bytes from the temperature sensor into temp	
	temp = xmReadByte(OUT_TEMP_G); // Read 2 bytes, beginning at OUT_TEMP_L_M
        temperature = temp; // Temperature is a 12-bit signed integer
}

void LSM330D::readGyro()
{
	uint8_t temp[6]; // We'll read six bytes from the gyro into temp
	gReadBytes(OUT_X_L_G, temp, 6); // Read 6 bytes, beginning at OUT_X_L_G
	gx = (temp[1] << 8) | temp[0]; // Store x-axis values into gx
	gy = (temp[3] << 8) | temp[2]; // Store y-axis values into gy
	gz = (temp[5] << 8) | temp[4]; // Store z-axis values into gz
}

float LSM330D::calcGyro(int16_t gyro)
{
	// Return the gyro raw reading times our pre-calculated DPS / (ADC tick):
	return gRes * gyro; 
}

float LSM330D::calcAccel(int16_t accel)
{
	// Return the accel raw reading times our pre-calculated g's / (ADC tick):
	return aRes * accel;
}


void LSM330D::setGyroScale(gyro_scale gScl)
{
	// We need to preserve the other bytes in CTRL_REG4_G. So, first read it:
	uint8_t temp = gReadByte(CTRL_REG4_G);
	// Then mask out the gyro scale bits:
	temp &= 0xCF;
	// Then shift in our new scale bits:
	temp |= gScl << 4;
	// And write the new register value back into CTRL_REG4_G:
	gWriteByte(CTRL_REG4_G, temp);
	
	// We've updated the sensor, but we also need to update our class variables
	// First update gScale:
	gScale = gScl;
	// Then calculate a new gRes, which relies on gScale being set correctly:
	calcgRes();
}

void LSM330D::setAccelScale(accel_scale aScl)
{
	// We need to preserve the other bytes in CTRL_REG4_A. So, first read it:
	uint8_t temp = xmReadByte(CTRL_REG4_A);
	// Then mask out the accel scale bits:
	temp &= 0xCF;
	// Then shift in our new scale bits:
	temp |= aScl << 4;
	// And write the new register value back into CTRL_REG2_XM:
	xmWriteByte(CTRL_REG4_A, temp);
	
	// We've updated the sensor, but we also need to update our class variables
	// First update aScale:
	aScale = aScl;
	// Then calculate a new aRes, which relies on aScale being set correctly:
	calcaRes();
}


void LSM330D::setGyroODR(gyro_odr gRate)
{
	// We need to preserve the other bytes in CTRL_REG1_G. So, first read it:
	uint8_t temp = gReadByte(CTRL_REG1_G);
	// Then mask out the gyro ODR bits:
	temp &= 0x0F;
	// Then shift in our new ODR bits:
	temp |= (gRate << 4);
	// And write the new register value back into CTRL_REG1_G:
	gWriteByte(CTRL_REG1_G, temp);
}
void LSM330D::setAccelODR(accel_odr aRate)
{
	// We need to preserve the other bytes in CTRL_REG1_XM. So, first read it:
	uint8_t temp = xmReadByte(CTRL_REG1_A);
	// Then mask out the accel ODR bits:
	temp &= 0x0F;
	// Then shift in our new ODR bits:
	temp |= (aRate << 4);
	// And write the new register value back into CTRL_REG1_XM:
	xmWriteByte(CTRL_REG1_A, temp);
}


void LSM330D::configGyroInt(uint8_t int1Cfg, uint16_t int1ThsX, uint16_t int1ThsY, uint16_t int1ThsZ, uint8_t duration)
{
	gWriteByte(INT1_CFG_G, int1Cfg);
	gWriteByte(INT1_TSH_XH_G, (int1ThsX & 0xFF00) >> 8);
	gWriteByte(INT1_TSH_XL_G, (int1ThsX & 0xFF));
	gWriteByte(INT1_TSH_YH_G, (int1ThsY & 0xFF00) >> 8);
	gWriteByte(INT1_TSH_YL_G, (int1ThsY & 0xFF));
	gWriteByte(INT1_TSH_ZH_G, (int1ThsZ & 0xFF00) >> 8);
	gWriteByte(INT1_TSH_ZL_G, (int1ThsZ & 0xFF));
	if (duration)
		gWriteByte(INT1_DURATION_G, 0x80 | duration);
	else
		gWriteByte(INT1_DURATION_G, 0x00);
}

void LSM330D::calcgRes()
{
	// Possible gyro scales (and their register bit settings) are:
	// 250 DPS (00), 500 DPS (01), 2000 DPS (10). Here's a bit of an algorithm
	// to calculate DPS/(ADC tick) based on that 2-bit value:
	switch (gScale)
	{
	case G_SCALE_250DPS:
		gRes = 250.0 / 32768.0;
		break;
	case G_SCALE_500DPS:
		gRes = 500.0 / 32768.0;
		break;
	case G_SCALE_2000DPS:
		gRes = 2000.0 / 32768.0;
		break;
	}
}

void LSM330D::calcaRes()
{
	// Possible accelerometer scales (and their register bit settings) are:
	// 2 g (00), 4g (01), 8g (10), 16g (11). 
	
	switch (aScale)
	{
	case A_SCALE_16G:
		aRes = 16.0 / 32768.0;
		break;
	case A_SCALE_8G:
		aRes = 8.0 / 32768.0;
		break;
	case A_SCALE_4G:
		aRes = 4.0 / 32768.0;
		break;
	case A_SCALE_2G:
		aRes = 2.0 / 32768.0;
		break;
	}
	
}


	
void LSM330D::gWriteByte(uint8_t subAddress, uint8_t data)
{
	// Whether we're using I2C or SPI, write a byte using the
	// gyro-specific I2C address or SPI CS pin.
	if (interfaceMode == MODE_I2C)
		I2CwriteByte(gAddress, subAddress, data);
	
}

void LSM330D::xmWriteByte(uint8_t subAddress, uint8_t data)
{
	// Whether we're using I2C or SPI, write a byte using the
	// accelerometer-specific I2C address or SPI CS pin.
	if (interfaceMode == MODE_I2C)
		return I2CwriteByte(xmAddress, subAddress, data);
	
}

uint8_t LSM330D::gReadByte(uint8_t subAddress)
{
	// Whether we're using I2C or SPI, read a byte using the
	// gyro-specific I2C address or SPI CS pin.
	if (interfaceMode == MODE_I2C)
		return I2CreadByte(gAddress, subAddress);
	
}

void LSM330D::gReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count)
{
	// Whether we're using I2C or SPI, read multiple bytes using the
	// gyro-specific I2C address or SPI CS pin.
	if (interfaceMode == MODE_I2C)
		I2CreadBytes(gAddress, subAddress, dest, count);
	
}

uint8_t LSM330D::xmReadByte(uint8_t subAddress)
{
	// Whether we're using I2C or SPI, read a byte using the
	// accelerometer-specific I2C address or SPI CS pin.
	if (interfaceMode == MODE_I2C)
		return I2CreadByte(xmAddress, subAddress);
	
}

void LSM330D::xmReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count)
{
	// Whether we're using I2C or SPI, read multiple bytes using the
	// accelerometer-specific I2C address or SPI CS pin.
	if (interfaceMode == MODE_I2C)
		I2CreadBytes(xmAddress, subAddress, dest, count);
	
}



void LSM330D::initI2C()
{
	Wire.begin();	// Initialize I2C library
}


        // Wire.h read and write protocols
        void LSM330D::I2CwriteByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	Wire.beginTransmission(address);  // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
	Wire.write(data);                 // Put data in Tx buffer
	Wire.endTransmission();           // Send the Tx buffer
}

        uint8_t LSM330D::I2CreadByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data; // `data` will store the register data	 
	Wire.beginTransmission(address);         // Initialize the Tx buffer
	Wire.write(subAddress);	                 // Put slave register address in Tx buffer
	Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
	Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address 
	data = Wire.read();                      // Fill Rx buffer with result
	return data;                             // Return data read from slave register
}

        void LSM330D::I2CreadBytes(uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t count)
{  
	Wire.beginTransmission(address);   // Initialize the Tx buffer
	// Next send the register to be read. OR with 0x80 to indicate multi-read.
	Wire.write(subAddress | 0x80);     // Put slave register address in Tx buffer
	Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
        Wire.requestFrom(address, count);  // Read bytes from slave register address 
	while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}
