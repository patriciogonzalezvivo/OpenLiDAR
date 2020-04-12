/*
    These are the base functions for the BerryIMUv1 and BerryIMUv2


    Both the BerryIMUv1 and BerryIMUv2 are supported

    Feel free to do whatever you like with this code.
    Distributed as-is; no warranty is given.

    http://ozzmaker.com/
*/

#include <stdint.h>
#include "i2c-dev.h"
#include "LSM9DS0.h"
#include "LSM9DS1.h"

int file;
int LSM9DS0 = 0;
int LSM9DS1 = 0;

void  readBlock(uint8_t command, uint8_t size, uint8_t *data)
{
	int result = i2c_smbus_read_i2c_block_data(file, command, size, data);
	if (result != size){
		printf("Failed to read block from I2C.");
		exit(1);
	}
}

void selectDevice(int file, int addr)
{
	if (ioctl(file, I2C_SLAVE, addr) < 0) {
		printf("Failed to select I2C device.");
	}
}


void readACC(int  a[])
{
	uint8_t block[6];
	if (LSM9DS0){
		selectDevice(file,LSM9DS0_ACC_ADDRESS);
		readBlock(0x80 |  LSM9DS0_OUT_X_L_A, sizeof(block), block);
	}
	else if (LSM9DS1){
		selectDevice(file,LSM9DS1_ACC_ADDRESS);
		readBlock(0x80 |  LSM9DS1_OUT_X_L_XL, sizeof(block), block);       
	}

	// Combine readings for each axis.
	a[0] = (int16_t)(block[0] | block[1] << 8);
	a[1] = (int16_t)(block[2] | block[3] << 8);
	a[2] = (int16_t)(block[4] | block[5] << 8);
}


void readMAG(int  m[])
{
	uint8_t block[6];
	if (LSM9DS0){
		selectDevice(file,LSM9DS0_MAG_ADDRESS);
		readBlock(0x80 |  LSM9DS0_OUT_X_L_M, sizeof(block), block);
	}
	else if (LSM9DS1){
		selectDevice(file,LSM9DS1_MAG_ADDRESS);
		readBlock(0x80 |  LSM9DS1_OUT_X_L_M, sizeof(block), block);    
	}

	// Combine readings for each axis.
	m[0] = (int16_t)(block[0] | block[1] << 8);
	m[1] = (int16_t)(block[2] | block[3] << 8);
	m[2] = (int16_t)(block[4] | block[5] << 8);
}


void readGYR(int g[])
{
	uint8_t block[6];
    if (LSM9DS0){
		selectDevice(file,LSM9DS0_GYR_ADDRESS);
		readBlock(0x80 |  LSM9DS0_OUT_X_L_M, sizeof(block), block);
	}
	else if (LSM9DS1){
		selectDevice(file,LSM9DS1_GYR_ADDRESS);
		readBlock(0x80 |  LSM9DS1_OUT_X_L_M, sizeof(block), block);    
	}

	// Combine readings for each axis.
	g[0] = (int16_t)(block[0] | block[1] << 8);
	g[1] = (int16_t)(block[2] | block[3] << 8);
	g[2] = (int16_t)(block[4] | block[5] << 8);
}


void writeAccReg(uint8_t reg, uint8_t value)
{
	if (LSM9DS0)
		selectDevice(file,LSM9DS0_ACC_ADDRESS);
	else if (LSM9DS1)
		selectDevice(file,LSM9DS1_ACC_ADDRESS);

	int result = i2c_smbus_write_byte_data(file, reg, value);
	if (result == -1){
		printf ("Failed to write byte to I2C Acc.");
		exit(1);
	}
}

void writeMagReg(uint8_t reg, uint8_t value)
{
	if (LSM9DS0)
		selectDevice(file,LSM9DS0_MAG_ADDRESS);
	else if (LSM9DS1)
		selectDevice(file,LSM9DS1_MAG_ADDRESS);
  
	int result = i2c_smbus_write_byte_data(file, reg, value);
	if (result == -1){
		printf("Failed to write byte to I2C Mag.");
		exit(1);
	}
}


void writeGyrReg(uint8_t reg, uint8_t value)
{
    if (LSM9DS0)
		selectDevice(file,LSM9DS0_GYR_ADDRESS);
	else if (LSM9DS1)
		selectDevice(file,LSM9DS1_GYR_ADDRESS);
  
	int result = i2c_smbus_write_byte_data(file, reg, value);
	if (result == -1){
		printf("Failed to write byte to I2C Gyr.");
		exit(1);
	}
}


void detectIMU()
{

	__u16 block[I2C_SMBUS_BLOCK_MAX];

	int res, bus,  size;


	char filename[20];
	sprintf(filename, "/dev/i2c-%d", 1);
	file = open(filename, O_RDWR);
	if (file<0) {
		printf("Unable to open I2C bus!");
			exit(1);
	}

	//Detect if BerryIMUv1 (Which uses a LSM9DS0) is connected
	selectDevice(file,LSM9DS0_ACC_ADDRESS);
	int LSM9DS0_WHO_XM_response = i2c_smbus_read_byte_data(file, LSM9DS0_WHO_AM_I_XM);

	selectDevice(file,LSM9DS0_GYR_ADDRESS);	
	int LSM9DS0_WHO_G_response = i2c_smbus_read_byte_data(file, LSM9DS0_WHO_AM_I_G);

	if (LSM9DS0_WHO_G_response == 0xd4 && LSM9DS0_WHO_XM_response == 0x49){
		printf ("\n\n\n#####   BerryIMUv1/LSM9DS0  DETECTED    #####\n\n");
		LSM9DS0 = 1;
	}




	//Detect if BerryIMUv2 (Which uses a LSM9DS1) is connected
	selectDevice(file,LSM9DS1_MAG_ADDRESS);
	int LSM9DS1_WHO_M_response = i2c_smbus_read_byte_data(file, LSM9DS1_WHO_AM_I_M);

	selectDevice(file,LSM9DS1_GYR_ADDRESS);	
	int LSM9DS1_WHO_XG_response = i2c_smbus_read_byte_data(file, LSM9DS1_WHO_AM_I_XG);

    if (LSM9DS1_WHO_XG_response == 0x68 && LSM9DS1_WHO_M_response == 0x3d){
		printf ("\n\n\n#####   BerryIMUv2/LSM9DS1  DETECTED    #####\n\n");
		LSM9DS1 = 1;
	}
  


	if (!LSM9DS0 && !LSM9DS1){
		printf ("NO IMU DETECTED\n");
		exit(1);
	}
}




void enableIMU()
{

	if (LSM9DS0){//For BerryIMUv1
		// Enable accelerometer.
		writeAccReg(LSM9DS0_CTRL_REG1_XM, 0b01100111); //  z,y,x axis enabled, continuous update,  100Hz data rate
		writeAccReg(LSM9DS0_CTRL_REG2_XM, 0b00100000); // +/- 16G full scale

		//Enable the magnetometer
		writeMagReg(LSM9DS0_CTRL_REG5_XM, 0b11110000); // Temp enable, M data rate = 50Hz
		writeMagReg(LSM9DS0_CTRL_REG6_XM, 0b01100000); // +/-12gauss
		writeMagReg(LSM9DS0_CTRL_REG7_XM, 0b00000000); // Continuous-conversion mode

		// Enable Gyro
		writeGyrReg(LSM9DS0_CTRL_REG1_G, 0b00001111); // Normal power mode, all axes enabled
		writeGyrReg(LSM9DS0_CTRL_REG4_G, 0b00110000); // Continuos update, 2000 dps full scale
	}

	if (LSM9DS1){//For BerryIMUv2
		// Enable the gyroscope
		writeGyrReg(LSM9DS1_CTRL_REG4,0b00111000);      // z, y, x axis enabled for gyro
		writeGyrReg(LSM9DS1_CTRL_REG1_G,0b10111000);    // Gyro ODR = 476Hz, 2000 dps
		writeGyrReg(LSM9DS1_ORIENT_CFG_G,0b10111000);   // Swap orientation 

		// Enable the accelerometer
		writeAccReg(LSM9DS1_CTRL_REG5_XL,0b00111000);   // z, y, x axis enabled for accelerometer
		writeAccReg(LSM9DS1_CTRL_REG6_XL,0b00101000);   // +/- 16g

		//Enable the magnetometer
		writeMagReg(LSM9DS1_CTRL_REG1_M, 0b10011100);   // Temp compensation enabled,Low power mode mode,80Hz ODR
		writeMagReg(LSM9DS1_CTRL_REG2_M, 0b01000000);   // +/-12gauss
		writeMagReg(LSM9DS1_CTRL_REG3_M, 0b00000000);   // continuos update
		writeMagReg(LSM9DS1_CTRL_REG4_M, 0b00000000);   // lower power mode for Z axis
	}

}



