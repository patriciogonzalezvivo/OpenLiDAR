#include "BerryIMU.h"

#include <sys/stat.h>
#include <stdint.h>
#include <fcntl.h>

#include "tools/timeOps.h"

#include "berry/i2c-dev.h"
#include "berry/LSM9DS0.h"
#include "berry/LSM9DS1.h"

#define DT 0.2          // [s/loop] loop period.  0.2  = 200ms
#define AA 0.97         // complementary filter constant

#define A_GAIN 0.0573   // [deg/LSB]
#define G_GAIN 0.070    // [deg/s/LSB]
#define RAD_TO_DEG 57.29578
#define M_PI 3.14159265358979323846

BerryIMU::BerryIMU() : m_magMax(-32767), m_magMin(32767), m_prevTime(0.0), m_file(-1), m_LSM9DS0(false), m_LSM9DS1(false), m_calibrating(false) {
}

BerryIMU::~BerryIMU() {
}

/*
    These are the base functions for the BerryIMUv1 and BerryIMUv2


    Both the BerryIMUv1 and BerryIMUv2 are supported

    Feel free to do whatever you like with this code.
    Distributed as-is; no warranty is given.

    http://ozzmaker.com/
*/

void BerryIMU::readBlock(uint8_t _command, uint8_t _size, uint8_t *_data) {
    int result = i2c_smbus_read_i2c_block_data(m_file, _command, _size, _data);
    if (result != _size){
        std::cout << "Failed to read block from I2C." << std::endl;
        exit(1);
    }
}

void BerryIMU::selectDevice(int _file, int _addr) {
    if (ioctl(_file, I2C_SLAVE, _addr) < 0) {
        std::cout << "Failed to select I2C device." << std::endl;
    }
}

bool BerryIMU::connect(const char* _portName, bool _verbose) {
    
    // Detect IMU
    //
    __u16 block[I2C_SMBUS_BLOCK_MAX];

    int res, bus,  size;
    char filename[20];
    sprintf(filename, _portName, 1);
    m_file = open(filename, O_RDWR);
    if (m_file < 0) {
        std::cout << "Unable to open I2C bus!" << std::endl;
        return false;
    }

    //Detect if BerryIMUv1 (Which uses a LSM9DS0) is connected
    selectDevice(m_file,LSM9DS0_ACC_ADDRESS);
    int LSM9DS0_WHO_XM_response = i2c_smbus_read_byte_data(m_file, LSM9DS0_WHO_AM_I_XM);

    selectDevice(m_file,LSM9DS0_GYR_ADDRESS); 
    int LSM9DS0_WHO_G_response = i2c_smbus_read_byte_data(m_file, LSM9DS0_WHO_AM_I_G);

    if (LSM9DS0_WHO_G_response == 0xd4 && LSM9DS0_WHO_XM_response == 0x49)
        m_LSM9DS0 = true;

    //Detect if BerryIMUv2 (Which uses a LSM9DS1) is connected
    selectDevice(m_file,LSM9DS1_MAG_ADDRESS);
    int LSM9DS1_WHO_M_response = i2c_smbus_read_byte_data(m_file, LSM9DS1_WHO_AM_I_M);

    selectDevice(m_file,LSM9DS1_GYR_ADDRESS); 
    int LSM9DS1_WHO_XG_response = i2c_smbus_read_byte_data(m_file, LSM9DS1_WHO_AM_I_XG);

    if (LSM9DS1_WHO_XG_response == 0x68 && LSM9DS1_WHO_M_response == 0x3d)
        m_LSM9DS1 = true;

    if (!m_LSM9DS0 && !m_LSM9DS1){
        m_connected = false;
        std::cout << "NO IMU DETECTED" << std::endl;
        return false;
    }
    else 
        m_connected = true;

    if (_verbose)
        printFirmware();

    // enableIMU
    if (m_LSM9DS0){//For BerryIMUv1
        // Enable accelerometer.
        writeAccReg(LSM9DS0_CTRL_REG1_XM,   0b01100111);    //  z,y,x axis enabled, continuous update,  100Hz data rate
        writeAccReg(LSM9DS0_CTRL_REG2_XM,   0b00100000);    // +/- 16G full scale

        //Enable the magnetometer
        writeMagReg(LSM9DS0_CTRL_REG5_XM,   0b11110000);    // Temp enable, M data rate = 50Hz
        writeMagReg(LSM9DS0_CTRL_REG6_XM,   0b01100000);    // +/-12gauss
        writeMagReg(LSM9DS0_CTRL_REG7_XM,   0b00000000);    // Continuous-conversion mode

        // Enable Gyro
        writeGyrReg(LSM9DS0_CTRL_REG1_G,    0b00001111);    // Normal power mode, all axes enabled
        writeGyrReg(LSM9DS0_CTRL_REG4_G,    0b00110000);    // Continuos update, 2000 dps full scale
    }

    if (m_LSM9DS1){//For BerryIMUv2

        // Enable the gyroscope
        writeGyrReg(LSM9DS1_CTRL_REG4,      0b00111000);    // z, y, x axis enabled for gyro
        writeGyrReg(LSM9DS1_CTRL_REG1_G,    0b10111000);    // Gyro ODR = 476Hz, 2000 dps
        writeGyrReg(LSM9DS1_ORIENT_CFG_G,   0b10111000);    // Swap orientation 

        // Enable the accelerometer
        writeAccReg(LSM9DS1_CTRL_REG5_XL,   0b00111000);    // z, y, x axis enabled for accelerometer
        writeAccReg(LSM9DS1_CTRL_REG6_XL,   0b00101000);    // +/- 16g

        //Enable the magnetometer
        writeMagReg(LSM9DS1_CTRL_REG1_M,    0b10011100);    // Temp compensation enabled,Low power mode mode,80Hz ODR
        writeMagReg(LSM9DS1_CTRL_REG2_M,    0b01000000);    // +/-12gauss
        writeMagReg(LSM9DS1_CTRL_REG3_M,    0b00000000);    // continuos update
        writeMagReg(LSM9DS1_CTRL_REG4_M,    0b00000000);    // lower power mode for Z axis
    }

    m_magMax = glm::ivec3(-32767);
    m_magMin = glm::ivec3(32767);

    return true;
}

void BerryIMU::disconnect() {
}

bool BerryIMU::printFirmware() {
    if (m_LSM9DS0)
        std::cout << "BerryIMUv1/LSM9DS0  DETECTED" << std::endl;

    if (m_LSM9DS1)
        std::cout << "BerryIMUv2/LSM9DS1  DETECTED" << std::endl;

    return true;
}

void BerryIMU::writeAccReg(uint8_t _reg, uint8_t _value) {
    if (m_LSM9DS0)
        selectDevice(m_file,LSM9DS0_ACC_ADDRESS);
    else if (m_LSM9DS1)
        selectDevice(m_file,LSM9DS1_ACC_ADDRESS);

    int result = i2c_smbus_write_byte_data(m_file, _reg, _value);
    if (result == -1) {
        printf ("Failed to write byte to I2C Acc.");
        exit(1);
    }
}

void BerryIMU::writeMagReg(uint8_t _reg, uint8_t _value) {
    if (m_LSM9DS0)
        selectDevice(m_file,LSM9DS0_MAG_ADDRESS);
    else if (m_LSM9DS1)
        selectDevice(m_file,LSM9DS1_MAG_ADDRESS);
  
    int result = i2c_smbus_write_byte_data(m_file, _reg, _value);
    if (result == -1) {
        printf("Failed to write byte to I2C Mag.");
        exit(1);
    }
}

void BerryIMU::writeGyrReg(uint8_t _reg, uint8_t _value) {
    if (m_LSM9DS0)
        selectDevice(m_file,LSM9DS0_GYR_ADDRESS);
    else if (m_LSM9DS1)
        selectDevice(m_file,LSM9DS1_GYR_ADDRESS);
  
    int result = i2c_smbus_write_byte_data(m_file, _reg, _value);
    if (result == -1) {
        printf("Failed to write byte to I2C Gyr.");
        exit(1);
    }
}

void BerryIMU::readACC(int _a[]) {
    uint8_t block[6];
    if (m_LSM9DS0){
        selectDevice(m_file,LSM9DS0_ACC_ADDRESS);
        readBlock(0x80 |  LSM9DS0_OUT_X_L_A, sizeof(block), block);
    }
    else if (m_LSM9DS1){
        selectDevice(m_file,LSM9DS1_ACC_ADDRESS);
        readBlock(0x80 |  LSM9DS1_OUT_X_L_XL, sizeof(block), block);       
    }

    // Combine readings for each axis.
    _a[0] = (int16_t)(block[0] | block[1] << 8);
    _a[1] = (int16_t)(block[2] | block[3] << 8);
    _a[2] = (int16_t)(block[4] | block[5] << 8);
}


void BerryIMU::readMAG(int _m[]) {
    uint8_t block[6];
    if (m_LSM9DS0){
        selectDevice(m_file,LSM9DS0_MAG_ADDRESS);
        readBlock(0x80 |  LSM9DS0_OUT_X_L_M, sizeof(block), block);
    }
    else if (m_LSM9DS1){
        selectDevice(m_file,LSM9DS1_MAG_ADDRESS);
        readBlock(0x80 |  LSM9DS1_OUT_X_L_M, sizeof(block), block);    
    }

    // Combine readings for each axis.
    _m[0] = (int16_t)(block[0] | block[1] << 8);
    _m[1] = (int16_t)(block[2] | block[3] << 8);
    _m[2] = (int16_t)(block[4] | block[5] << 8);
}


void BerryIMU::readGYR(int _g[]) {
    uint8_t block[6];
    if (m_LSM9DS0) {
        selectDevice(m_file,LSM9DS0_GYR_ADDRESS);
        readBlock(0x80 |  LSM9DS0_OUT_X_L_M, sizeof(block), block);
    }
    else if (m_LSM9DS1) {
        selectDevice(m_file,LSM9DS1_GYR_ADDRESS);
        readBlock(0x80 |  LSM9DS1_OUT_X_L_M, sizeof(block), block);    
    }

    // Combine readings for each axis.
    _g[0] = (int16_t)(block[0] | block[1] << 8);
    _g[1] = (int16_t)(block[2] | block[3] << 8);
    _g[2] = (int16_t)(block[4] | block[5] << 8);
}

void BerryIMU::update(){
    updateAccGyr();
    updateMag();
}

void BerryIMU::updateAccGyr(){
    double currentTime = getElapsedSeconds();
    double deltaTime = currentTime - m_prevTime;
    m_prevTime = currentTime;

    int  acc_raw[3];
    int  mag_raw[3];
    int  gyr_raw[3];

    float rate_gyr_y = 0.0; // [deg/s]
    float rate_gyr_x = 0.0; // [deg/s]
    float rate_gyr_z = 0.0; // [deg/s]

    //read ACC and GYR data
    readACC(acc_raw);
    readGYR(gyr_raw);

    //Convert Gyro raw to degrees per second
    rate_gyr_x = (float) gyr_raw[0] * G_GAIN;
    rate_gyr_y = (float) gyr_raw[1] * G_GAIN;
    rate_gyr_z = (float) gyr_raw[2] * G_GAIN;

    //Calculate the angles from the gyro
    m_gyr.x += rate_gyr_x * deltaTime;
    m_gyr.y += rate_gyr_y * deltaTime;
    m_gyr.z += rate_gyr_z * deltaTime;

    //Convert Accelerometer values to degrees
    m_acc.x = (float) (atan2(acc_raw[1], acc_raw[2]) + M_PI) * RAD_TO_DEG;
    m_acc.y = (float) (atan2(acc_raw[2], acc_raw[0]) + M_PI) * RAD_TO_DEG;

    //Change the rotation value of the accelerometer to -/+ 180 and move the Y axis '0' point to up.
    //Two different pieces of code are used depending on how your IMU is mounted.
    //If IMU is upside down
    /*
    if (m_acc.x >180)
        m_acc.x -= (float)360.0;
    m_acc.y-=90;
    if (m_acc.y >180)
    A   ccYangle -= (float)360.0;
    */

    //If IMU is up the correct way, use these lines
    m_acc.x -= (float)180.0;
    if (m_acc.y > 90)
        m_acc.y -= (float)270;
    else
        m_acc.y += (float)90;

    //Normalize accelerometer raw values.
    float accXnorm, accYnorm, pitch, roll;
    accXnorm = acc_raw[0]/sqrt(acc_raw[0] * acc_raw[0] + acc_raw[1] * acc_raw[1] + acc_raw[2] * acc_raw[2]);
    accYnorm = acc_raw[1]/sqrt(acc_raw[0] * acc_raw[0] + acc_raw[1] * acc_raw[1] + acc_raw[2] * acc_raw[2]);

    //Calculate pitch and roll
    m_pitch = asin(accXnorm) * RAD_TO_DEG;
    m_roll = -asin(accYnorm/cos(pitch)) * RAD_TO_DEG;

    m_pitch -= (float)180.0;
    if (m_roll > 90)
        m_roll -= (float)270;
    else
        m_roll += (float)90;
}

void BerryIMU::updateMag() {

    // Magnetometer
    // -----------------------------------------------------------------------
    int magRaw[3];
    readMAG(magRaw);

    if (m_calibrating) 
    {
        if (magRaw[0] > m_magMax.x) m_magMax.x = magRaw[0];
        if (magRaw[1] > m_magMax.y) m_magMax.y = magRaw[1];
        if (magRaw[2] > m_magMax.z) m_magMax.z = magRaw[2];

        if (magRaw[0] < m_magMin.x) m_magMin.x = magRaw[0];
        if (magRaw[1] < m_magMin.y) m_magMin.y = magRaw[1];
        if (magRaw[2] < m_magMin.z) m_magMin.z = magRaw[2];
    }

    //Apply hard iron calibration
    magRaw[0] -= (m_magMin.x + m_magMax.x) / 2;
    magRaw[1] -= (m_magMin.y + m_magMax.y) / 2;
    magRaw[2] -= (m_magMin.z + m_magMax.z) / 2;

    //Apply soft iron calibration
    glm::vec3 scaledMag;
    scaledMag.x  = (float)(magRaw[0] - m_magMin.x) / (m_magMax.x - m_magMin.x) * 2.f - 1.f;
    scaledMag.y  = (float)(magRaw[1] - m_magMin.y) / (m_magMax.y - m_magMin.y) * 2.f - 1.f;
    scaledMag.z  = (float)(magRaw[2] - m_magMin.z) / (m_magMax.z - m_magMin.z) * 2.f - 1.f;

    //Compute m_heading
    m_heading = 180 * atan2(scaledMag.y, scaledMag.x) / M_PI;

    // //Convert heading to 0 - 360
    if(m_heading < 0.0)
        m_heading += 360.0;

    // //Calculate the new tilt compensated values
    // float magXcomp, magYcomp;
    // magXcomp = magRaw[0]*cos(m_pitch)+magRaw[2]*sin(m_pitch);
    // if (m_LSM9DS0)
    //     magYcomp = magRaw[0]*sin(m_roll)*sin(m_pitch)+magRaw[1]*cos(m_roll)-magRaw[2]*sin(m_roll)*cos(m_pitch); // LSM9DS0
    // else
    //     magYcomp = magRaw[0]*sin(m_roll)*sin(m_pitch)+magRaw[1]*cos(m_roll)+magRaw[2]*sin(m_roll)*cos(m_pitch); // LSM9DS1

    // //Calculate heading
    // m_heading = 180 * atan2(magYcomp, magXcomp) / M_PI;


    // //Local declination in mrads into radians
    // float declination = 217.9 / 1000.0;

    // //Add the declination correction to our current heading
    // m_heading += declination * 180/M_PI;

    //Correct the m_heading if declination forces it over 360
    // if ( m_heading > 360)
    //     m_heading -= 360;

}

bool BerryIMU::calibrate(bool _start) {
    m_calibrating = _start; 
    std::cout << "Min: " << m_magMin.x << " " << m_magMin.y << " " << m_magMin.z << std::endl;
    std::cout << "Max: " << m_magMax.x << " " << m_magMax.y << " " << m_magMax.z << std::endl;
    return true;
}
