#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <time.h>
#include <math.h>
#include <sys/time.h>
#include <stdint.h>
#include <signal.h>
#include <sys/shm.h>
#include <sys/stat.h>

// gcc -o week1 week_1_student.cpp -lwiringPi  -lm

int setup_imu();
void calibrate_imu();
void read_imu();
void update_filter();
void print_imu_data();

// global variables
int accel_address, gyro_address;
float x_gyro_calibration = 0;
float y_gyro_calibration = 0;
float z_gyro_calibration = 0;
float roll_calibration = 0;
float pitch_calibration = 0;
float accel_z_calibration = 0;
float imu_data[6]; // accel xyz,  gyro xyz,
long time_curr;
long time_prev;
struct timespec te;
float yaw = 0;
float pitch_angle = 0;
float roll_angle = 0;

int main(int argc, char *argv[])
{

    setup_imu();
    //sample 1000 times and average them to calibrate
    calibrate_imu();

    while (1)
    {
        read_imu();
        print_imu_data();
    }
}

void calibrate_imu()
{

    for (int i = 0; i < 1000; i++)
    {
        read_imu();

        x_gyro_calibration += imu_data[3];
        y_gyro_calibration += imu_data[4];
        z_gyro_calibration += imu_data[5];

        roll_calibration += atan2(imu_data[2], imu_data[0]) * 180.0 / M_PI;
        pitch_calibration += atan2(-imu_data[1], sqrt(imu_data[0]*imu_data[0] + imu_data[2]*imu_data[2])) * 180.0 / M_PI;
    }

    x_gyro_calibration /= 1000.0;
    y_gyro_calibration /= 1000.0;
    z_gyro_calibration /= 1000.0;
    roll_calibration /= 1000.0;
    pitch_calibration /= 1000.0;


    printf("calibration complete, %f %f %f %f %f %f\n\r", x_gyro_calibration, y_gyro_calibration, z_gyro_calibration, roll_calibration, pitch_calibration);
}

void read_imu()
{
    uint8_t address = 0; // todo: set address value for accel x value
    float ax = 0;
    float az = 0;
    float ay = 0;
    int vh = 0;
    int vl = 0;
    int vw = 0;


    //-----------------------------accel reads---------------------------------------

    address = 0x12; // x accel
    vw = wiringPiI2CReadReg16(accel_address, address);
    // convert from 2's complement
    if (vw > 0x8000)
    {
        vw = vw ^ 0xffff;
        vw = -vw - 1;
    }
    imu_data[0] = ((float)vw) / 32768.0 * 3.0; // convert to g's

    address = 0x14; // y accel
    vw = wiringPiI2CReadReg16(accel_address, address);
    // convert from 2's complement
    if (vw > 0x8000)
    {
        vw = vw ^ 0xffff;
        vw = -vw - 1;
    }
    imu_data[1] = ((float)vw) / 32768.0 * 3.0; // convert to g's

    address = 0x16; // z accel
    vw = wiringPiI2CReadReg16(accel_address, address);
    // convert from 2's complement
    if (vw > 0x8000)
    {
        vw = vw ^ 0xffff;
        vw = -vw - 1;
    }
    imu_data[2] = ((float)vw) / 32768.0 * 3.0; // convert to g's


    //-----------------------------gyro reads---------------------------------------
    
    address = 0x02; // x gyro
    vw = wiringPiI2CReadReg16(gyro_address, address);



    // convert from 2's complement
    if (vw > 0x8000)
    {
        vw = vw ^ 0xffff;
        vw = -vw - 1;
    }
    imu_data[3] = ((float)vw) / 32768.0 * 1000.0; // convert to degrees/sec

    address = 0x04; // y gyro
    vw = wiringPiI2CReadReg16(gyro_address, address);

    // convert from 2's complement
    if (vw > 0x8000)
    {
        vw = vw ^ 0xffff;
        vw = -vw - 1;
    }
    imu_data[4] = ((float)vw) / 32768.0 * 1000.0; // convert to degrees/sec

    address = 0x06; // z gyro
    vw = wiringPiI2CReadReg16(gyro_address, address);


    // convert from 2's complement
    if (vw > 0x8000)
    {
        vw = vw ^ 0xffff;
        vw = -vw - 1;
    }
    imu_data[5] = ((float)vw) / 32768.0 * 1000.0; // convert to degrees/sec
}

void print_imu_data()
{

    roll_angle = atan2(imu_data[2], imu_data[0]) * 180.0 / M_PI;
    pitch_angle = atan2(imu_data[1], sqrt(imu_data[0]*imu_data[0] + imu_data[2]*imu_data[2])) * 180.0 / M_PI;

    roll_angle -= roll_calibration;
    pitch_angle -= pitch_calibration;

    float gx = imu_data[3] - x_gyro_calibration;
    float gy = imu_data[4] - y_gyro_calibration;
    float gz = imu_data[5] - z_gyro_calibration;

    printf("Gx:%10.5f | Gy:%10.5f | Gz:%10.5f | Roll:%10.5f | Pitch:%10.5f\n\r", gx, gy, gz, roll_angle, pitch_angle);
}

int setup_imu()
{
    wiringPiSetup();

    // setup imu on I2C
    accel_address = wiringPiI2CSetup(0x19);

    gyro_address = wiringPiI2CSetup(0x69);

    if (accel_address == -1)
    {
        printf("-----cant connect to accel I2C device %d --------\n", accel_address);
        return -1;
    }
    else if (gyro_address == -1)
    {
        printf("-----cant connect to gyro I2C device %d --------\n", gyro_address);
        return -1;
    }
    else
    {
        printf("all i2c devices detected\n");
        sleep(1);
        wiringPiI2CWriteReg8(accel_address, 0x7d, 0x04); // power on accel
        wiringPiI2CWriteReg8(accel_address, 0x41, 0x00); // accel range to +_3g
        wiringPiI2CWriteReg8(accel_address, 0x40, 0x89); // high speed filtered accel

        wiringPiI2CWriteReg8(gyro_address, 0x11, 0x00); // power on gyro
        wiringPiI2CWriteReg8(gyro_address, 0x0F, 0x01); // set gyro to +-1000dps
        wiringPiI2CWriteReg8(gyro_address, 0x01, 0x03); // set data rate and bandwith

        sleep(1);
    }
    return 0;
}
