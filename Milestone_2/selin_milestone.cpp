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

#include <array>

// gcc -o week1 week_1_student.cpp -lwiringPi  -lm
#define ALPHA 0.02
#define MAX_GYRO_RATE 300
#define MAX_ANGLE 45
#define TIMEOUT_THRESHOLD 0.35


#define THURST_MIN 0 
#define THRUST_MAX 2000
#define THRUST_NEUTRAL 100
#define THURST_AMPLITUDE 100

#define PITCH_AMPLITUDE 10
#define PITCH_P_GAIN 10


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
float imu_data[6]; // accel xyz,  gyro xyz, raw values
long time_curr;
long time_prev;
struct timespec te;
float yaw = 0;
float pitch_angle = 0;
float roll_angle = 0;
float accels_and_gyros[6]; // accel xyz,  gyro xyz,

long last_joystick_time = 0;
long new_joystick_time = 0;
long last_joystick_sequence = 0;

float roll_filtered = 0; 
float pitch_filtered = 0;       
int sample = 0;    

float integrated_roll = 0;
float integrated_pitch = 0;
 

std::array <int, 4> motor_commands;

std::array<int, 4> pitch_signs = {1, -1, 1, -1};

struct Joystick {
    int key0;
    int key1;
    int key2;
    int key3;
    int pitch;
    int roll;
    int yaw;
    int thrust;
    int sequence_num;
};
    

Joystick* shared_memory;
int run_program=1;





void trap(int signal){
    
    printf("ending program\n\r");
    run_program=0;
}
    


void calibrate_imu()
{
    //printf("calibrating imu...\n");
    x_gyro_calibration = 0;
    y_gyro_calibration = 0;
    z_gyro_calibration = 0;
    roll_calibration = 0;
    pitch_calibration = 0;


    for (int i = 0; i < 1000; i++)
    {
        read_imu();

        x_gyro_calibration += imu_data[3];
        y_gyro_calibration += imu_data[4];
        z_gyro_calibration += imu_data[5];

        roll_calibration += atan2(imu_data[2], imu_data[0]) * 180.0 / M_PI;
        pitch_calibration += atan2(imu_data[1], sqrt(imu_data[0] * imu_data[0] + imu_data[2] * imu_data[2])) * 180.0 / M_PI;

    }

    x_gyro_calibration /= 1000.0;
    y_gyro_calibration /= 1000.0;
    z_gyro_calibration /= 1000.0;
    roll_calibration /= 1000.0;
    pitch_calibration /= 1000.0;


    //printf("calibration complete: %f %f %f %f %f %f\n", x_gyro_calibration, y_gyro_calibration, z_gyro_calibration, roll_calibration, pitch_calibration);

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
    imu_data[5] = -((float)vw) / 32768.0 * 1000.0; // convert to degrees/sec


    // calculate raw roll and pitch
    roll_angle = atan2(imu_data[2], imu_data[0]) * 180.0 / M_PI;
    pitch_angle = atan2(imu_data[1], sqrt(imu_data[0] * imu_data[0] + imu_data[2] * imu_data[2])) * 180.0 / M_PI;

    // calibration
    roll_angle -= roll_calibration;
    pitch_angle -= pitch_calibration;

    // accels without calibration
    accels_and_gyros[0] = imu_data[0];
    accels_and_gyros[1] = imu_data[1];
    accels_and_gyros[2] = imu_data[2];

    // calibrate gyro values
    accels_and_gyros[3] = imu_data[3] - x_gyro_calibration;
    accels_and_gyros[4] = imu_data[4] - y_gyro_calibration;
    accels_and_gyros[5] = imu_data[5] - z_gyro_calibration;

}

void print_imu_data()
{
    printf("%d, %10.5f, %10.5f, %10.5f, %10.5f, %10.5f, %10.5f\n", 
        sample, roll_filtered, roll_angle, integrated_roll, 
        pitch_filtered, pitch_angle, integrated_pitch);

}

void print_roll_data()
{
    printf("%d, %10.5f, %10.5f, %10.5f\n", 
        sample, roll_filtered, roll_angle, integrated_roll);

}

void print_pitch_data()
{
    printf("%d, %10.5f, %10.5f, %10.5f\n", 
        sample, pitch_angle, pitch_filtered, integrated_pitch);

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
        //printf("all i2c devices detected\n");
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


void update_filter(){
    //get current time in nanoseconds
    timespec_get(&te,TIME_UTC);
    time_curr=te.tv_nsec;
    
    //compute time since last execution
    float imu_diff=time_curr-time_prev;
    //check for rollover
    if(imu_diff<=0){
        imu_diff+=1000000000;
    }

    //convert to seconds
    imu_diff=imu_diff/1000000000;

    time_prev=time_curr;
    //comp. filter for roll, pitch here:

    float gyro_roll_dt = accels_and_gyros[4] * imu_diff;
    float gyro_pitch_dt = accels_and_gyros[5] * imu_diff;

    //roll_gyro = roll_filtered + accels_and_gyros[3] * dt; 
    //pitch_gyro = pitch_filtered + accels_and_gyros[4] * dt;

    //printf("roll gyro: %f, pitch gyro: %f\n", roll_gyro, pitch_gyro);

    //COMPLEMENTARY 
    roll_filtered = ALPHA * roll_angle + (1 - ALPHA) * (gyro_roll_dt + roll_filtered);
    pitch_filtered = ALPHA * pitch_angle + (1 - ALPHA) * (gyro_pitch_dt + pitch_filtered);

    integrated_roll += accels_and_gyros[4] * imu_diff;
    integrated_pitch += accels_and_gyros[5] * imu_diff;
    
}


    //function to add
void setup_joystick(){

    int segment_id;
    struct shmid_ds shmbuffer;
    int segment_size;
    const int shared_segment_size = 0x6400;
    int smhkey=33222;

    /* Allocate a shared memory segment. */
    segment_id = shmget (smhkey, shared_segment_size,IPC_CREAT | 0666);

    /* Attach the shared memory segment. */
    shared_memory = (Joystick*) shmat (segment_id, 0, 0);
    printf ("shared memory attached at address %p\n", shared_memory);

    /* Determine the segment's size. */
    shmctl (segment_id, IPC_STAT, &shmbuffer);
    segment_size = shmbuffer.shm_segsz;
    printf ("segment size: %d\n", segment_size);


    /* Write a string to the shared memory segment. */
    //sprintf (shared_memory, "test!!!!.");
}
    //when cntrl+c pressed, kill motors

void safety_check(Joystick data){

    timespec_get(&te,TIME_UTC);
    time_curr=te.tv_nsec;

    float _diff = time_curr - last_joystick_time;
    if(_diff <= 0){
        _diff += 1000000000;
    }
     

    if(last_joystick_sequence != data.sequence_num){
        
        last_joystick_time = time_curr;
        last_joystick_sequence = data.sequence_num;        
    }
    else if(_diff / 1000000000 > TIMEOUT_THRESHOLD ){
        printf("_diff = %f seconds\n", _diff / 1000000000.0f);
        printf("Joystick timeout, killing program\n");
        run_program = 0;   
    }
    
    
    //gyro rates
    if(fabs(accels_and_gyros[3]) > MAX_GYRO_RATE || 
        fabs(accels_and_gyros[4]) > MAX_GYRO_RATE || 
        fabs(accels_and_gyros[5]) > MAX_GYRO_RATE){

        printf("Gyro rate exceeded, killing program\n");
        run_program = 0;
    }

    if(fabs(roll_filtered) > MAX_ANGLE ){
        printf("Roll angle exceeded, killing program\n");
        run_program = 0;
    }

    if(fabs(pitch_filtered) > MAX_ANGLE ){
        printf("Pitch angle exceeded, killing program\n");
        run_program = 0;
    }

    if(data.key1 == 1){
        printf("Button B pressed, killing program\n");
        run_program = 0;
    }
}



void set_motors(Joystick data){
    int init_thrust = (THRUST_NEUTRAL + THURST_AMPLITUDE) * -(data.thrust - 128) / 127;
    motor_commands.fill(init_thrust);

    int pitch_desired = PITCH_AMPLITUDE * (data.pitch - 128) / 127;
    int pitch_error =  pitch_desired - pitch_filtered;

    int pitch_correction = PITCH_P_GAIN * pitch_error;
    //pitch_corrections = {PITCH_P_GAIN * pitch_error, PITCH_P_GAIN * pitch_error, -(PITCH_P_GAIN * pitch_error), -(PITCH_P_GAIN * pitch_error)};


    for (int i = 0; i < motor_commands.size(); i++){
        motor_commands[i] += pitch_correction * pitch_signs[i];
    }
}

void print_motors(){
    printf("%d, %d, %10.5f, %10.5f \n\r" , motor_commands[0], motor_commands[2], pitch_filtered, accels_and_gyros[4]);
}

int main(int argc, char *argv[])
{
    setup_imu();
    calibrate_imu();
    setup_joystick();
    last_joystick_sequence = shared_memory->sequence_num;
    signal(SIGINT, &trap);

    timespec_get(&te, TIME_UTC);
    last_joystick_time = te.tv_nsec;

    Joystick joystick_data=*shared_memory;
    
    //be sure to update the while(1) in main to use run_program instead
    //printf("Sample, Pitch_angle, Pitch_filtered, Integrated_pitch, \n");
    while (run_program == 1)
    {
        read_imu();
        joystick_data = *shared_memory;
        safety_check(joystick_data);
        
        //printf("1\n");
        update_filter();
        //printf("2\n");
        //print_pitch_data();
        //printf("3\n");
        
        set_motors(joystick_data);
        print_motors();
        sample++;
    }
}
