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

//Defines
#define MAX_GYRO_RATE 600.0
#define MAX_ROLL_ANGLE 45.0
#define MAX_PITCH_ANGLE 45.0
#define JOYSTICK_TIMEOUT_S 0.75

#define THURST_MIN 0 
#define THRUST_MAX 2000
#define THRUST_NEUTRAL 1400
#define THURST_AMPLITUDE 200

#define PITCH_AMPLITUDE 6 //10
#define PITCH_P_GAIN 20 //15
#define PITCH_D_GAIN 3 //3
#define PITCH_I_GAIN 4 //4
#define I_SATURATE_PITCH 100 //100

#define ROLL_AMPLITUDE 6 //10
#define ROLL_I_GAIN 4 //4
#define ROLL_P_GAIN 28 //15 
#define ROLL_D_GAIN 4 //3
#define I_SATURATE_ROLL 100 //100

#define YAW_AMPLITUDE 70
#define YAW_P_GAIN 9

// Structs
struct Joystick
{
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

//Globals
int accel_address, gyro_address;

float x_gyro_calibration = 0;
float y_gyro_calibration = 0;
float z_gyro_calibration = 0;

float roll_calibration = 0;
float pitch_calibration = 0;

float imu_data[6];
float accels_and_gyros[6];

float yaw = 0;
float pitch_angle = 0;
float roll_angle = 0;

float f_roll_angle = 0;
float f_pitch_angle = 0;

float i_roll = 0;
float i_pitch = 0;

long time_curr;
long time_prev;

float joy_time_curr;
float joy_time_prev;

int last_sequence = 0;
int joy_initialized = 0;
int run_program = 1;

int init_thrust = 0;
int pitch_desired = 0;
float pitch_error = 0.0;

float integ_pitch_err = 0.0;

float integ_roll_err = 0.0;
float roll_error = 0.0;
int roll_desired = 0;

float yaw_error = 0.0;
int yaw_desired = 0;

double joy_dt = 0.0;

int time_curr_I;
int time_prev_I;


int motor_address,acceleration_address,gyroscope_address;
int motor0, motor1, motor2, motor3;

bool motors_paused = false;


// int pitch_prev_error = 0;
// int time_curr_D;
// int time_prev_D;

struct timespec te;

Joystick *shared_memory;


std::array <int, 4> motor_commands;

std::array<int, 4> pitch_signs = {-1, 1, -1, 1};
std::array<int, 4> roll_signs = {-1, -1, 1, 1};
std::array<int, 4> yaw_signs = {-1, 1, 1, -1};

//Function prototypes
int setup_imu();
void calibrate_imu();
void read_imu();
void update_filter();

void print_imu_data();
void print_roll_data();
void print_pitch_data();
void print_motors();

void setup_joystick();
void safety_check(Joystick data);
void trap(int signal);

void set_motors_P(Joystick data);
void set_motors_D(Joystick data);
void set_motors_I(Joystick data);
void set_motors_PID(Joystick data);
void set_motors(std::array<int, 4> motors);
void motor_enable();

bool is_motors_paused(Joystick data);

int main(int argc, char *argv[])
{

    setup_imu();
    calibrate_imu();
    motor_address=wiringPiI2CSetup(0x56);
    motor_enable();
    setup_joystick();
    
    last_sequence = shared_memory->sequence_num;
    signal(SIGINT, &trap);

    timespec_get(&te, TIME_UTC);
    joy_time_prev = te.tv_nsec;

    Joystick joystick_data=*shared_memory;

    while (run_program == 1)
    {
        //while(!is_motors_paused(joystick_data)){
            read_imu();
            update_filter();
            joystick_data = *shared_memory;
            safety_check(joystick_data);

            if(is_motors_paused(joystick_data)){
                printf("paused ... \n");
            }
            if(!is_motors_paused(joystick_data)){ 
                
                set_motors_PID(joystick_data);
                
            }
            set_motors(motor_commands);
            //print_motors();
        //}
    }
    return 0;
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

void calibrate_imu()
{
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
    imu_data[5] = -((float)vw) / 32768.0 * 1000.0; // convert to degrees/sec

    ////////////////////COMPUTATION OF ROLL/PITCH AND CALIBRATION

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

void update_filter()
{
    // get current time in nanoseconds
    timespec_get(&te, TIME_UTC);
    time_curr = te.tv_nsec;
    // compute time since last execution
    float imu_diff = time_curr - time_prev;
    // check for rollover
    if (imu_diff <= 0)
    {
        imu_diff += 1000000000;
    }
    // convert to seconds
    imu_diff = imu_diff / 1000000000;
    time_prev = time_curr;
    // comp. filter for roll, pitch here:
    float A = 0.02;

    // gyro delta (deg)
    float roll_gyro_delta = accels_and_gyros[4] * imu_diff;
    float pitch_gyro_delta = accels_and_gyros[5] * imu_diff;

    // complementary filter
    f_roll_angle = A * roll_angle + (1 - A) * (roll_gyro_delta + f_roll_angle);
    f_pitch_angle = A * pitch_angle + (1 - A) * (pitch_gyro_delta + f_pitch_angle);

    // integrated gyro output
    i_roll += accels_and_gyros[4] * imu_diff;
    i_pitch += accels_and_gyros[5] * imu_diff;
}

void setup_joystick()
{
    int segment_id;
    struct shmid_ds shmbuffer;
    int segment_size;
    const int shared_segment_size = 0x6400;
    int smhkey = 33222;
    /* Allocate a shared memory segment. */
    segment_id = shmget(smhkey, shared_segment_size, IPC_CREAT | 0666);
    /* Attach the shared memory segment. */
    shared_memory = (Joystick *)shmat(segment_id, 0, 0);
    printf("shared memory attached at address %p\n", shared_memory);
    /* Determine the segment's size. */
    shmctl(segment_id, IPC_STAT, &shmbuffer);
    segment_size = shmbuffer.shm_segsz;
    printf("segment size: %d\n", segment_size);
    /* Write a string to the shared memory segment. */
    // sprintf (shared_memory, "test!!!!.");
}

void trap(int signal)
{
    printf("ending program\n\r");
    run_program = 0;
}

void safety_check(Joystick data)
{
    timespec_get(&te, TIME_UTC);
    joy_time_curr = ((float) te.tv_nsec) * 1e-9;

    //Joystick joystick_data = *shared_memory;

    if (!joy_initialized)
    {
        joy_time_prev = joy_time_curr;
        last_sequence = data.sequence_num;
        joy_initialized = 1;
        return;
    }

    // reset timer when new packet arrives
    if (data.sequence_num != last_sequence)
    {
        last_sequence = data.sequence_num;
        joy_time_prev = joy_time_curr;
    }

    if (joy_time_curr < joy_time_prev)
    {
        joy_time_curr =+ 1.0;
    }

        joy_dt = joy_time_curr - joy_time_prev;

    if (joy_dt > JOYSTICK_TIMEOUT_S)
    {
        printf("SAFETY STOP: Joystick timeout with last time %f and current time %f\n\r", joy_time_prev, joy_time_curr);
        run_program = 0;
    }

    // exceed gyro rate
    if (fabs(accels_and_gyros[3]) > MAX_GYRO_RATE ||
        fabs(accels_and_gyros[4]) > MAX_GYRO_RATE ||
        fabs(accels_and_gyros[5]) > MAX_GYRO_RATE)
    {
        printf("SAFETY STOP: Gyro rate exceeded limit of %f\n\r", MAX_GYRO_RATE);
        run_program = 0;
    }

    // exceed roll angle
    if (fabs(f_roll_angle) > MAX_ROLL_ANGLE)
    {
        printf("SAFETY STOP: Roll angle exceeded limit of %f\n\r", MAX_ROLL_ANGLE);
        run_program = 0;
    }

    // exceed pitch angle
    if (fabs(f_pitch_angle) > MAX_PITCH_ANGLE)
    {
        printf("SAFETY STOP: Pitch angle exceeded limit of %f\n\r", MAX_PITCH_ANGLE);
        run_program = 0;
    }

    // b button pressed
    if (data.key1 == 1)
    {
        printf("SAFETY STOP: B pressed\n\r");
        run_program = 0;
    }
}



void set_motors_PID(Joystick data){

    timespec_get(&te, TIME_UTC);
    time_curr_I = te.tv_nsec;
    // compute time since last execution
    float imu_diff = time_curr_I - time_prev_I;
    // check for rollover
    if (imu_diff <= 0)
    {
        imu_diff += 1000000000;
    }
    // convert to seconds
    imu_diff = imu_diff / 1000000000;
    


    //
    init_thrust = THRUST_NEUTRAL + float(THURST_AMPLITUDE * -(data.thrust - 128)) / 127.0;
    motor_commands.fill(init_thrust);

    pitch_desired = PITCH_AMPLITUDE * (data.pitch - 128) / 127;
    pitch_error =  (float)pitch_desired - f_pitch_angle;

    roll_desired = ROLL_AMPLITUDE * (data.roll - 128) / 127;
    roll_error = (float)roll_desired - f_roll_angle;

    yaw_desired = -(YAW_AMPLITUDE * (data.yaw - 128) / 127);
    yaw_error = (float)yaw_desired - accels_and_gyros[3];

    integ_pitch_err += imu_diff *float(pitch_error)*(float)PITCH_I_GAIN;
    integ_roll_err += imu_diff *float(roll_error)*(float)ROLL_I_GAIN;

    if(integ_pitch_err > I_SATURATE_PITCH){
        integ_pitch_err = I_SATURATE_PITCH;
    }
    else if(integ_pitch_err < -I_SATURATE_PITCH){
        integ_pitch_err = -I_SATURATE_PITCH;
    }

    if(integ_pitch_err > I_SATURATE_ROLL){
        integ_pitch_err = I_SATURATE_ROLL;
    }
    else if(integ_pitch_err < -I_SATURATE_ROLL){
        integ_pitch_err = -I_SATURATE_ROLL;
    }

    int pitch_correction = integ_pitch_err + PITCH_P_GAIN * pitch_error - PITCH_D_GAIN * accels_and_gyros[5];
    int roll_correction = integ_roll_err + ROLL_P_GAIN * roll_error - ROLL_D_GAIN * accels_and_gyros[4];
    //pitch_corrections = {PITCH_P_GAIN * pitch_error, PITCH_P_GAIN * pitch_error, -(PITCH_P_GAIN * pitch_error), -(PITCH_P_GAIN * pitch_error)};

    int yaw_correction = yaw_error * YAW_P_GAIN;
    


    motor_commands[0] = motor_commands[0] + pitch_correction * pitch_signs[0] + roll_correction * roll_signs[0] + yaw_correction * yaw_signs[0];
    motor_commands[1] = motor_commands[1] + pitch_correction * pitch_signs[1] + roll_correction * roll_signs[1] + yaw_correction * yaw_signs[1];
    motor_commands[2] = motor_commands[2] + pitch_correction * pitch_signs[2] + roll_correction * roll_signs[2] + yaw_correction * yaw_signs[2];
    motor_commands[3] = motor_commands[3] + pitch_correction * pitch_signs[3] + roll_correction * roll_signs[3] + yaw_correction * yaw_signs[3];



    // if (run_program == 0) {
    //     motor_commands.fill(0);
    // }

    // if(motor_commands[0] > THRUST_MAX || motor_commands[1] > THRUST_MAX || motor_commands[2] > THRUST_MAX || motor_commands[3] > THRUST_MAX){
    //     motor_commands.fill(THRUST_MAX);
    // }
    // else if (motor_commands[0] < 0 || motor_commands[1] < 0 || motor_commands[2] < 0 || motor_commands[3] < 0){
    //     motor_commands.fill(0);
    // }

    

    time_prev_I = time_curr_I;
    

    //printf("Motors paused...");
}


bool is_motors_paused(Joystick data){
    //A = key0
    //Y = key3
    if(data.key0 == 1){
        motor_commands.fill(2);
        motors_paused = true;
    }

    if(data.key3 == 1){
        
        motors_paused = false;
    }
    return motors_paused;
}


void print_motors(){
    //printf("%d, %d, %10.5f, %10.5f \n\r" , motor_commands[0], motor_commands[1], f_pitch_angle, accels_and_gyros[4]);
    printf("%10.5f, %d, %d, %d, %d, %d \n\r" , f_pitch_angle, pitch_desired, motor_commands[0], motor_commands[1], motor_commands[2], motor_commands[3]); //pitch_angle);
}


//add global variable
//in main() after wiringPiSetup() add
void motor_enable()
{
    uint8_t motor_id=0;
    uint8_t special_command=0;
    uint16_t commanded_speed_0=1000;
    uint16_t commanded_speed_1=0;
    uint16_t commanded_speed=0;
    uint8_t data[2];
    int cal_delay=50;

    for(int i=0;i<1000;i++)
    {
        motor_id=0;
        commanded_speed=0;
        data[0] = 0x80 + (motor_id<<5) + (special_command<<4) + ((commanded_speed>>7)&0x0f);
        data[1] = commanded_speed&0x7f;
        wiringPiI2CWrite(motor_address,data[0]);
        
        usleep(cal_delay);
        
        wiringPiI2CWrite(motor_address,data[1]);
        usleep(cal_delay);
       
        motor_id=1;
        commanded_speed=0;
        data[0] = 0x80 + (motor_id<<5) + (special_command<<4) + ((commanded_speed>>7)&0x0f);
        data[1]=commanded_speed&0x7f;
        wiringPiI2CWrite(motor_address,data[0]);
        usleep(cal_delay);
        wiringPiI2CWrite(motor_address,data[1]);
        usleep(cal_delay);
        
        motor_id=2;
        commanded_speed=0;
        data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
        data[1]=commanded_speed&0x7f;
        wiringPiI2CWrite(motor_address,data[0]);
        usleep(cal_delay);
        wiringPiI2CWrite(motor_address,data[1]);
        usleep(cal_delay);
        
        motor_id=3;
        commanded_speed=0;
        data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7
        )&0x0f);
        data[1]=commanded_speed&0x7f;
        wiringPiI2CWrite(motor_address,data[0]);
        usleep(cal_delay);
        wiringPiI2CWrite(motor_address,data[1]);
        usleep(cal_delay);
    }

for(int i=0;i<2000;i++)
    {
        motor_id=0;
        commanded_speed=50;
        data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
        data[1]=commanded_speed&0x7f;
        wiringPiI2CWrite(motor_address,data[0]);
        usleep(cal_delay);
        wiringPiI2CWrite(motor_address,data[1]);
        usleep(cal_delay);
        
        motor_id=1;
        commanded_speed=50;
        data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
        data[1]=commanded_speed&0x7f;
        wiringPiI2CWrite(motor_address,data[0]);
        usleep(cal_delay);
        wiringPiI2CWrite(motor_address,data[1]);
        usleep(cal_delay);
        
        motor_id=2;
        commanded_speed=50;
        data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
        data[1]=commanded_speed&0x7f;
        wiringPiI2CWrite(motor_address,data[0]);
        usleep(cal_delay);
        wiringPiI2CWrite(motor_address,data[1]);
        usleep(cal_delay);
        
        motor_id=3;
        commanded_speed=50;
        data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
        data[1]=commanded_speed&0x7f;
        wiringPiI2CWrite(motor_address,data[0]);
        usleep(cal_delay);
        wiringPiI2CWrite(motor_address,data[1]);
        usleep(cal_delay);
    }
for(int i=0;i<500;i++)
    {
        motor_id=0;
        commanded_speed=0;
        data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
        data[1]=commanded_speed&0x7f;
        wiringPiI2CWrite(motor_address,data[0]);
        usleep(cal_delay);
        wiringPiI2CWrite(motor_address,data[1]);
        usleep(cal_delay);
        
        motor_id=1;
        commanded_speed=0;
        data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
        data[1]=commanded_speed&0x7f;
        wiringPiI2CWrite(motor_address,data[0]);
        usleep(cal_delay);
        wiringPiI2CWrite(motor_address,data[1]);
        usleep(cal_delay);
       
        motor_id=2;
        commanded_speed=0;
        data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
        data[1]=commanded_speed&0x7f;
        wiringPiI2CWrite(motor_address,data[0]);
        usleep(cal_delay);
        wiringPiI2CWrite(motor_address,data[1]);
        usleep(cal_delay);
        
        motor_id=3;
        commanded_speed=0;
        data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
        data[1]=commanded_speed&0x7f;
        wiringPiI2CWrite(motor_address,data[0]);
        usleep(cal_delay);
        wiringPiI2CWrite(motor_address,data[1]);
        usleep(cal_delay);
    }
}
void set_motors(std::array<int, 4> motors)
{
    
    motor0=motors[0];   
    motor1=motors[1];
    motor2=motors[2];
    motor3=motors[3];

    if(motor0<0)
        motor0=0;

    if(motor0>2000)
        motor0=2000;

    if(motor1<0)
        motor1=0;

    if(motor1>2000)
        motor1=2000;

    if(motor2<0)
        motor2=0;

    if(motor2>2000)
        motor2=2000;

    if(motor3<0)
        motor3=0;

    if(motor3>2000)
        motor3=2000;

    uint8_t motor_id=0;
    uint8_t special_command=0;
    uint16_t commanded_speed_0=1000;
    uint16_t commanded_speed_1=0;
    uint16_t commanded_speed=0;
    uint8_t data[2];

    // wiringPiI2CWriteReg8(motor_address, 0x00,data[0] );
    //wiringPiI2CWrite (motor_address,data[0]) ;
    int com_delay=500;
    motor_id=0;
    commanded_speed=motor0;
    data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
    data[1]=commanded_speed&0x7f;
    wiringPiI2CWrite(motor_address,data[0]);
    usleep(com_delay);
    wiringPiI2CWrite(motor_address,data[1]);
    usleep(com_delay);
    
    motor_id=1;
    commanded_speed=motor1;
    data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&
    0x0f);
    data[1]=commanded_speed&0x7f;
    wiringPiI2CWrite(motor_address,data[0]);
    usleep(com_delay);
    wiringPiI2CWrite(motor_address,data[1]);
    usleep(com_delay);
    
    motor_id=2;
    commanded_speed=motor2;
    data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&
    0x0f);
    data[1]=commanded_speed&0x7f;
    wiringPiI2CWrite(motor_address,data[0]);
    usleep(com_delay);
    wiringPiI2CWrite(motor_address,data[1]);
    usleep(com_delay);
    
    motor_id=3;
    commanded_speed=motor3;
    data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&
    0x0f);
    data[1]=commanded_speed&0x7f; 
    wiringPiI2CWrite(motor_address,data[0]);
    usleep(com_delay);
    wiringPiI2CWrite(motor_address,data[1]);
    usleep(com_delay);
}