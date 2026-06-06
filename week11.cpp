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
#include <string>


// ====================================================================
// DEFINES
// ====================================================================

//safety limits
#define MAX_GYRO_RATE 600.0
#define MAX_ROLL_ANGLE 45.0
#define MAX_PITCH_ANGLE 45.0
#define JOYSTICK_TIMEOUT_S 0.75

// thrust
#define THRUST_MIN 0 
#define THRUST_MAX 2000
#define THRUST_NEUTRAL 1300
#define THRUST_AMPLITUDE 300

// pitch PID
#define PITCH_AMPLITUDE 6 //10
#define PITCH_P_GAIN 20 //15
#define PITCH_D_GAIN 3 //3
#define PITCH_I_GAIN 4 //4
#define I_SATURATE_PITCH 100 //100

// roll PID
#define ROLL_AMPLITUDE 6 //10
#define ROLL_P_GAIN 28 //15 
#define ROLL_D_GAIN 4 //3
#define ROLL_I_GAIN 4 //4
#define I_SATURATE_ROLL 100 //100

// yaw control
#define YAW_AMPLITUDE 70
#define YAW_P_GAIN_CTRL 5

#define YAW_P_GAIN_AUTON 1


// camera autnomous control
#define P_CAM 40  //
#define D_CAM 10
#define P_CAM_THRUST 0
#define D_CAM_THRUST 0.0
#define I_CAM_THRUST 0.0
#define I_CAM_SAT 100
#define AUTO_X 0
#define AUTO_Y 0
#define AUTO_Z 0
#define CAM_FILTER_ALPHA 0.4

// ====================================================================
// STRUCTS
// ====================================================================

struct data // data struct from udp_rx file
{
    int key0;
    int key1;
    int key2;
    int key3;
    int pitch;
    int roll;
    int yaw;
    int thrust;
    float x;
    float y;
    float z;
    float camera_yaw;
    int success;
    int sequence_num;
};

// ====================================================================
// GLOBALS
// ====================================================================

// hardware
int motor_address, accel_address, gyro_address;
int motor0, motor1, motor2, motor3;
struct timespec te;

// IMU and filter
float imu_data[6];
float accels_and_gyros[6];

float x_gyro_calibration = 0;
float y_gyro_calibration = 0;
float z_gyro_calibration = 0;
float roll_calibration = 0;
float pitch_calibration = 0;

float yaw = 0;
float pitch_angle = 0;
float roll_angle = 0;

float f_roll_angle = 0;     // complementary filter output
float f_pitch_angle = 0;

float i_roll = 0;           // integrated gyro (referenced not used)
float i_pitch = 0;

long time_curr;             // timestamps for IMU filter
long time_prev;

// control state
int init_thrust = 0;

float pitch_actual = 0.0;
float roll_actual = 0.0;
float yaw_actual = 0.0;

float pitch_desired = 0.0;
float roll_desired = 0.0;
float yaw_desired = 0.0;
float thrust_desired = 0.0;

float pitch_error = 0.0;
float roll_error = 0.0;
float yaw_error = 0.0;

float integ_pitch_err = 0.0;
float integ_roll_err = 0.0;

int p_gain_yaw = 0;

int time_curr_I;
int time_prev_I;

std::array <int, 4> motor_commands;

std::array<int, 4> pitch_signs = {-1, 1, -1, 1};
std::array<int, 4> roll_signs = {-1, -1, 1, 1};
std::array<int, 4> yaw_signs = {-1, 1, 1, -1};

// joystick and safeties
data joystick_data;
data *shared_memory;

int last_sequence = 0;
int joy_initialized = 0;
int run_program = 1;

float joy_time_curr = 0;
float joy_time_prev = 0;
double joy_dt = 0.0;


bool motors_paused = false;
bool auton = false;
bool last_state_x = false;

// camera autoomy
float cam_x_prev = 0.0;
float cam_y_prev = 0.0;
float cam_z_prev = 0.0;

float cam_x_estimated = 0.0;
float cam_y_estimated = 0.0;
float cam_z_estimated = 0.0;

float cam_time_prev = 0.0;
int last_cam_sequence = 0;

float auto_thrust_error = 0.0;
float auto_thrust_P = 0.0;
float auto_thrust_D = 0.0;
float auto_i_thrust_err_ = 0.0;

// logging for tuning 
double prog_t0   = 0.0;   // program start time, for an elapsed-time x-axis
float  cam_dt_log = 0.0;  // last camera dt
float  y_cam_P = 0.0;     // camera P term -> desired pitch (Y axis)
float  y_cam_D = 0.0;     // camera D term -> desired pitch
float  x_cam_P = 0.0;     // camera P term -> desired roll  (X axis)
float  x_cam_D = 0.0;     // camera D term -> desired roll
float  y_vel_filt = 0.0;     // filtered camera Y velocity for logging and tuning
float x_vel_filt = 0.0;

// ====================================================================
// Function defining
// ====================================================================

// hardware
int setup_imu();
void calibrate_imu();
void motor_enable();
void setup_joystick();

// IMU
void read_imu();
void update_filter();

// control
void set_motors_PID(data data, bool autonomous);
void set_motors(std::array<int, 4> motors);
void camera_control();

// state and safeties
void safety_check(data data);
void is_auton(data button);
bool is_motors_paused(data data);
void trap(int signal);

// debug
void double_check_print();
void print_motors();

void print_imu_data();
void print_roll_data();
void print_pitch_data();
void print_for_logging(std::string label);

// milestoning
/*
void set_motors_P(data data);
void set_motors_D(data data);
void set_motors_I(data data);
*/

// ====================================================================
// Function defining
// ====================================================================

//using namespace std;

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
    prog_t0 = (double)te.tv_sec + (double)te.tv_nsec * 1e-9;


    printf("time_s, cam_y_raw, cam_y_est, y_P, y_D, pitch_des, pitch_act, cam_dt, integ_pitch_err\n");

    joystick_data=*shared_memory;

    while (run_program == 1)
    {
        read_imu();
        update_filter();

        joystick_data = *shared_memory;
        safety_check(joystick_data);
        is_auton(joystick_data);

        if(!is_motors_paused(joystick_data))
        {
            set_motors_PID(joystick_data, auton);
        }


        set_motors(motor_commands);
        //double_check_print();
        //print_motors();
        print_for_logging("pitch");
    }
    return 0;
}

// ====================================================================
// debug print
// ====================================================================
void double_check_print()
{
    printf(
        "joystick: %d %d %d %d | yaw: %d pitch: %d roll: %d thrust: %d | "
        "x: %.3f y: %.3f z: %.3f yaw: %.3f | success: %d seq: %d\n",
        joystick_data.key0,
        joystick_data.key1,
        joystick_data.key2,
        joystick_data.key3,
        joystick_data.yaw,
        joystick_data.pitch,
        joystick_data.roll,
        joystick_data.thrust,
        joystick_data.x,
        joystick_data.y,
        joystick_data.z,
        joystick_data.camera_yaw,
        joystick_data.success,
        joystick_data.sequence_num
        );
}

void print_motors(){
    //printf("%d, %d, %d, %d \n\r" , motor_commands[0], motor_commands[1], motor_commands[2], motor_commands[3]);
    //printf("%10.5f, %d, %d, %d, %d, %d \n\r" , f_pitch_angle, pitch_desired, motor_commands[0], motor_commands[1], motor_commands[2], motor_commands[3]); //pitch_angle);
    //if (auton){
        //printf("%10.5f \n\r", (-joystick_data.camera_yaw));
    printf("pitch: %.3f, roll: %.3f, thrust: %.3f \n\r", pitch_desired, roll_desired, thrust_desired);
    //}
}

void print_for_logging(std::string label){
    timespec_get(&te, TIME_UTC);
    double now = (double)te.tv_sec + (double)te.tv_nsec * 1e-9;
    float t = (float)(now - prog_t0);

    if(label == "pitch"){
        printf("%.4f, %.4f, %.4f, %.3f, %.3f, %.3f, %.3f, %.5f, %.5f\n",
           t,                 // time_s
           joystick_data.y,   // cam_y_raw   
           cam_y_estimated,   // cam_y_est   
           y_cam_P,           // y_P term
           y_cam_D,           // y_D term
           pitch_desired,     // pitch_des   
           f_pitch_angle,     // pitch_act   
           cam_dt_log,
           integ_pitch_err    // integrated pitch error
           );
    }
    else if(label == "roll"){
        printf("%.4f, %.4f, %.4f, %.3f, %.3f, %.3f, %.3f, %.5f\n",
           t,                 // time_s
           joystick_data.x,   // cam_x_raw   
           cam_x_estimated,   // cam_x_est   
           x_cam_P,           // x_P term
           x_cam_D,           // x_D term
           roll_desired,      // roll_des    
           f_roll_angle,      // roll_act   
           cam_dt_log);       // cam_dt
    }

}

// ====================================================================
// IMU setup and calibration
// ====================================================================

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

// ====================================================================
// IMU read and filter
// ====================================================================

void read_imu()
{
    int vw = 0;

    //-----------------------------accel reads---------------------------------------

    int address = 0x12; // x accel
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

    float imu_diff = time_curr - time_prev; // compute time since last execution
    if (imu_diff <= 0) // check for rollover
    {
        imu_diff += 1000000000;
    }
    
    imu_diff = imu_diff / 1000000000; // convert to seconds

    time_prev = time_curr;

    float A = 0.02; // comp. filter for roll, pitch here:

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

// ====================================================================
// camera autonomy
// ====================================================================

void camera_control()
{
    // only on a new frame
    if (joystick_data.sequence_num == last_cam_sequence || joystick_data.success != 1){
        return;
    }

    // camera dt
    timespec_get(&te, TIME_UTC);
    float cam_time_curr = (float)te.tv_nsec * 1e-9f;
    float cam_dt = cam_time_curr - cam_time_prev;
    if (cam_dt <= 0.0){
        cam_dt += 1.0f;
    } 

    //exponential filter
    cam_y_estimated = cam_y_estimated * (1.0 - CAM_FILTER_ALPHA) + joystick_data.y * CAM_FILTER_ALPHA;
    cam_x_estimated = cam_x_estimated * (1.0 - CAM_FILTER_ALPHA) + joystick_data.x * CAM_FILTER_ALPHA;
    cam_z_estimated = cam_z_estimated * (1.0 - CAM_FILTER_ALPHA) + joystick_data.z * CAM_FILTER_ALPHA;

    //camera control for pitch (Y axis) - seperated for logging and tuning
    float y_velocity = (cam_y_estimated - cam_y_prev) / cam_dt;
    y_cam_P = -P_CAM * (cam_y_estimated - AUTO_Y);
    y_cam_D = D_CAM * y_velocity;
    //y_vel_filt = y_vel_filt*0.8 + y_velocity*0.2; 
    pitch_desired = 0.5*(y_cam_P + y_cam_D) + 0.5*((float)(PITCH_AMPLITUDE * (joystick_data.pitch - 128)) / 127.0);

    //camera control for roll (X axis) - seperated for logging and tuning
    float x_velocity = (cam_x_estimated - cam_x_prev) / cam_dt;
    x_cam_P = -P_CAM * (cam_x_estimated - AUTO_X);
    x_cam_D = D_CAM * x_velocity;
    //x_vel_filt = x_vel_filt * 0.8f + x_velocity * 0.2f;
    roll_desired = 0.5*(x_cam_P + x_cam_D) + 0.5*((float)(ROLL_AMPLITUDE * (joystick_data.roll - 128)) / 127.0);



    yaw_desired = 0;

    

    auto_thrust_error = AUTO_Z - joystick_data.z; //camera is above drone
    auto_thrust_P = auto_thrust_error*P_CAM_THRUST;
    auto_thrust_D = D_CAM_THRUST * (cam_z_estimated - cam_z_prev) / cam_dt;

    auto_i_thrust_err_ += cam_dt * auto_thrust_error * I_CAM_THRUST;

    if(auto_i_thrust_err_ > I_CAM_SAT){
        auto_i_thrust_err_ = I_CAM_SAT;
    }
    else if(auto_i_thrust_err_ < -I_CAM_SAT){
        auto_i_thrust_err_ = -I_CAM_SAT;
    }
    

    thrust_desired = 0.5*(auto_i_thrust_err_ + auto_thrust_P + auto_thrust_D) + 0.5 * ((float)(THRUST_AMPLITUDE * -(joystick_data.thrust - 128)) / 127.0);


    cam_y_prev = cam_y_estimated;
    cam_x_prev = cam_x_estimated;
    cam_z_prev = cam_z_estimated;
    cam_time_prev = cam_time_curr;
    cam_dt_log = cam_dt;
    last_cam_sequence = joystick_data.sequence_num;
}

// ====================================================================
// motor PID
// ====================================================================


void set_motors_PID(data data, bool autonomous){

    timespec_get(&te, TIME_UTC);
    time_curr_I = te.tv_nsec;
    float imu_diff = time_curr_I - time_prev_I;
    if (imu_diff <= 0)
    {
        imu_diff += 1000000000;
    }
    imu_diff = imu_diff / 1000000000;

    if(autonomous){
        // finds desired values from camera
        camera_control();

        integ_pitch_err = 0.0;
        integ_roll_err = 0.0;
        yaw_actual = -data.camera_yaw;
        p_gain_yaw = YAW_P_GAIN_AUTON; 

    }

    else{
        pitch_desired = (float)(PITCH_AMPLITUDE * (data.pitch - 128)) / 127.0; 
        roll_desired = (float)(ROLL_AMPLITUDE * (data.roll - 128)) / 127.0; 
        yaw_desired = -(float)(YAW_AMPLITUDE * (data.yaw - 128)) / 127.0;

        thrust_desired = (float)(THRUST_AMPLITUDE * -(data.thrust - 128)) / 127.0;

        yaw_actual = accels_and_gyros[3];

        p_gain_yaw = YAW_P_GAIN_CTRL; 
    }

    pitch_actual = f_pitch_angle;
    roll_actual = f_roll_angle;

    init_thrust = THRUST_NEUTRAL + thrust_desired;
    motor_commands.fill(init_thrust);

    pitch_error = pitch_desired - pitch_actual;
    roll_error = roll_desired - roll_actual;
    yaw_error = yaw_desired - yaw_actual;

    integ_pitch_err += imu_diff * pitch_error * PITCH_I_GAIN;
    integ_roll_err += imu_diff * roll_error * ROLL_I_GAIN;

    if(integ_pitch_err > I_SATURATE_PITCH){
        integ_pitch_err = I_SATURATE_PITCH;
    }
    else if(integ_pitch_err < -I_SATURATE_PITCH){
        integ_pitch_err = -I_SATURATE_PITCH;
    }

    if(integ_roll_err > I_SATURATE_ROLL){
        integ_roll_err = I_SATURATE_ROLL;
    }
    else if(integ_roll_err < -I_SATURATE_ROLL){
        integ_roll_err = -I_SATURATE_ROLL;
    }

    float pitch_correction = integ_pitch_err + PITCH_P_GAIN * pitch_error - PITCH_D_GAIN * accels_and_gyros[5];
    float roll_correction = integ_roll_err + ROLL_P_GAIN * roll_error - ROLL_D_GAIN * accels_and_gyros[4];
    float yaw_correction = yaw_error * p_gain_yaw;
    
    for (int i = 0; i < 4; i++)
    {
        motor_commands[i] += pitch_correction * pitch_signs[i]
                           + roll_correction  * roll_signs[i]
                           + yaw_correction   * yaw_signs[i];
    }

    time_prev_I = time_curr_I;
}

// ====================================================================
// safety and state
// ====================================================================

void safety_check(data data)
{
    timespec_get(&te, TIME_UTC);
    joy_time_curr = ((float) te.tv_nsec) * 1e-9;

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
        joy_time_curr += 1.0;
    }

        joy_dt = joy_time_curr - joy_time_prev;

    if (joy_dt > JOYSTICK_TIMEOUT_S)
    {
        printf("SAFETY STOP: Joystick timeout (%.3f s)\n\r", joy_dt);
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

void is_auton(data button){

    if(button.key2 == 1 && last_state_x != button.key2){
        auton = !auton;
        printf("Autonomy: %s\n\r", auton ? "ON" : "OFF");         
    }

    last_state_x = button.key2;
}

bool is_motors_paused(data data){
    //A = key0
    //Y = key3
    if(data.key0 == 1){
        motor_commands.fill(2);
        motors_paused = true;
        integ_pitch_err = 0.0;
        integ_roll_err = 0.0;
        printf("paused ... \n");
    }

    if(data.key3 == 1){
        motors_paused = false;
        printf("resumed ... \n");
    }
    return motors_paused;
}

void trap(int signal)
{
    printf("ending program\n\r");
    run_program = 0;
}

// ====================================================================
// joystick / shared memory setup
// ====================================================================

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
    shared_memory = (data *)shmat(segment_id, 0, 0);
    printf("shared memory attached at address %p\n", shared_memory);
    /* Determine the segment's size. */
    shmctl(segment_id, IPC_STAT, &shmbuffer);
    segment_size = shmbuffer.shm_segsz;
    printf("segment size: %d\n", segment_size);
    /* Write a string to the shared memory segment. */
    // sprintf (shared_memory, "test!!!!.");
}

// ====================================================================
// motor enable
// ====================================================================

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

// ====================================================================
// set motors (write commands)
// ====================================================================

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