/*
* MPU registers
*/
#define DEBUG 0
#define ADDRESS_GYRO_X_OUT 0x43
#define ADDRESS_GYRO_Y_OUT 0x45
#define ADDRESS_GYRO_Z_OUT 0x47
#define ADDRESS_ACCEL_X_OUT 0x3b
#define ADDRESS_ACCEL_Y_OUT 0x3d
#define ADDRESS_ACCEL_Z_OUT 0x3f
#define ADDRESS_TEMP 0x41
#define POWER_MGMT_1 0x6b
#define MPU_ADDR 0x68
#define K 0.88
#ifndef M_PI
#define M_PI 3.14159265
#endif
#define RAD_TO_DEG 57.2957786
#define _XOPEN_SOURCE 500
#define _GNU_SOURCE
#define NOW "now_data"
#define PREVIOUS "previous_data"
/*
* typs
*/
typedef struct data_axis {
  float x;
  float y;
  float z;
} axis_t;
typedef struct data_rotation {
  float previousFilterTimer;
  float previousDataRotation;
  float value_axis;
} rotation_t;
/*
* MPU functions
*/
int setupMPU(short int);
void accel(int, axis_t, axis_t *);
void gyro(int, axis_t, axis_t *);
// void temprech(int, short int);
float dist(float, float);
void get_x_rotation(axis_t, rotation_t *);
void get_y_rotation(axis_t, rotation_t *);
void calibration_gyro(int, axis_t *);
void calibration_accel(int, axis_t *);
float readWod2c(int, short int);
float mpu_filter(float, rotation_t *);
void calibr_data_to_file(axis_t, axis_t);
void calibr_data_to_file_v2(axis_t *, axis_t *);
void read_calibr_daat_from_file(axis_t *, bool);
void calibration_v2(int);
