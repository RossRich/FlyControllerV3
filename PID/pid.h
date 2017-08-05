#define DEBUG 0

#define KP .002
#define KI 0
#define KD 0
#define INTEGRATED_LIMIT 2
/*
* typs
*/
typedef struct data_pid {
  float previous_pid_timer;
  float integrated_error;
  float previous_real_angle;
} pid_param_t;
/*
* pid functions
*/
void pid(motor_t *, float, pid_param_t *);
