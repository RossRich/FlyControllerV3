#define MOTOR_PIN_FL 1
#define MOTOR_PIN_FR 0
#define MOTOR_PIN_BL 2
#define MOTOR_PIN_BR 3
#define MAX_POWER 21
#define MIN_POWER 10
/*
* types
*/
typedef unsigned short int usi;
typedef struct data_motor {
  usi motor_pin;
  const char *axis;
  usi target_power; // user set power
  float target_angle;
  usi power; // calculation power
  float pid;
} motor_t;
/*
* functions
*/
void pwm_init(motor_t *);
void calculation(motor_t *);
usi limit_power(usi);
void pwm_write(motor_t *);
void write_power(motor_t *, usi);
