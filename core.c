
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdbool.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

#include "./MPU/orientation.h"
#include "./PWM/pwm.h"
#include "./PID/pid.h"
#include "core.h"
/*
* core functions
*/
void core() {
  /*
  * pipe
  */
  int pipefd[2];
  if (pipe(pipefd) == -1) {
    perror("pipe");
    exit(EXIT_FAILURE);
  }
  int flags = fcntl(pipefd[0], F_GETFL, 0);
  if (fcntl(pipefd[0], F_SETFL, flags | O_NONBLOCK)) {
    perror("fcntl");
    close(pipefd[0]);
    exit(EXIT_FAILURE);
  }
  /*
  * fork
  */
  pid_t proces = fork();
  if (proces == -1) {
    perror("fork");
    exit(EXIT_FAILURE);
  }
  /*
  * main logic
  */
  if (!proces) {
    wiringPiSetup();
    int mpu_data = setupMPU(MPU_ADDR);
    wiringPiI2CWriteReg8(mpu_data, POWER_MGMT_1, 0x00); // disable sleep mode
    /*
    * init ac
    */
    bool old_data = true;
    bool now_data = false;
    axis_t calibration_data_accel = {649.676025, -667.919983, 0.0},
           data_ac = {0, 0, 0};
    // memset(&calibration_data_accel, 0, sizeof(calibration_data_accel));
    memset(&data_ac, 0, sizeof(data_ac));
    // read_calibr_daat_from_file(&calibration_data_accel, now_data);
    /*
    *init gyro
    */
    axis_t calibration_data_gyro = {-120.525002, 256.752014, -85.705002},
           // axis_t calibration_data_gyro = {0,0,0},
        data_gy;
    memset(&data_gy, 0, sizeof(data_gy));
    /*
    * init data rotation
    */
    float rotation_x;
    float rotation_y;
    rotation_t data_rotation_x;
    rotation_t data_rotation_y;
    memset(&data_rotation_x, 0, sizeof(data_rotation_x));
    memset(&data_rotation_y, 0, sizeof(data_rotation_y));
    /*
    * init pid
    */
    pid_param_t pid_param_x;
    pid_param_t pid_param_y;
    memset(&pid_param_x, 0, (sizeof(pid_param_x)));
    memset(&pid_param_y, 0, (sizeof(pid_param_y)));
    /*
    * init motors
    */
    motor_t fl = {MOTOR_PIN_FL, "x+", 0.0, 10, 0}; // roll
    motor_t br = {MOTOR_PIN_BR, "x-", 0.0, 10, 0};
    motor_t fr = {MOTOR_PIN_FR, "y-", 0.0, 10, 0}; // pitch
    motor_t bl = {MOTOR_PIN_BL, "y+", 0.0, 10, 0};
    motor_t pitch[2] = {bl, fr};
    motor_t roll[2] = {br, fl};
    // pwm_init(roll);
    pwm_init(pitch);
    // delay(PAUSE);
    /*
    * helper
    */
    usi power_value = 0;
    char read_buf[BUFFER_SIZE];
    close(pipefd[1]);
    int read_fd = 0;
    // calibration_v2(mpu_data);
    // calibr_data_to_file(calibration_data_gyro, calibration_data_accel);
    // exit(EXIT_SUCCESS);
    while (1) {
      if ((read_fd = read(pipefd[0], &read_buf, BUFFER_SIZE)) == -1) {
        int e = errno;
        if (e == EINTR) {
          continue;
        } else if (e == EAGAIN) {
          // puts("No data");
        } else {
          perror("Core read: ");
          _exit(EXIT_FAILURE);
        }
      }
      if (strncmp(read_buf, "e", 1) == 0) {
        roll[0].power = 10;
        roll[1].power = 10;
        pitch[0].power = 10;
        pitch[1].power = 10;
        pwm_write(roll);
        pwm_write(pitch);
        delay(PAUSE);
        puts("Core exit");
        break;
      } else if (strncmp(read_buf, "c", 1) == 0) {
        puts("***\n");
        puts("Calibration mode start.\nCallibraton gyro.\nDon't touch "
             "sensor...\n");
        calibration_gyro(mpu_data, &calibration_data_gyro);
        sleep(5);
        puts("\nCalibration accel.\nRotate the sensor...\n");
        calibration_accel(mpu_data, &calibration_data_accel);
        puts("\nCalibration completed\n***\n");
        puts("Copy data to file\n");
        calibr_data_to_file_v2(&calibration_data_gyro, &calibration_data_accel);
        read_calibr_daat_from_file(&calibration_data_accel, now_data);
        puts("***\n");
        strcpy(read_buf, "");
      } else if (strncmp(read_buf, "o", 1) == 0) {
        read_calibr_daat_from_file(&calibration_data_accel, old_data);
        strcpy(read_buf, "");
      }
      power_value = atoi(read_buf);
      write_power(roll, power_value);
      write_power(pitch, power_value);

      accel(mpu_data, calibration_data_accel, &data_ac);
      gyro(mpu_data, calibration_data_gyro, &data_gy);

      // printf("RAWA X: %f\tY: %f\tZ: %f\n", data_ac.x, data_ac.y, data_ac.z);
      // printf("RAWG X: %f\tY: %f\tZ: %f\n", data_gy.x, data_gy.y, data_gy.z);

      get_x_rotation(data_ac, &data_rotation_x);
      get_y_rotation(data_ac, &data_rotation_y);

      // printf("noFil_x: %5.1f\n", data_rotation_x.value_axis);
      // printf("noFil_y: %5.1f\n", data_rotation_y.value_axis);

      rotation_x = mpu_filter(data_gy.x, &data_rotation_x);
      rotation_y = mpu_filter(data_gy.y, &data_rotation_y);

      printf("x: %5.1f\n", rotation_x);
      printf("y: %5.1f\n", rotation_y);
      pid(roll, rotation_x, &pid_param_x);
      pid(pitch, rotation_y, &pid_param_y);

      printf("roll_pid: %5.1f\tpitch_pid: %5.1f\n", roll[0].pid, pitch[0].pid);

      calculation(pitch);
      // calculation(roll);

      for (int i = 0; i < 2; i++) {
        printf("roll %3s: target_power %3.1i, power %3.1i, pid %3.1f\n",
               roll[i].axis, roll[i].target_power, roll[i].power,
               roll[i].pid);

        printf("pitch %2s: target_power %3.1i, power %3.1i, pif %3.1f\n",
               pitch[i].axis, pitch[i].target_power, pitch[i].power,
               pitch[i].pid);
      }
      // pwm_write(roll);
      pwm_write(pitch);

      delay(50);
    }
    close(pipefd[0]);
    _exit(EXIT_SUCCESS);
  } else {
    char write_buf[BUFFER_SIZE];
    close(pipefd[0]);
    int write_fd = 0;
    while (1) {
      puts("Enter value: ");
      fgets(write_buf, BUFFER_SIZE, stdin);
      if ((write_fd = write(pipefd[1], &write_buf, BUFFER_SIZE)) > 0) {
        if (strncmp(write_buf, "e", 1) == 0) {
          break;
        }
      } else if (write_fd == -1) {
        perror("write");
        exit(EXIT_FAILURE);
      }
    }
    close(pipefd[1]);
  }

  int pid_status;
  puts("wait....");
  if (waitpid(proces, &pid_status, 0) == -1) {
    perror("waitpid");
    exit(EXIT_FAILURE);
  }
  if (WEXITSTATUS(pid_status))
    puts("Code not zero");
  puts("Console exit");
}
/*
* main
*/
int main(int argc, char const *argv[]) {
  core();
  return 0;
}
