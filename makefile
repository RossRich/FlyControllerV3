.PHONY: all clean
CC = gcc -std=c99
ORIENTATION_PATH = MPU
PID_PATH = PID
PWM_PATH = PWM
LIB = -lwiringPi -lpthread -lm

all: core

pwm.o: $(PWM_PATH)/pwm.c $(PWM_PATH)/pwm.h
	$(CC) -c $(PWM_PATH)/pwm.c

pid.o: $(PID_PATH)/pid.c $(PID_PATH)/pid.h $(PWM_PATH)/pwm.h
	$(CC) -c $(PID_PATH)/pid.c

orientation.o: $(ORIENTATION_PATH)/orientation.c $(ORIENTATION_PATH)/orientation.h
	$(CC) -c $(ORIENTATION_PATH)/orientation.c

core.o: core.c core.h $(ORIENTATION_PATH)/orientation.h $(PID_PATH)/pid.h $(PWM_PATH)/pwm.h
	$(CC) -c core.c

core: core.o $(ORIENTATION_PATH)/orientation.o $(PID_PATH)/pid.o $(PWM_PATH)/pwm.o
	sudo $(CC) core.o $(ORIENTATION_PATH)/orientation.o $(PID_PATH)/pid.o $(PWM_PATH)/pwm.o -o run $(LIB)

clean:
	rm -rf *.o $(ORIENTATION_PATH)/*.o $(PID_PATH)/*.o $(PWM_PATH)/*.o run
