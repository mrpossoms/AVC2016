#ifndef AVC_CONTROLS_SERVOS
#define AVC_CONTROLS_SERVOS

#define SERVO_STEERING 0
#define SERVO_THROTTLE 1

int conInit();
int conSet(int servo, int percent);

#endif
