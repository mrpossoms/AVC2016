#ifndef AVC_CONTROLS_SERVOS
#define AVC_CONTROLS_SERVOS

#define SERVO_STEERING 0
#define SERVO_THROTTLE 1

int ctrlInit();
int ctrlSet(int servo, int percent);

#endif
