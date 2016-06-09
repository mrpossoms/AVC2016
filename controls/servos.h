#ifndef AVC_CONTROLS_SERVOS
#define AVC_CONTROLS_SERVOS

#ifdef __cplusplus
extern "C" {
#endif

#define SERVO_STEERING 0
#define SERVO_THROTTLE 1

int ctrlInit();
int ctrlSet(int servo, int percent);
int ctrlGet(int servo);

#ifdef __cplusplus
}
#endif

#endif
