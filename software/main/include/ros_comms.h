#ifndef ROS_COMMS_H
#define ROS_COMMS_H

#ifdef __cplusplus
extern "C" {
#endif

// PWM Parameters

#define PWMLIMIT  1023U
#define PWM_PERIOD 200U

extern uint32_t duties[2];

extern float phases[2];
            
void rosserial_setup();

void rosserial_spinonce();

#ifdef __cplusplus
}
#endif

#endif /* ROS_COMMMS_H*/
