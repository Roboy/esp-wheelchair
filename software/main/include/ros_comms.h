#ifndef ROS_COMMS_H
#define ROS_COMMS_H

#ifdef __cplusplus
extern "C" {
#endif

// PWM Parameters

#define PWMLIMIT  1023U
#define PWM_PERIOD 200U

uint32_t duties[2] = { 500, 500 };

float phases[2] = { 0, 0 };
            
void rosserial_setup();

void rosserial_spinonce();

#ifdef __cplusplus
}
#endif

#endif /* ROS_COMMMS_H*/
