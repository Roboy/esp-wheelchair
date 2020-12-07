#ifndef ROS_COMMS_H
#define ROS_COMMS_H

#ifdef __cplusplus
extern "C" {
#endif

// PWM Parameters

#define PWMLIMIT  1023U
#define PWM_PERIOD 1000U
#define N_PWM_PINS 4

extern uint32_t duties[N_PWM_PINS];

extern float phases[N_PWM_PINS];
            
extern const gpio_num_t gpio_pins[5];

void rosserial_setup();

void rosserial_spinonce();

#ifdef __cplusplus
}
#endif

#endif /* ROS_COMMMS_H*/
