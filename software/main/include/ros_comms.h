#ifndef ROS_COMMS_H
#define ROS_COMMS_H

#ifdef __cplusplus
extern "C" {
#endif

// Timer parameters

#define TIMEOUT_IN_US 500000UL

// PWM Parameters

#define PWMLIMIT  99U
#define PWM_PERIOD 100U
// #define PWM_PERIOD 50U

#define N_PWM_PINS 3


extern uint32_t duties[N_PWM_PINS];

extern float phases[N_PWM_PINS];
            
extern const gpio_num_t gpio_pins[6];

extern esp_timer_handle_t timer_handle;

void rosserial_setup();

void rosserial_spinonce();

#ifdef __cplusplus
}
#endif

#endif /* ROS_COMMMS_H*/
