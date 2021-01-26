#ifndef ROS_COMMS_H
#define ROS_COMMS_H

#ifdef __cplusplus
extern "C" {
#endif

// HW timer parameters

#define TIMEOUT_IN_MS 500UL
#define HW_TIMER_DIV TIMER_CLKDIV_256
#define HW_TIMER_FREQ TIMER_BASE_CLK/HW_TIMER_DIV
#define HW_TIMER_LOAD_TICKS (TIMEOUT_IN_MS*HW_TIMER_FREQ)/1000

// PWM Parameters

#define PWMLIMIT  99U
#define PWM_PERIOD 100U
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
