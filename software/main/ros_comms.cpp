#include "driver/pwm.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Empty.h"
#include "ros_comms.h"

static const char* TAG = "ros-comms";

// ROSserial elements
ros::NodeHandle nh;

std_msgs::String status_msg;

ros::Publisher status_pub("/roboy/pinky/middleware/espchair/status", &status_msg);

// Init PWM Parameters

uint32_t duties[N_PWM_PINS] = { 0, 0, 0, 0 };

float phases[N_PWM_PINS] = { 0, 0, 0, 0 };

static const uint32_t pwm_limit_val = PWMLIMIT;

static const int pwm_lim_top = pwm_limit_val;
static const int pwm_lim_bot = -1*(pwm_limit_val);

static bool emergency_stop_active = false;

esp_timer_handle_t timer_handle;

// GPIO List

const gpio_num_t gpio_pins[5] = {
            GPIO_NUM_5,
            GPIO_NUM_16,
            GPIO_NUM_0,
            GPIO_NUM_2,
            GPIO_NUM_15
};

void pwm_update_R( const std_msgs::Int16& drive_R )
{
  int pwm_tmp = drive_R.data;
  int msg_len = 100;
  char message[msg_len];
  snprintf(message, msg_len, "Got motor right value: %d\n", pwm_tmp);
  status_msg.data = message;
  status_pub.publish(&status_msg);

  if ( pwm_tmp > pwm_lim_top )     // Clamping
  {
    // ESP_LOGW(TAG, "Warning, sent PWM of %d over top limit, pwm_lim_top = %d", pwm_tmp, pwm_lim_top);
    pwm_tmp = pwm_lim_top;
    snprintf(message, msg_len, "Warning, sent PWM of %d over top limit, pwm_lim_top = %d", pwm_tmp, pwm_lim_top);
    status_msg.data = message;
    status_pub.publish(&status_msg);
  }
  if ( pwm_tmp < pwm_lim_bot )
  {
    // ESP_LOGW(TAG, "Warning, PWM of %d below pwm_lim_bot", pwm_tmp);
    pwm_tmp = pwm_lim_bot;
    snprintf(message, msg_len, "Warning, PWM of %d below pwm_lim_bot", pwm_tmp);
    status_msg.data = message;
    status_pub.publish(&status_msg);
  }

  if (pwm_tmp >= 0 )    // Direction reversal
  {
    duties[0] = pwm_tmp;
    duties[2] = 0;

    gpio_set_level(gpio_pins[0],0);
    gpio_set_level(gpio_pins[2],1);

  }else{
    pwm_tmp *= -1;
    duties[0] = 0;
    duties[2] = pwm_tmp;

    gpio_set_level(gpio_pins[0],1);
    gpio_set_level(gpio_pins[2],0);

  }

  ESP_ERROR_CHECK( esp_timer_stop(timer_handle) ); //Feed the timer
  ESP_ERROR_CHECK( esp_timer_start_once(timer_handle, TIMEOUT_IN_US) );

  // ESP_LOGI(TAG, "duties0: %d, duties1: %d, duties2: %d, duties3: %d",duties[0],duties[1],duties[2],duties[3]);
  // ESP_LOGI(TAG, "pwm_tmp = %d, with PWMLIM %d, duties_0 = %d", pwm_tmp, pwm_limit_val, duties[0]);

  if ( !emergency_stop_active )
  {
    ESP_ERROR_CHECK( pwm_set_duties(duties) );
    ESP_ERROR_CHECK( pwm_start() );
  } else {
    for ( int i = 0; i < 4; i++){       //Ensure pwm=0
      duties[i] = 0;                    //Set all PWM to 0
      gpio_set_level(gpio_pins[i],1);   //Disable all low side switches
    }
  }

}

void pwm_update_L( const std_msgs::Int16& drive_L )
{
  int pwm_tmp = drive_L.data;
  int msg_len = 100;
  char message[msg_len];
  snprintf(message, msg_len, "Got motor left value: %d\n", pwm_tmp);
  status_msg.data = message;
  status_pub.publish(&status_msg);
  if ( pwm_tmp > pwm_lim_top )     // Clamping
  {
    // ESP_LOGW(TAG, "Warning, sent PWM of %d over top limit, pwm_lim_top = %d", pwm_tmp, pwm_lim_top);
    pwm_tmp = pwm_lim_top;
    snprintf(message, msg_len, "Warning, sent PWM of %d over top limit, pwm_lim_top = %d", pwm_tmp, pwm_lim_top);
    status_msg.data = message;
    status_pub.publish(&status_msg);
  }
  if ( pwm_tmp < pwm_lim_bot )
  {
    // ESP_LOGW(TAG, "Warning, PWM of %d below pwm_lim_bot", pwm_tmp);
    pwm_tmp = pwm_lim_bot;
    snprintf(message, msg_len, "Warning, PWM of %d below pwm_lim_bot", pwm_tmp);
    status_msg.data = message;
    status_pub.publish(&status_msg);
  }

  if (pwm_tmp >= 0 )    // Direction reversal
  {
    duties[1] = pwm_tmp;
    duties[3] = 0;

    gpio_set_level(gpio_pins[1],0);
    gpio_set_level(gpio_pins[3],1);

  }else{              
    pwm_tmp *= -1;
    duties[1] = 0;
    duties[3] = pwm_tmp;

    gpio_set_level(gpio_pins[1],1);
    gpio_set_level(gpio_pins[3],0);

  }

  ESP_ERROR_CHECK( esp_timer_stop(timer_handle) ); //Feed the timer
  ESP_ERROR_CHECK( esp_timer_start_once(timer_handle, TIMEOUT_IN_US) );

  // ESP_LOGI(TAG, "duties0: %d, duties1: %d, duties2: %d, duties3: %d",duties[0],duties[1],duties[2],duties[3]);
  // ESP_LOGI(TAG, "pwm_tmp = %d, with PWMLIM %d, duties_0 = %d", pwm_tmp, pwm_limit_val, duties[0]);

  if ( !emergency_stop_active )
  {
    ESP_ERROR_CHECK( pwm_set_duties(duties) );
    ESP_ERROR_CHECK( pwm_start() );
  } else {
    for ( int i = 0; i < 4; i++){       //Ensure pwm=0
      duties[i] = 0;                    //Set all PWM to 0
      gpio_set_level(gpio_pins[i],1);   //Disable all low side switches
    }
  }
}

void e_stop( const std_msgs::Empty& e_stop_flag )
{
  int msg_len = 100;
  char message[msg_len];
  gpio_set_level(GPIO_NUM_15,0);      //Disconnect main relay

  emergency_stop_active = true;

  for ( int i = 0; i < 4; i++){   
    duties[i] = 0;                    //Set all PWM to 0
    gpio_set_level(gpio_pins[i],1);   //Disable all low side switches
  }

  ESP_ERROR_CHECK( pwm_set_duties(duties) );
  ESP_ERROR_CHECK( pwm_start() );

  // ESP_LOGW(TAG, "GOT EMERGENCY STOP SIGNAL, DISABLING MOTORS!");
  snprintf(message, msg_len, "GOT EMERGENCY STOP SIGNAL, DISABLING MOTORS!");
  status_msg.data = message;
  status_pub.publish(&status_msg);
}

void e_recover( const std_msgs::Empty& msg )
{
  int msg_len = 100;
  char message[msg_len];

  emergency_stop_active = false;

  for ( int i = 0; i < 4; i++){       //Ensure pwm=0 at start
    duties[i] = 0;                    //Set all PWM to 0
    gpio_set_level(gpio_pins[i],1);   //Disable all low side switches
  }

  gpio_set_level(GPIO_NUM_15,1);      //Enable main relay
  
  ESP_ERROR_CHECK( pwm_set_duties(duties) );
  ESP_ERROR_CHECK( pwm_start() );
  
  snprintf(message, msg_len, "GOT EMERGENCY RECOVER SIGNAL, ENABLING MOTORS");
  status_msg.data = message;
  status_pub.publish(&status_msg);
}

ros::Subscriber<std_msgs::Int16> w_left_sub("/roboy/pinky/middleware/espchair/wheels/left", &pwm_update_L);
ros::Subscriber<std_msgs::Int16> w_right_sub("/roboy/pinky/middleware/espchair/wheels/right", &pwm_update_R);
ros::Subscriber<std_msgs::Empty> emergency_stop_sub("/roboy/pinky/middleware/espchair/emergency/stop", &e_stop);
ros::Subscriber<std_msgs::Empty> emergency_recover_sub("/roboy/pinky/middleware/espchair/emergency/recover", &e_recover);


void timer_callback(void *arg)
{
  ESP_LOGI(TAG, "Timer expired!");

  for ( int i = 0; i < 4; i++){       //Stop all motors
    duties[i] = 0;                    //Set all PWM to 0
    gpio_set_level(gpio_pins[i],1);   //Disable all low side switches
  }

  ESP_ERROR_CHECK( pwm_set_duties(duties) );
  ESP_ERROR_CHECK( pwm_start() );
}

void rosserial_setup()
{
  const esp_timer_create_args_t timer_cfg = {
    .callback = &timer_callback,
    .name = "wdt_pwm"
  };

  nh.initNode();
  nh.advertise(status_pub);
  nh.subscribe(w_right_sub);
  nh.subscribe(w_left_sub);
  nh.subscribe(emergency_stop_sub);
  nh.subscribe(emergency_recover_sub);

  ESP_ERROR_CHECK( esp_timer_init() );

  ESP_ERROR_CHECK( esp_timer_create(&timer_cfg, &timer_handle) );


}

void rosserial_spinonce()
{
  static int i = 0;

  if ( i >= 10 )
  {
    i = 0;
  }

  i++;

  nh.spinOnce();
}

