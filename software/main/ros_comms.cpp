#include "driver/pwm.h"
#include "driver/gpio.h"
#include "ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "ros_comms.h"

static const char* TAG = "ros-comms";

// ROSserial elements
ros::NodeHandle nh;

std_msgs::String status_msg;

ros::Publisher status_pub("status", &status_msg);

// Init PWM Parameters

uint32_t duties[N_PWM_PINS] = { 0, 0, 0, 0 };

float phases[N_PWM_PINS] = { 0, 0, 0, 0 };

static const uint32_t pwm_limit_val = PWMLIMIT;

static const int pwm_lim_top = pwm_limit_val/2;
static const int pwm_lim_bot = -1*(pwm_limit_val/2 + 1);

// GPIO List

const gpio_num_t gpio_pins[5] = {
            GPIO_NUM_5,
            GPIO_NUM_16,
            GPIO_NUM_0,
            GPIO_NUM_2,
            GPIO_NUM_15
};

void pwm_update_t( const std_msgs::Int16& throttle )
{
  int th_tmp = throttle.data;
  char message[100];
  snprintf(message, sizeof(message), "Got throttle value: %d\n", th_tmp);
  status_msg.data = message;
  status_pub.publish(&status_msg);

  if ( th_tmp > pwm_lim_top )     // Clamping
  {
    ESP_LOGW(TAG, "Warning, sent PWM of %d over top limit, pwm_lim_top = %d", th_tmp, pwm_lim_top);
    th_tmp = pwm_lim_top;
  }
  if ( th_tmp < pwm_lim_bot )
  {
    ESP_LOGW(TAG, "Warning, PWM of %d below pwm_lim_bot", th_tmp);
    th_tmp = pwm_lim_bot;
  }

  if (th_tmp >= 0 )    // Direction reversal
  {
    duties[0] = th_tmp;
    duties[1] = th_tmp;
    duties[2] = 0;
    duties[3] = 0;

    gpio_set_level(gpio_pins[0],0);
    gpio_set_level(gpio_pins[1],0);
    gpio_set_level(gpio_pins[2],1);
    gpio_set_level(gpio_pins[3],1);

    ESP_LOGI(TAG, "duties0: %d, duties1: %d, duties2: %d, duties3: %d",duties[0],duties[1],duties[2],duties[3]);
  }else{
    th_tmp *= -1;
    duties[0] = 0;
    duties[1] = 0;
    duties[2] = th_tmp;
    duties[3] = th_tmp;

    gpio_set_level(gpio_pins[0],1);
    gpio_set_level(gpio_pins[1],1);
    gpio_set_level(gpio_pins[2],0);
    gpio_set_level(gpio_pins[3],0);

    ESP_LOGI(TAG, "duties0: %d, duties1: %d, duties2: %d, duties3: %d",duties[0],duties[1],duties[2],duties[3]);
  }


  ESP_LOGI(TAG, "th_tmp = %d, with PWMLIM %d, duties_0 = %d", th_tmp, pwm_limit_val, duties[0]);
  ESP_ERROR_CHECK( pwm_set_duties(duties) );
  ESP_ERROR_CHECK( pwm_start() );
}

void pwm_update_s( const std_msgs::Int16& steering )
{
  int st_tmp = steering.data;
  char message[100];
  snprintf(message, 100, "Got steering value: %d\n", st_tmp);
  status_msg.data = message;
  status_pub.publish(&status_msg);
  if ( st_tmp > pwm_lim_top )     // Clamping
  {
    ESP_LOGW(TAG, "Warning, sent PWM of %d over top limit, pwm_lim_top = %d", st_tmp, pwm_lim_top);
    st_tmp = pwm_lim_top;
  }
  if ( st_tmp < pwm_lim_bot )
  {
    ESP_LOGW(TAG, "Warning, PWM of %d below pwm_lim_bot", st_tmp);
    st_tmp = pwm_lim_bot;
  }

  if (st_tmp >= 0 )    // Run LEFT wheel to steer RIGHT
  {
    duties[0] = 0;
    duties[1] = st_tmp;
    duties[2] = 0;
    duties[3] = 0;

    gpio_set_level(gpio_pins[0],1);
    gpio_set_level(gpio_pins[1],0);
    gpio_set_level(gpio_pins[2],1);
    gpio_set_level(gpio_pins[3],1);

    ESP_LOGI(TAG, "duties0: %d, duties1: %d, duties2: %d, duties3: %d",duties[0],duties[1],duties[2],duties[3]);
  }else{              // Run RIGHT wheel to steer LEFT
    st_tmp *= -1;
    duties[0] = st_tmp;
    duties[1] = 0;
    duties[2] = 0;
    duties[3] = 0;

    gpio_set_level(gpio_pins[0],0);
    gpio_set_level(gpio_pins[1],1);
    gpio_set_level(gpio_pins[2],1);
    gpio_set_level(gpio_pins[3],1);

    ESP_LOGI(TAG, "duties0: %d, duties1: %d, duties2: %d, duties3: %d",duties[0],duties[1],duties[2],duties[3]);
  }


  ESP_LOGI(TAG, "st_tmp = %d, with PWMLIM %d, duties_0 = %d", st_tmp, pwm_limit_val, duties[0]);
  ESP_ERROR_CHECK( pwm_set_duties(duties) );
  ESP_ERROR_CHECK( pwm_start() );
}

ros::Subscriber<std_msgs::Int16> throttle_sub("throttle", &pwm_update_t);
ros::Subscriber<std_msgs::Int16> steer_sub("steering", &pwm_update_s);


void rosserial_setup()
{
  nh.initNode();
  nh.advertise(status_pub);
  nh.subscribe(throttle_sub);
  nh.subscribe(steer_sub);
}

void rosserial_spinonce()
{
  nh.spinOnce();
}

