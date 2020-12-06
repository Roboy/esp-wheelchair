#include "driver/pwm.h"
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

uint32_t duties[3] = { 500, 500, 500 };

float phases[3] = { 0, 0, 0};

static const uint32_t pwm_limit_val = PWMLIMIT;

static const int pwm_lim_top = pwm_limit_val/2;
static const int pwm_lim_bot = -1*(pwm_limit_val/2 + 1);

void pwm_update_t( const std_msgs::Int16& throttle )
{
  int th_tmp = throttle.data;
  char message[100];
  snprintf(message, sizeof(message), "Got throttle value: %d\n", th_tmp);
  status_msg.data = message;
  status_pub.publish(&status_msg);

  if ( th_tmp > pwm_lim_top )
  {
    ESP_LOGW(TAG, "Warning, sent PWM of %d over top limit, pwm_lim_top = %d", th_tmp, pwm_lim_top);
    th_tmp = pwm_limit_val/2;
  }
  if ( th_tmp < pwm_lim_bot )
  {
    ESP_LOGW(TAG, "Warning, sent PWM of %d over bot limit, pwm_lim_bot = %d", th_tmp, pwm_lim_bot);
    th_tmp = -(pwm_limit_val/2);
  }
  duties[0] = th_tmp + 512;
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
  if ( st_tmp > pwm_lim_top )
  {
    ESP_LOGW(TAG, "Warning, sent PWM of %d over top limit, pwm_lim_top = %d", st_tmp, pwm_lim_top);
    st_tmp = pwm_limit_val/2;
  }else if ( st_tmp < pwm_lim_bot )
  {
    ESP_LOGW(TAG, "Warning, sent PWM of %d over bot limit, pwm_lim_bot = %d", st_tmp, pwm_lim_bot);
    st_tmp = -(pwm_limit_val/2);
  }
  duties[1] = st_tmp + 512;
  ESP_LOGI(TAG, "st_tmp = %d, with PWMLIM %d, duties_1 = %d", st_tmp, pwm_limit_val, duties[1]);
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

