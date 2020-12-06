#include "driver/pwm.h"
#include "ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "ros_comms.h"

// ROSserial elements
ros::NodeHandle nh;

std_msgs::String status_msg;

ros::Publisher status_pub("status", &status_msg);

// Init PWM Parameters

uint32_t duties[2] = { 500, 500 };

float phases[2] = { 0, 0 };

void pwm_update_t( const std_msgs::Int16& throttle )
{
  int th_tmp = throttle.data;
  char message[100];
  snprintf(message, sizeof(message), "Got throttle value: %d\n", th_tmp);
  status_msg.data = message;
  status_pub.publish(&status_msg);
  if ( th_tmp > PWMLIMIT/2 )
  {
    th_tmp = PWMLIMIT/2;
  }else if ( th_tmp < -(PWMLIMIT/2 + 1) )
  {
    th_tmp = -(PWMLIMIT/2);
  }
  duties[0] = th_tmp + 512;
  ESP_ERROR_CHECK( pwm_set_duties(duties) );
}

void pwm_update_s( const std_msgs::Int16& steering )
{
  int st_tmp = steering.data;
  char message[100];
  snprintf(message, 100, "Got steering value: %d\n", st_tmp);
  status_msg.data = message;
  status_pub.publish(&status_msg);
  if ( st_tmp > PWMLIMIT/2 )
  {
    st_tmp = PWMLIMIT/2;
  }else if ( st_tmp < -(PWMLIMIT/2 + 1) )
  {
    st_tmp = -(PWMLIMIT/2);
  }
  duties[1] = st_tmp + 512;
  ESP_ERROR_CHECK( pwm_set_duties(duties) );
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

