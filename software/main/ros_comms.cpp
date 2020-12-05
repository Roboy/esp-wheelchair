#include "ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "ros_comms.h"

// ROSserial elements
ros::NodeHandle nh;

std_msgs::String status_msg;
std_msgs::Int16  throttle;
std_msgs::Int16  steer;

ros::Publisher status_pub("status", &status_msg);
ros::Subscriber<std_msgs::Int16> throttle_sub("throttle", 10, &pwm_update_t);
ros::Subscriber<std_msgs::Int16> steer_sub("steering", 10, &pwm_update_s);


void rosserial_setup()
{
  nh.initNode();
  nh.advertise(status_pub);
  nh.subscribe(throttle_sub);
  nh.subscribe(steer_sub);
}

void rosserial_spinonce();
{
  nh.spinOnce();
}
