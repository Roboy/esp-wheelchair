#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/Twist.h>

double speed;
double angular;
double userspeed;
double emergencyStopThreshold = 0.1; 


void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  // Convert the PointCloud2 message to a PCL point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  // Process the point cloud here 
  int num_points = cloud->points.size();
  float minDist = 99999;
  for(int i = 0; i< cloud->points.size();i++){
    float z = cloud->points[0].z;
    if(minDist > z){
      minDist = z;
    }
    
  }
  if(minDist > 1){
    minDist = 1;
  }
  if(minDist < 0){
    minDist = 0;
  }

  // later it going to use a different ToF sensor for going backward
  // if(userspeed < 0){
  //   speed = -1 * minDist;
  // }else if (userspeed > 0){
  //   speed = minDist;
  // }else{
  //   speed = 0;
  // }

  ROS_INFO_STREAM("minDist:"<<minDist);
  
  if (minDist < emergencyStopThreshold){
    speed = 0;
  }

  
}

void userInputCallback(const geometry_msgs::Twist& msg)
{
  userspeed = msg.linear.x;
  angular = msg.angular.z;
  // if (msg.angular.z > 0){
  //   ROS_INFO_STREAM("Subscriber velocities:"<<" linear="<<msg.linear.x<<" angular="<<msg.angular.z);
  // }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wheelchair_potentialField_speedController");
  ros::NodeHandle n;
  // publishes to potentialfield speed & angular
  ros::Publisher potential_Field_speed_pub = n.advertise<std_msgs::Float64>("/roboy/pinky/middleware/espchair/wheels/speed", 10);
  ros::Publisher potential_Field_angular_pub = n.advertise<std_msgs::Float64>("/roboy/pinky/middleware/espchair/wheels/angular", 10);

  speed = 0.0;
  angular = 0.0;
  
  // Subscribe to the point cloud topic
  ros::Subscriber user_input_sub = n.subscribe("/cmd_vel", 100, userInputCallback);

  // subcribe to the point cloud topic
  ros::Subscriber pointcloud_sub = n.subscribe("/royale_camera_driver/point_cloud", 1, pointCloudCallback);


  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ROS_INFO_STREAM(speed);
    std_msgs::Float64 speed_msg;
    speed_msg.data = speed;
    potential_Field_speed_pub.publish(speed_msg);

    std_msgs::Float64 angular_msg;
    angular_msg.data = angular;
    potential_Field_angular_pub.publish(angular_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}




