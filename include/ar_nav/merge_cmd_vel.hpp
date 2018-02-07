#ifndef MERGE_CMD_VEL_CPP
#define MERGE_CMD_VEL_CPP

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

class MergeCmdVel
{
public:
  MergeCmdVel();
  ros::NodeHandle nh;

  void cf_controller_cb(const geometry_msgs::Twist& cmd_vel);
  void joy_teleop_cb(const geometry_msgs::Twist& cmd_vel);
  void merge_cmd_vel(void);

private:
  ros::Subscriber controller_sub_;
  ros::Subscriber joy_teleop_sub_;

  ros::Publisher cmd_vel_pub_;
  ros::Publisher cmd_vel_stamped_pub_;

  geometry_msgs::Twist cmd_vel_controller_;
  geometry_msgs::Twist cmd_vel_teleop_;
};


#endif // MERGE_CMD_VEL_CPP
