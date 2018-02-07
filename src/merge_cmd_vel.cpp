#include "ar_nav/merge_cmd_vel.hpp"

MergeCmdVel::MergeCmdVel()
{
  ros::NodeHandle params("~");
  std::string cmd_vel_controller_topic;
  std::string cmd_vel_teleop_topic;
  std::string cmd_vel_out_topic;

  params.param<std::string>("cmd_vel_controller_topic", cmd_vel_controller_topic, "cmd_vel_controller");
  params.param<std::string>("cmd_vel_teleop_topic", cmd_vel_teleop_topic, "cmd_vel_teleop");
  params.param<std::string>("cmd_vel_out", cmd_vel_out_topic, "cmd_vel");

  controller_sub_ = nh.subscribe (cmd_vel_controller_topic, 1, &MergeCmdVel::cf_controller_cb, this);
  joy_teleop_sub_ = nh.subscribe (cmd_vel_teleop_topic, 1, &MergeCmdVel::joy_teleop_cb, this);
  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>(cmd_vel_out_topic,1);
}

void MergeCmdVel::merge_cmd_vel(void){
  geometry_msgs::Twist cmd_vel_out;
  cmd_vel_out.linear.x = cmd_vel_controller_.linear.x + cmd_vel_teleop_.linear.x;
  cmd_vel_out.linear.y = cmd_vel_controller_.linear.y + cmd_vel_teleop_.linear.y;
  cmd_vel_out.linear.z = cmd_vel_controller_.linear.z + cmd_vel_teleop_.linear.z;
  cmd_vel_out.angular.x = cmd_vel_controller_.angular.x + cmd_vel_teleop_.angular.x;
  cmd_vel_out.angular.y = cmd_vel_controller_.angular.y + cmd_vel_teleop_.angular.y;
  cmd_vel_out.angular.z = cmd_vel_controller_.angular.z + cmd_vel_teleop_.angular.z;

  cmd_vel_pub_.publish(cmd_vel_out);
}

void MergeCmdVel::cf_controller_cb(const geometry_msgs::Twist& cmd_vel){
  cmd_vel_controller_ = cmd_vel;
  merge_cmd_vel();
}

void MergeCmdVel::joy_teleop_cb(const geometry_msgs::Twist& cmd_vel){
  cmd_vel_teleop_ = cmd_vel;
  merge_cmd_vel();
}


int main(int argc,char* argv[])
{
  ros::init(argc, argv, "merge_cmd_vel"); // Name of the node
  MergeCmdVel Node;
  ros::spin();
}
