#ifndef AR_NAV_MULTI_HPP
#define AR_NAV_MULTI_HPP

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

class ArNavMulti {

public:
	ArNavMulti();
	ros::NodeHandle nh;

private:
	// Functions
	void markerPoseCallback(const geometry_msgs::TransformStamped &msg);

	// Subscribers
	ros::Subscriber m_marker_pose_sub;

	// Publisher
	ros::Publisher m_cf_pose_pub;
	ros::Publisher debug_pose_pub;

	// Variables
  tf::TransformListener m_tf_listener;
	tf::Transform m_transform;
	tf::TransformBroadcaster m_br;
	geometry_msgs::PoseStamped m_cf_pose;
	std::string m_cf_frame;
	std::string m_world_frame;

  std::vector<std::string> m_ar_boards_list;
  std::string m_ar_boards;
};

#endif	// AR_NAV_MULTI_HPP
