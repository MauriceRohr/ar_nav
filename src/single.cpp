#include "ar_nav/single.hpp"
#include <tf2/transform_datatypes.h>
#include <angles/angles.h>

namespace tf
{
  inline
  void convert(const geometry_msgs::Transform& trans, geometry_msgs::Pose& pose)
  {
      pose.orientation = trans.rotation;
      pose.position.x = trans.translation.x;
      pose.position.y = trans.translation.y;
      pose.position.z = trans.translation.z;
  }

  inline
  void convert(const geometry_msgs::Pose& pose, geometry_msgs::Transform& trans)
    {
      trans.rotation = pose.orientation;
      trans.translation.x = pose.position.x;
      trans.translation.y = pose.position.y;
      trans.translation.z = pose.position.z;
  }

  inline
  void convert(const geometry_msgs::TransformStamped& trans, geometry_msgs::PoseStamped& pose)
  {
      convert(trans.transform, pose.pose);
      pose.header = trans.header;
  }

  inline
  void convert(const geometry_msgs::PoseStamped& pose, geometry_msgs::TransformStamped& trans)
  {
      convert(pose.pose, trans.transform);
      trans.header = pose.header;
  }
}

ArNavSingle::ArNavSingle() {
	// initialize topics
	ros::NodeHandle n("~");
	std::string s;
  n.param<std::string>("marker_transform_topic", m_marker_transform_topic, "/ar_single_board/transform");
	n.param<std::string>("world_frame", m_world_frame, "world");
  n.param<std::string>("cf_frame", m_cf_frame, "crazyflie/base_link");
  n.param<std::string>("cf_goal_frame", m_cf_goal_frame, "crazyflie/goal");
  n.param<std::string>("ar_board", m_ar_board, "aruco_board");

  m_marker_pose_sub = nh.subscribe(m_marker_transform_topic, 1, &ArNavSingle::markerPoseCallback, this);
	m_cf_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("cf_pose", 1);
  debug_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("debug_pose", 1);
  


  m_ar_board_list.push_back(m_ar_board);
  // We publish some identity transforms and goal poses for the crazyflie controller
  // Since we are not using an external localization system, we get a
  // valid transformation tree only when the aruco_board is in the field of view
  // of the camera.
  tf::StampedTransform init_tf;
  init_tf.setIdentity();
  init_tf.frame_id_ = "/"+m_world_frame;
  init_tf.child_frame_id_ = "/"+m_cf_frame;
  init_tf.stamp_ = ros::Time::now();
  m_br.sendTransform(init_tf);

  geometry_msgs::TransformStamped goal_tf_msg;
  geometry_msgs::PoseStamped goal_pose_msg;
  tf::transformStampedTFToMsg(init_tf, goal_tf_msg);
  tf::convert(goal_tf_msg, goal_pose_msg);
  m_cf_pose_pub.publish(goal_pose_msg);

  // wait for active connections
  // TODO: replace with subscriber callback
  ros::Rate rate(10);
	while (m_cf_pose_pub.getNumSubscribers() < 1)
  {
		rate.sleep();
    m_br.sendTransform(init_tf);
    m_cf_pose_pub.publish(goal_pose_msg);
  }
}

void ArNavSingle::markerPoseCallback(const geometry_msgs::TransformStamped &tf_msg) {
	try {

    // if requested waypoint is found, target it
    bool requested_board_detected = false;

    for(std::string ar_board: m_ar_board_list){
      if(("/"+ar_board) == tf_msg.child_frame_id){
        requested_board_detected = true;
        ROS_DEBUG("Requested_board_detected %s", ar_board.c_str());
        ROS_DEBUG("Board Parent frame %s", tf_msg.header.frame_id.c_str());
        ROS_DEBUG("Board Child frame %s", tf_msg.child_frame_id.c_str());
      }
    }
    if(requested_board_detected){
      //Then place the goal to hover on top of the board
      // publish camera transform

      tf::StampedTransform world_to_board_tf;
      tf::StampedTransform board_to_cam_tf;
      tf::StampedTransform cam_to_cfbaselink_tf;
      tf::StampedTransform cam_to_board_tf;
      tf::StampedTransform world_to_cfbaselink_tf;

      tf::StampedTransform world_to_goal_tf;

      try{
        m_tf_listener.lookupTransform("/"+m_world_frame, "/"+m_ar_board, ros::Time(0), world_to_board_tf);
        m_tf_listener.lookupTransform("/cam", "/"+m_cf_frame, ros::Time(0), cam_to_cfbaselink_tf);
      }
      catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }

      tf::transformStampedMsgToTF(tf_msg, cam_to_board_tf);
      board_to_cam_tf.setData(cam_to_board_tf.inverse());



      world_to_cfbaselink_tf.setData(world_to_board_tf*board_to_cam_tf*cam_to_cfbaselink_tf);
      world_to_cfbaselink_tf.child_frame_id_ = "/"+m_cf_frame; // crazyflie/base_link
      world_to_cfbaselink_tf.frame_id_ = "/"+m_world_frame;  // world
      world_to_cfbaselink_tf.stamp_ = ros::Time::now();
      m_br.sendTransform(world_to_cfbaselink_tf);


      // The Goal follows ROS conventions (Z axis up, X to the righ and Y to the forwar direction of movement)
      // We Position the Goal above the world coordinate frame (our marker)
      tf::Vector3 goal_position(0,0,0.7);
      world_to_goal_tf.setIdentity();
      world_to_goal_tf.setOrigin(goal_position);
      world_to_goal_tf.child_frame_id_ = "/"+m_cf_goal_frame; // crazyflie/goal
      world_to_goal_tf.frame_id_ = "/"+m_world_frame; // world
      world_to_goal_tf.stamp_ = ros::Time::now();

      m_br.sendTransform(world_to_goal_tf);

      geometry_msgs::TransformStamped goal_tf_msg;
      geometry_msgs::PoseStamped goal_pose_msg;
      tf::transformStampedTFToMsg(world_to_goal_tf, goal_tf_msg);
      tf::convert(goal_tf_msg, goal_pose_msg);


      ROS_DEBUG("Publishing transform and goal pose:%f, transform:%f", goal_pose_msg.pose.position.z, goal_tf_msg.transform.translation.z);
      m_cf_pose_pub.publish(goal_pose_msg);
    }

  }
	catch (...) {
		ROS_ERROR("Failed to publish crazyflie pose");
	}
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "single");
  ArNavSingle node;
	ros::spin();
	return 0;
}
