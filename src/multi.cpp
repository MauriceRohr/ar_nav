#include "ar_nav/multi.hpp"

ArNavMulti::ArNavMulti() {
	// initialize topics
	ros::NodeHandle n("~");
	std::string s;
	n.param<std::string>("marker_pose_topic", s, "/marker_pose");
	n.param<std::string>("world_frame", m_world_frame, "world");
  n.param<std::string>("cf_frame", m_cf_frame, "crazyflie/base_link");
  n.param<std::string>("ar_boards", m_ar_boards, "board_c3po");

  m_marker_pose_sub = nh.subscribe(s, 1, &ArNavMulti::markerPoseCallback, this);
	m_cf_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("cf_pose", 1);

	debug_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("debug_pose", 1); // DEBUG, containing current markers pose (without stepping)
  
  std::stringstream ssb(m_ar_boards);
  std::string tok;
  while (getline(ssb, tok, '|')) {
    m_ar_boards_list.push_back("/"+tok);
  }

	// wait for active connections
	// TODO: replace with subscriber callback
	ros::Rate rate(10);
	while (m_cf_pose_pub.getNumSubscribers() < 1)
		rate.sleep();
}

void ArNavMulti::markerPoseCallback(const geometry_msgs::TransformStamped &tf_msg) {
	try {

    // if requested waypoint is found, target it
    bool requested_board_detected = false;

    for(std::string ar_board: m_ar_boards_list){
      if(ar_board == tf_msg.child_frame_id){
        requested_board_detected = true;
        ROS_INFO("Requested_board_detected %s", ar_board.c_str());
        ROS_INFO("Board Parent frame %s", tf_msg.header.frame_id.c_str());
        ROS_INFO("Board Child frame %s", tf_msg.child_frame_id.c_str());
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
        m_tf_listener.lookupTransform("/world", "/board_c3po",
                                 ros::Time(0), world_to_board_tf);
        m_tf_listener.lookupTransform("/cam", "/crazyflie/base_link",
                                 ros::Time(0), cam_to_cfbaselink_tf);
      }
      catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }

      tf::transformStampedMsgToTF(tf_msg, cam_to_board_tf);
      board_to_cam_tf.setData(cam_to_board_tf.inverse());

      world_to_cfbaselink_tf.setData(world_to_board_tf*board_to_cam_tf*cam_to_cfbaselink_tf);
      world_to_cfbaselink_tf.child_frame_id_ = "/crazyflie/base_link";
      world_to_cfbaselink_tf.frame_id_ = "/world";
      world_to_cfbaselink_tf.stamp_ = ros::Time::now();
      m_br.sendTransform(world_to_cfbaselink_tf);

      tf::Vector3 goal_position(0,0,1.0);
      world_to_goal_tf.setIdentity();
      world_to_goal_tf.setOrigin(goal_position);
      world_to_goal_tf.child_frame_id_ = "/crazyflie/goal";
      world_to_goal_tf.frame_id_ = "/world";
      world_to_goal_tf.stamp_ = ros::Time::now();
      m_br.sendTransform(world_to_goal_tf);

    }



		// wait for right TransformStamped
//		if (!bt.child_frame_id.std::string::compare("/" + m_waypoint_list[m_current_waypoint_id])) {
//			setCfPose(bt);
//			debug_pose_pub.publish(m_cf_pose);
//			float distance = sqrt(pow(bt.transform.translation.x, 2) + pow(bt.transform.translation.y, 2));
//			if (m_step_active) {
//				ROS_WARN_STREAM("Stepping to target: " << m_waypoint_list[m_current_waypoint_id]);
//				// linear target change
//				float step_size = 0.05;
//				float step_range = 0.15;
//				geometry_msgs::TransformStamped step;
//				step.header = bt.header;
//				step.child_frame_id = bt.child_frame_id;
//				step.transform.translation.x = bt.transform.translation.x * step_size / distance;
//				step.transform.translation.y = bt.transform.translation.y * step_size / distance;
//				step.transform.translation.z = bt.transform.translation.z;
//				step.transform.rotation = bt.transform.rotation;
//				setCfPose(step);
//				ROS_INFO_STREAM(step.transform.translation << "\n" << bt.transform.translation);
//				if ((bt.transform.translation.x < step_range && bt.transform.translation.x > -step_range) && (bt.transform.translation.y < step_range && bt.transform.translation.y > -step_range)) {
//					m_step_active = false;
//				}
//			} else {
//				setCfPose(bt);
//			}

//			// broadcast pose and transformation
//      ROS_INFO("Publishing transform");
//			m_cf_pose_pub.publish(m_cf_pose);
//			sendCfPose();

//			// if CF stays in range of marker, next one is targeted
//			if (!m_waypoint_change.std::string::compare("auto")) {
//				float timeout_range = 0.15;
//				if ((distance < timeout_range && distance > -timeout_range) && m_next_waypoint_timeout != ros::Time(0.0) && !m_request_active) {
//					ros::Duration timeout(4.0);
//					if (ros::Time::now() - m_next_waypoint_timeout > timeout) {
//						requestWaypoint(1);
//					}
//				}
//				else
//					m_next_waypoint_timeout = ros::Time::now();
//			}
//		}
  }
	catch (...) {
		ROS_ERROR("Failed to publish crazyflie pose");
	}
}

void ArNavMulti::sendCfPose() {
	try {
		m_transform.setOrigin(tf::Vector3(m_cf_pose.pose.position.x, m_cf_pose.pose.position.y, m_cf_pose.pose.position.z));
		m_transform.setRotation(tf::Quaternion(m_cf_pose.pose.orientation.x, m_cf_pose.pose.orientation.y, m_cf_pose.pose.orientation.z, m_cf_pose.pose.orientation.w));
		m_br.sendTransform(tf::StampedTransform(m_transform, ros::Time::now(), m_world_frame, m_cf_frame));
	}
	catch (...) {
		ROS_ERROR("Failed to update frame relation");
	}
}

void ArNavMulti::setCfPose(const geometry_msgs::TransformStamped &bt) {
	// transform TransformStamped to PoseStamped
	m_cf_pose.header.seq = bt.header.seq;
	m_cf_pose.header.stamp = bt.header.stamp;
	m_cf_pose.header.frame_id = bt.header.frame_id;
	m_cf_pose.pose.position.x = bt.transform.translation.y;
	m_cf_pose.pose.position.y = bt.transform.translation.x;
	m_cf_pose.pose.position.z = bt.transform.translation.z;
	m_cf_pose.pose.orientation.x = bt.transform.rotation.y;
	m_cf_pose.pose.orientation.y = bt.transform.rotation.x;
	m_cf_pose.pose.orientation.z = bt.transform.rotation.z;
	m_cf_pose.pose.orientation.w = bt.transform.rotation.w;

	tfScalar roll, pitch, yaw;
    	tf::Matrix3x3(tf::Quaternion(m_cf_pose.pose.orientation.x, m_cf_pose.pose.orientation.y, m_cf_pose.pose.orientation.z, m_cf_pose.pose.orientation.w)).getRPY(roll, pitch, yaw);
}

void ArNavMulti::initializeCfPose() {
	try {
		m_cf_pose.header.seq = 0;
		m_cf_pose.header.stamp = ros::Time::now();
		m_cf_pose.header.frame_id = m_cf_frame;
	      	m_cf_pose.pose.position.x = 0.0f;
	      	m_cf_pose.pose.position.y = 0.0f;
	      	m_cf_pose.pose.position.z = 0.0f;
	      	m_cf_pose.pose.orientation.x = 0.0f;
	      	m_cf_pose.pose.orientation.y = 0.0f;
	      	m_cf_pose.pose.orientation.z = 0.0f;
	      	m_cf_pose.pose.orientation.w = 1.0f;
		m_cf_pose_pub.publish(m_cf_pose);
		sendCfPose();
	}
	catch (...) {
		ROS_WARN("Could not initialize crazyflie frame");
	}
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "multi");
	ArNavMulti node;
  //node.initializeCfPose();
	ros::spin();
	return 0;
}
