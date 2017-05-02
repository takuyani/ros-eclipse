/**
 * main
 *
 * define ROS node "ball_tracker".
 *
 * @file		main.cpp
 * @author		Takuya Niibori
 * @attention	none
 */

//C++ Standard Library
#include <cstdint>
//C Standard Library
//Add Install Library
#include <ros/ros.h>
//My Library
#include "ball_tracker.hpp"

const std::string NODE_NAME = "ball_tracker";

/**
 * @brief	main function
 */
int main(int argc, char **argv) {

//	try {
		ros::init(argc, argv, NODE_NAME);
//	} catch (ros::InvalidNodeNameException& e) {
//		ROS_ERROR("TeleopSourceNode::init: error initialising node");
//		return false;
//	}


	ros::NodeHandle nh("~");

	BallTracker bt(nh);

	ROS_INFO_STREAM("Running Node:[" << NODE_NAME << "]");

	ros::spin();

	return (1);
}

