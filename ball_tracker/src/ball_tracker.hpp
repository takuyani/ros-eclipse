/**
 * @brief		Ball Tracker
 *
 * @file		ball_tracker.hpp
 * @author		Takuya Niibori
 * @attention	none
 */

#ifndef BALL_TRACKER_HPP_
#define BALL_TRACKER_HPP_

//C++ Standard Library
#include <cstdint>
#include <memory>
//C Standard Library
//Add Install Library
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <dynamic_reconfigure/server.h>
#include <ball_tracker/BallTrackerConfig.h>

//My Library

/**
 * @class	BallTracker
 * @brief	Ball Tracker
 */
class BallTracker {
public:

	//***** User Define *****

	//***** Const Value *****
	const std::string TOPIC_IN_IMAGE = "image_in";
	const std::string TOPIC_IN_CAMERA_INFO = "camera_info";
	const std::string TOPIC_IN_POINTS2 = "points2_in";
	const std::string TOPIC_OUT_IMAGE = "image_out";

	const std::string PARAM_NAME_LOWER_H = "th_lower_h";
	const std::string PARAM_NAME_LOWER_S = "th_lower_s";
	const std::string PARAM_NAME_LOWER_V = "th_lower_v";
	const std::string PARAM_NAME_UPPER_H = "th_upper_h";
	const std::string PARAM_NAME_UPPER_S = "th_upper_s";
	const std::string PARAM_NAME_UPPER_V = "th_upper_v";

	const std::string PARAM_NAME_DEPTH = "th_depth";

	//***** Constructor, Destructor *****
	BallTracker(ros::NodeHandle&);
	virtual ~BallTracker();

	//***** Method *****
	void publishTest();

private:
	//***** User Define *****
	// Dynamic reconfigure
	using	BTConfig = ball_tracker::BallTrackerConfig;
	using	ReconfigureServer = dynamic_reconfigure::Server<BTConfig>;

	//***** Const Value *****
	static constexpr int32_t HSV_CH = 3;	//!< HSV Channel
	static constexpr int32_t HSV_H = 0;		//!< HSV Hue Channel
	static constexpr int32_t HSV_S = 1;		//!< HSV Saturation Channel
	static constexpr int32_t HSV_V = 2;		//!< HSV Value Channel

	//***** Method *****
	void callbackExactTime(const sensor_msgs::ImageConstPtr&, const sensor_msgs::PointCloud2ConstPtr&);
	void callbackConfig(BTConfig&, uint32_t);

	//***** Member Variable *****
	ros::NodeHandle mNh;				//!< ROS node handle
	image_transport::ImageTransport mIt;

	image_transport::Publisher mImagePub;

	ros::Publisher mPubTest;			//!< ROS Publisher(for Test)

	message_filters::Subscriber<sensor_msgs::Image> mSubImage;
	message_filters::Subscriber<sensor_msgs::PointCloud2> mSubPointCloud2;
	using syncPolicyT = message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::PointCloud2>;
	using synchronizerT = message_filters::Synchronizer<syncPolicyT>;
	synchronizerT mSync;

	// Dynamic reconfigure
	ReconfigureServer	mSrv;
	ReconfigureServer::CallbackType	mCallBckTyp;

};

#endif /* BALL_TRACKER_HPP_ */

