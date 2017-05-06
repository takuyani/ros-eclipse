/**
 * @brief		Ball Tracker
 *
 * @file		ball_tracker.cpp
 * @author		Takuya Niibori
 * @attention	none
 */

//C++ Standard Library
//C Standard Library
//Add Install Library
#include <ros/ros.h>

#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

//My Library
#include "ball_tracker.hpp"

using namespace std;
using namespace ros;
using namespace sensor_msgs;
using namespace message_filters;

/**
 * @brief	Constructor
 *
 * @param[in]		aNh			Ros node handle.
 */
BallTracker::BallTracker(ros::NodeHandle &aNh) :
		mNh(aNh), mIt(aNh), mSubImage(aNh, ros::names::remap(TOPIC_IN_IMAGE),
				1), mSubPointCloud2(aNh, ros::names::remap(TOPIC_IN_POINTS2),
				1), mSync(syncPolicyT(10), mSubImage, mSubPointCloud2) {

	constexpr uint8_t LOWER_H_DEF = 0;	// Lower Hue
	constexpr uint8_t LOWER_S_DEF = 0;	// Lower Saturation
	constexpr uint8_t LOWER_V_DEF = 0;	// Lower Value
	constexpr uint8_t UPPER_H_DEF = 255;	// Upper Hue
	constexpr uint8_t UPPER_S_DEF = 255;	// Upper Saturation
	constexpr uint8_t UPPER_V_DEF = 255;	// Upper Value
	constexpr double DEPTH_DEF = 5.0;		// Depth Value(5.0[m])

	if (mNh.hasParam(PARAM_NAME_LOWER_H) == false) {
		mNh.setParam(PARAM_NAME_LOWER_H, LOWER_H_DEF);
	}

	if (mNh.hasParam(PARAM_NAME_LOWER_S) == false) {
		mNh.setParam(PARAM_NAME_LOWER_S, LOWER_S_DEF);
	}

	if (mNh.hasParam(PARAM_NAME_LOWER_V) == false) {
		mNh.setParam(PARAM_NAME_LOWER_V, LOWER_V_DEF);
	}

	if (mNh.hasParam(PARAM_NAME_UPPER_H) == false) {
		mNh.setParam(PARAM_NAME_UPPER_H, UPPER_H_DEF);
	}

	if (mNh.hasParam(PARAM_NAME_UPPER_S) == false) {
		mNh.setParam(PARAM_NAME_UPPER_S, UPPER_S_DEF);
	}

	if (mNh.hasParam(PARAM_NAME_UPPER_V) == false) {
		mNh.setParam(PARAM_NAME_UPPER_V, UPPER_V_DEF);
	}

	if (mNh.hasParam(PARAM_NAME_DEPTH) == false) {
		mNh.setParam(PARAM_NAME_DEPTH, DEPTH_DEF);
	}

	ROS_INFO_STREAM("subscribe_image: " + ros::names::remap(TOPIC_IN_IMAGE));
//	mSubImgRaw = mNh.subscribe(ros::names::remap(TOPIC_IN_IMAGE), 1, &BallTracker::callback_image, this);

	ROS_INFO_STREAM(
			"subscribe_camera_info: "
					+ ros::names::remap(TOPIC_IN_CAMERA_INFO));
//	mSubInfo = mNh.subscribe(ros::names::remap(TOPIC_IN_CAMERA_INFO), 1, &BallTracker::callback_camera_info, this);

	ROS_INFO_STREAM(
			"subscribe_points2: " + ros::names::remap(TOPIC_IN_POINTS2));
//	mSubPoints = mNh.subscribe(ros::names::remap(TOPIC_IN_POINTS2), 1, &BallTracker::callback_point_cloud, this);

	mSync.registerCallback(
			boost::bind(&BallTracker::callbackExactTime, this, _1, _2));

	ROS_INFO_STREAM("advertise: " + ros::names::remap(TOPIC_OUT_IMAGE));
	mImagePub = mIt.advertise(TOPIC_OUT_IMAGE, 1);

	mCallBckTyp = boost::bind(&BallTracker::callbackConfig, this, _1, _2);
	mSrv.setCallback(mCallBckTyp);

}

/**
 * @brief	Destructor
 */
BallTracker::~BallTracker() {
}

/**
 * @brief			test
 *
 * @return			none
 * @exception		none
 */
void BallTracker::publishTest() {
}

void BallTracker::callbackExactTime(const ImageConstPtr &aImg_cptr,
		const PointCloud2ConstPtr &aSmPc2) {
	std_msgs::String str;
	str.data = "Ball Tracker Test";

	ROS_INFO_STREAM("call callback ApproximateTime");

//	pcl::PointCloud<pcl::PointXYZ> cld;
//
//	pcl::PCLPointCloud2 pcl_pc2;
//	pcl_conversions::toPCL(*aSmPc2, pcl_pc2);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);
//
//	// Create the segmentation object
//	pcl::SACSegmentation<pcl::PointXYZ> seg;

//	// Optional
//	seg.setOptimizeCoefficients(true);
//	// Mandatory
//	seg.setModelType(pcl::SACMODEL_NORMAL_SPHERE);
//	seg.setMethodType(pcl::SAC_RANSAC);
//	seg.setDistanceThreshold(0.3); //容認する誤差範囲
//	seg.setRadiusLimits(0.1, 0.15); // Set the minimum and maximum allowable radius limits for the model
//	seg.setInputCloud(cld_ptr);
//	seg.setEpsAngle(15 / (180 / 3.141592654));
//	seg.setMaxIterations(1000000);
//
//	pcl::PointIndices inlierIndices;
//	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//
//	seg.segment(inlierIndices, *coefficients);
//
//	if (inlierIndices.indices.size() == 0)
//		ROS_INFO("RANSAC nothing found");
//	else {
//		ROS_INFO("RANSAC found shape with [%d] points", (int)inlierIndices.indices.size());
//		for (int c = 0; c < coefficients->values.size(); ++c)
//			ROS_INFO("Coeff %d = [%f]", (int)c+1, (float)coefficients->values[c]);
//
//		// mark the found inliers in green
//		for (int m = 0; m < inlierIndices.indices.size(); ++m) {
//			cld_ptr->points[inlierIndices.indices[m]].r = 0;
//			cld_ptr->points[inlierIndices.indices[m]].g = 255;
//			cld_ptr->points[inlierIndices.indices[m]].b = 0;
//		}
//	}

	cv_bridge::CvImagePtr cvImgPtr_RGB8, cvImgPtr_MONO8;
	try {
		// ROSからOpenCVの形式にtoCvCopy()で変換。cv_ptr->imageがcv::Matフォーマット。
		cvImgPtr_RGB8 = cv_bridge::toCvCopy(aImg_cptr,
				sensor_msgs::image_encodings::BGR8);
		cvImgPtr_MONO8 = cv_bridge::toCvCopy(aImg_cptr,
				sensor_msgs::image_encodings::MONO8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	cv::Mat hsvImg; //color_mask, gray_image, cv_image2, cv_image3;
	// RGB表色系をHSV表色系へ変換して、hsvImgに格納
	cv::cvtColor(cvImgPtr_RGB8->image, hsvImg, CV_BGR2HSV);

	// 色相(Hue), 彩度(Saturation), 明暗(Value, brightness)
	// 指定した範囲の色でマスク画像color_mask(CV_8U:符号なし8ビット整数)を生成
	// マスク画像は指定した範囲の色に該当する要素は255(8ビットすべて1)、それ以外は0
	cv::Mat maskImgMat;
	int32_t lowerHSV[HSV_CH], upperHSV[HSV_CH];
	mNh.getParam(PARAM_NAME_LOWER_H, lowerHSV[HSV_H]);
	mNh.getParam(PARAM_NAME_LOWER_S, lowerHSV[HSV_S]);
	mNh.getParam(PARAM_NAME_LOWER_V, lowerHSV[HSV_V]);
	mNh.getParam(PARAM_NAME_UPPER_H, upperHSV[HSV_H]);
	mNh.getParam(PARAM_NAME_UPPER_S, upperHSV[HSV_S]);
	mNh.getParam(PARAM_NAME_UPPER_V, upperHSV[HSV_V]);

	cv::Scalar lowerScalar, upperScalar;
	lowerScalar.val[0] = lowerHSV[HSV_H];
	lowerScalar.val[1] = lowerHSV[HSV_S];
	lowerScalar.val[2] = lowerHSV[HSV_V];
	lowerScalar.val[3] = 0;
	upperScalar.val[0] = upperHSV[HSV_H];
	upperScalar.val[1] = upperHSV[HSV_S];
	upperScalar.val[2] = upperHSV[HSV_V];
	upperScalar.val[3] = 0;
	cv::inRange(hsvImg, lowerScalar, upperScalar, maskImgMat);
	maskImgMat.copyTo(cvImgPtr_MONO8->image);

	//	// ビット毎の論理積。マスク画像は指定した範囲以外は0で、指定範囲の要素は255なので、ビット毎の論理積を適用すると、指定した範囲の色に対応する要素はそのままで、他は0になる。
	//	cv::bitwise_and(cv_ptr->image, cv_ptr->image, cv_image2, color_mask);
	//	// グレースケールに変換
	//	cv::cvtColor(cv_ptr->image, gray_image, CV_BGR2GRAY);
	//	// エッジを検出するためにCannyアルゴリズムを適用
	//	cv::Canny(gray_image, cv_ptr3->image, 15.0, 30.0, 3);
	//
	//	// ウインドウに円を描画
	//	cv::circle(cv_ptr->image, cv::Point(100, 100), 20, CV_RGB(0, 255, 0));
	//
	//	// 画像サイズは縦横半分に変更
	//	cv::Mat cv_half_image, cv_half_image2, cv_half_image3;
	//	cv::resize(cv_ptr->image, cv_half_image, cv::Size(), 0.5, 0.5);
	//	cv::resize(cv_image2, cv_half_image2, cv::Size(), 0.5, 0.5);
	//	cv::resize(cv_ptr3->image, cv_half_image3, cv::Size(), 0.5, 0.5);
	//
	//	// ウインドウ表示
	//	cv::imshow("Original Image", cv_half_image);
	//	cv::imshow("Result Image", cv_half_image2);
	//	cv::imshow("Edge Image", cv_half_image3);
	//	cv::waitKey(3);

	// エッジ画像をパブリッシュ。OpenCVからROS形式にtoImageMsg()で変換。
	mImagePub.publish(cvImgPtr_MONO8->toImageMsg());
}

void BallTracker::callbackConfig(BTConfig &config, uint32_t level) {
	ROS_INFO("Reconfigure Request: %d %f %s %s %d", config.int_param,
			config.double_param, config.str_param.c_str(),
			config.bool_param ? "True" : "False", config.size);
}
