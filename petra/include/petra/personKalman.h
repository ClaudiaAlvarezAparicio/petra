#ifndef PERSONKALMAN_HH
#define PERSONKALMAN_HH

#include <geometry_msgs/Point.h>
#include <opencv/cv.hpp>
#include <ros/ros.h>
using namespace cv;

struct PersonKalman{
	std::string id;
	geometry_msgs::Point currentPosition;
  cv::KalmanFilter* KF;
  Mat_<float> measurement;
	geometry_msgs::Point prediction;
  ros::Time timestamp;
};

#endif // PERSONKALMAN_HH
