#ifndef PROCESSIMAGE_HH
#define PROCESSIMAGE_HH

#include <ros/ros.h>
#include <petra/processImage.h>
#include <petra/Leg.h>
#include <opencv/cv.hpp>

using namespace cv;
using namespace ros;

#define LENGTH_MATRIX 256

namespace processimage{
  class ProcessImage{
  public:
    ProcessImage(){};
    ~ProcessImage(){};
    std::vector<petra::Leg> getLegsFromImage(cv::Mat image, std::string frame);
  private:
    geometry_msgs::Point getPointInFrameHokuyo(float pixelX, float pixelY);
  };
};
#endif
