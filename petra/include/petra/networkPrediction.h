#ifndef NETWORKPREDICTION_HH
#define NETWORKPREDICTION_HH

#include "tensorflow/core/public/session.h"
#include "tensorflow/core/platform/env.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <opencv/cv.hpp>
#include <cmath>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>

using namespace tensorflow;
using namespace ros;
using namespace cv;

#define LENGTH_MATRIX 256


namespace networkprediction{
  class NetworkPrediction{
  public:
    NetworkPrediction(std::string networkModel, std::string outNetworkLayer);
    ~NetworkPrediction();
    cv::Mat prediction(sensor_msgs::LaserScan scan);

  private:
    Session* session;
    tensorflow::SessionOptions options;
    Status status;
    GraphDef graph_def;
    char* puntero;
    bool primero = true;
    std::string networkModel = "modelo.pb";
    std::string outNetworkLayer = "conv2d_19/Sigmoid";

    cv::Mat createImage(sensor_msgs::LaserScan scan);
    void openConnection();
    void closeConnection();
    bool isGoodValue(float num);
    geometry_msgs::Point getPointXY(float range, int index, float angleIncrement, float angleMin);
    void getPointInMatrix(geometry_msgs::Point pointLaser, int *i, int *j);
  };
};
#endif
