#ifndef PETRA_HH
#define PETRA_HH

#include <ros/ros.h>
#include <petra/networkPrediction.h>
#include <petra/processImage.h>
#include <petra/correlation.h>
#include <petra/kalmanFilterCorrelation.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>
#include <petra/Leg.h>
#include <petra/Person.h>
#include <petra/People.h>
#include <regex>
using namespace networkprediction;
using namespace processimage;
using namespace correlation;
using namespace kalmanfiltercorrelation;
using namespace ros;
using namespace cv;

namespace petra{
  class Petra{
  public:
    Petra(ros::NodeHandle nh);
    ~Petra();
  private:
    float petraTimer;
    NetworkPrediction* network;
    ProcessImage* processImg;
    Correlation* correlationEuclidean;
    KalmanFilterCorrelation* correlationKalman;
    ros::Subscriber scan_sub, clear_sub;
    ros::Publisher people_pub, markers_pub;
    ros::Timer timer;
    sensor_msgs::LaserScan globalScan;

    float maxDist, minDist;
    bool correlationEuclideanEnable;
    bool correlationKalmanEnable;

    void scanCallback(const sensor_msgs::LaserScan& scan);
    void clearCallback(const std_msgs::String& data);
    petra::People getPairs(std::vector<petra::Leg> *vectorLegs, ros::Time time);
    float euclideanDistance(geometry_msgs::Point point1, geometry_msgs::Point point2);
    geometry_msgs::Point getCenterPeson(geometry_msgs::Point point1, geometry_msgs::Point point2);
    void publishData(petra::People people, std::vector<petra::Leg> vectorLegs);
    void timerCallback(const ros::TimerEvent& t);
    int idPerson(petra::Person person);
  };
};
#endif
