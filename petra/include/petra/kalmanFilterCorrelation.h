#ifndef KALMAN_FILTER_CORRELATION_HH
#define KALMAN_FILTER_CORRELATION_HH

#include <ros/ros.h>
#include <petra/People.h>
#include <petra/personKalman.h>
#include <geometry_msgs/Point.h>
#include <opencv/cv.hpp>

using namespace ros;
using namespace cv;

namespace kalmanfiltercorrelation{
  class KalmanFilterCorrelation{
  public:
    KalmanFilterCorrelation(int seconds, float legsDistance);
    ~KalmanFilterCorrelation();
    void correlatePeople(petra::People *people, std::vector<petra::Leg> *vectorLegs);
    void restartHistoric();

  private:
    int peopleCount;
    int secsHistoric;
    float maxLegsDistance;
    std::vector<PersonKalman*> historicPeople;
    PersonKalman* newEntryHistoric(petra::Person person);
    void predictionKalman(PersonKalman* person);
    void correctionKalman(PersonKalman* person);
    void correctionAllKalman();
    void calculatePredictions();
    float euclideanDistance(geometry_msgs::Point point1, geometry_msgs::Point point2);
    bool isIndexAssigned(std::vector<int> vector, int index);
    void removePast();


  };
};

#endif
