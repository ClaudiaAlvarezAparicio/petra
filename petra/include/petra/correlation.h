#ifndef CORRELATION_HH
#define CORRELATION_HH
#include <ros/ros.h>
#include <petra/correlation.h>
#include <petra/Leg.h>
#include <petra/Person.h>
#include <petra/People.h>

using namespace ros;

namespace correlation{
  class Correlation{
  public:
    Correlation(int seconds);
    ~Correlation();
    void correlatePeople(petra::People *people, std::vector<petra::Leg> *vectorLegs);
    void restartHistoric();

  private:
    int peopleCount;
    int secsHistoric;
    petra::People historicPeople;

    float euclideanDistance(geometry_msgs::Point point1, geometry_msgs::Point point2);
    bool isIndexAssigned(std::vector<int> vector, int index);
    void removePast();
  };
};
#endif
