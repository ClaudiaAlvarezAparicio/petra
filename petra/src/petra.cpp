#include <petra/petra.h>

using namespace petra;

int main(int argc, char* argv[]){
    ros::init(argc, argv, "petra");

    ros::NodeHandle nh("~");

    Petra petra(nh);

    ros::spin();

    return 0;
}

Petra::Petra(ros::NodeHandle nh){
  std::string scan_topic = "";
  int secsHistoric;
  std::string networkModel;
  std::string outNetworkLayer;

  // Read parameters
  nh.getParam("scanTopic", scan_topic);
  nh.getParam("maxLegsDistance", this->maxDist);
  nh.getParam("minLegsDistance", this->minDist);
  nh.getParam("correlationEuclidean", this->correlationEuclideanEnable);
  nh.getParam("correlationKalman", this->correlationKalmanEnable);
  nh.getParam("historicTime", secsHistoric);
  nh.getParam("petraTimer", this->petraTimer);
  nh.getParam("networkModel", networkModel);
  nh.getParam("outNerworkLayer", outNetworkLayer);

  // Objects to predict and proccess images
  this->network = new NetworkPrediction(networkModel, outNetworkLayer);

  this->processImg = new ProcessImage();

  if (this->correlationEuclideanEnable == true){
    this->correlationEuclidean = new Correlation(secsHistoric);
  }else if(this->correlationKalmanEnable == true){
    this->correlationKalman = new KalmanFilterCorrelation(secsHistoric, this->maxDist);
  }

  // Publishers, Subscribers and Timer
  scan_sub = nh.subscribe(scan_topic, 1000, &Petra::scanCallback, this);
  clear_sub = nh.subscribe("restart", 1000, &Petra::clearCallback, this);
  people_pub = nh.advertise<petra::People>("/people", 100);
  markers_pub = nh.advertise<visualization_msgs::MarkerArray>("/markers", 100);
  timer = nh.createTimer(ros::Duration(this->petraTimer), &Petra::timerCallback, this);
}

Petra::~Petra(){
  delete this->network;
  delete this->processImg;
  if(this->correlationKalmanEnable == true){
    delete this->correlationKalman;
  }else if(this->correlationEuclideanEnable == true){
    delete this->correlationEuclidean;
  }
}

void Petra::clearCallback(const std_msgs::String& data){
  if(this->correlationKalmanEnable == true){
    this->correlationKalman->restartHistoric();
  }else if(this->correlationEuclideanEnable == true){
    this->correlationEuclidean->restartHistoric();
  }
}

void Petra::timerCallback(const ros::TimerEvent& t){

  // Prediction of the network
  cv::Mat image = this->network->prediction(this->globalScan);

  std::vector<petra::Leg> vectorLegs = this->processImg->getLegsFromImage(image, this->globalScan.header.frame_id);

  petra::People people = this->getPairs(&vectorLegs, this->globalScan.header.stamp);

  if (this->correlationEuclideanEnable == true){
    this->correlationEuclidean->correlatePeople(&people, &vectorLegs);
  }else if(this->correlationKalmanEnable == true){
    this->correlationKalman->correlatePeople(&people, &vectorLegs);
  }
  people.header.stamp = this->globalScan.header.stamp;

  this->publishData(people, vectorLegs);
}

void Petra::scanCallback(const sensor_msgs::LaserScan& scan){
  this->globalScan=scan;
}

petra::People Petra::getPairs(std::vector<petra::Leg> *vectorLegs, ros::Time time){
  // This msg contains an array of Person msg
  petra::People people;
  // Max value of float to inicialize
  float lessDist = LONG_MAX;
  // Positions of the vector to match legs
  int vecPosI, vecPosJ = 0;
  // Not legs matching with the preconditions
  bool notMatch = false;
  float distance = 0.0;

  while (notMatch == false && vectorLegs->size() > 1){
    for (int i = 0; i < vectorLegs->size() - 1; i++){
      for (int j = i + 1; j < vectorLegs->size(); j++){
        distance = this->euclideanDistance(vectorLegs->at(i).position, vectorLegs->at(j).position);
        // Test if the euclidean distance is less that the previous and save it
        if (distance < lessDist){
          lessDist = distance;
          vecPosI = i;
          vecPosJ = j;
        }
      }
    }

    // If satisfy both requisites, create a new msg Person
		if (this->maxDist > lessDist and lessDist > this->minDist){
			petra::Person person;
      person.header.stamp = time;
      person.header.frame_id = vectorLegs->at(vecPosI).header.frame_id;
      person.name = "person_" + std::to_string(people.people.size());
      vectorLegs->at(vecPosI).name = "leg1";
      vectorLegs->at(vecPosJ).name = "leg2";
      person.leg1 = vectorLegs->at(vecPosI);
      person.leg2 = vectorLegs->at(vecPosJ);
			person.position_person = this->getCenterPeson(vectorLegs->at(vecPosI).position, vectorLegs->at(vecPosJ).position);
      // Add the person to the vector people msg
      people.people.push_back(person);

      // Remove the asigned legs from the vector
      if (vecPosI > vecPosJ){
        vectorLegs->erase(vectorLegs->begin() + vecPosI);
        vectorLegs->erase(vectorLegs->begin() + vecPosJ);
      }else{
        vectorLegs->erase(vectorLegs->begin() + vecPosJ);
        vectorLegs->erase(vectorLegs->begin() + vecPosI);
      }

      // Restore the value of the variable
      lessDist = LONG_MAX;

		}else{
      // If the max and min distance not match. The points are not pairing
			notMatch = true;
		}
  }

  return people;
}

float Petra::euclideanDistance(geometry_msgs::Point point1, geometry_msgs::Point point2){
  float distance = 0.0;

  distance = sqrt(pow((point2.x-point1.x),2) + pow((point2.y-point1.y),2));

  return distance;
}

geometry_msgs::Point Petra::getCenterPeson(geometry_msgs::Point point1, geometry_msgs::Point point2){
  geometry_msgs::Point point;
  point.x = (point1.x + point2.x) / 2;
  point.y = (point1.y + point2.y) / 2;
  return point;
}


int Petra::idPerson(petra::Person person){
  std::string id = person.name;
  id.replace(0,7,"");
  return stoi(id);
}
void Petra::publishData(petra::People people, std::vector<petra::Leg> vectorLegs){
  int id;
  visualization_msgs::MarkerArray arrayMarkers;
  // People
  for (int i = 0; i < people.people.size(); i++){
    id = this->idPerson(people.people[i]);

    visualization_msgs::Marker person;
    visualization_msgs::Marker leg1;
    visualization_msgs::Marker leg2;

    person.header.stamp = people.people[i].header.stamp;
    person.header.frame_id =  people.people[i].header.frame_id;
    person.ns = people.people[i].name;
    person.id = id;
    if (this->correlationEuclideanEnable == true || this->correlationKalmanEnable == true){
      person.type = person.TEXT_VIEW_FACING;
      person.text = people.people[i].name;
    }else{
      person.type = person.SPHERE;
    }
    // CENTER
    person.pose.position.x = people.people[i].position_person.x;
    person.pose.position.y = people.people[i].position_person.y;
    person.scale.x = 0.15;
    person.scale.y = 0.15;
    person.scale.z = 0.15;
    person.color.a = 1;
    person.color.b = 1;
    person.lifetime = ros::Duration(0.2);

    // LEG 1
    leg1.header.stamp = people.people[i].header.stamp;
    leg1.header.frame_id = people.people[i].header.frame_id;
    leg1.ns = people.people[i].name;
    leg1.id = id + 50; // to not overwrite the points
    leg1.type = leg1.SPHERE;
    leg1.pose.position.x = people.people[i].leg1.position.x;
    leg1.pose.position.y = people.people[i].leg1.position.y;
    leg1.scale.x = 0.1;
    leg1.scale.y = 0.1;
    leg1.scale.z = 0.1;
    leg1.color.a = 1;
    leg1.color.r = 1;
    leg1.lifetime = ros::Duration(0.2);

    // LEG 2
    leg2.header.stamp = people.people[i].header.stamp;
    leg2.header.frame_id = people.people[i].header.frame_id;
    leg2.ns = people.people[i].name;
    leg2.id = id + 100; // to not overwrite the points
    leg2.type = leg2.SPHERE;
    leg2.pose.position.x = people.people[i].leg2.position.x;
    leg2.pose.position.y = people.people[i].leg2.position.y;
    leg2.scale.x = 0.1;
    leg2.scale.y = 0.1;
    leg2.scale.z = 0.1;
    leg2.color.a = 1;
    leg2.color.r = 1;
    leg2.lifetime = ros::Duration(0.2);
    // Add markers to array
    arrayMarkers.markers.push_back(person);
    arrayMarkers.markers.push_back(leg1);
    arrayMarkers.markers.push_back(leg2);
  }

  // Legs not pairs
  for (int i = 0; i < vectorLegs.size(); i++){
    visualization_msgs::Marker leg;
    leg.header.stamp = vectorLegs[i].header.stamp;
    leg.header.frame_id = vectorLegs[i].header.frame_id;
    leg.ns = vectorLegs[i].name;
    leg.id = i + 150; // to not overwrite the points
    leg.type = leg.SPHERE;
    leg.pose.position.x = vectorLegs[i].position.x;
    leg.pose.position.y = vectorLegs[i].position.y;
    leg.scale.x = 0.1;
    leg.scale.y = 0.1;
    leg.scale.z = 0.1;
    leg.color.a = 1;
    leg.color.b = 1;
    leg.lifetime = ros::Duration(0.2);
    arrayMarkers.markers.push_back(leg);
  }

  // Publish both topics
  people_pub.publish(people);
  markers_pub.publish(arrayMarkers);
}
