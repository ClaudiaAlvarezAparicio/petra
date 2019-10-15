#include <petra/correlation.h>

using namespace correlation;

Correlation::Correlation(int seconds){
  this->peopleCount = 0;
  this->secsHistoric = seconds;
}
Correlation::~Correlation(){
  this->historicPeople.people.clear();
}

void Correlation::restartHistoric(){
  this->peopleCount = 0;
  this->historicPeople.people.clear();
}

void Correlation::correlatePeople(petra::People *people, std::vector<petra::Leg> *vectorLegs){
  // First call, if the historic is empty initialize it with the people received
  if(this->historicPeople.people.size() == 0){
    for (int i = 0; i < people->people.size(); i++){
      // Actualize the name of the person
      people->people[i].name = "person_" + std::to_string(this->peopleCount);
      this->historicPeople.people.push_back(people->people[i]);
      // Increment counter
      this->peopleCount++;
    }
  }else{
    float distance = 0.0;
    float minDistance = LONG_MAX;
    float vecPosHist = INT_MAX;
    std::vector<int> indexAssigned;

    for (int i = 0; i < people->people.size(); i++){
      distance = 0.0;
      minDistance = LONG_MAX;
      vecPosHist = INT_MAX;

      for (int j = 0; j < this->historicPeople.people.size(); j++){
        distance = this->euclideanDistance(people->people[i].position_person, this->historicPeople.people[j].position_person);
        if (distance < minDistance && isIndexAssigned(indexAssigned, j) == false){
          minDistance = distance;
          vecPosHist= j;
        }
      }
      // When leaving the loop we have the position in the variable vecPosHist that corresponds to the historical
      // If this variable still has the value of INT_MAX is that no match was found
      // If it is different from INT_MAX, it means that a match has been found
      if (vecPosHist != INT_MAX){
          // Add the positioon to the indexAssigned vector
          indexAssigned.push_back(vecPosHist);
          // Actualize the name of the person
          people->people[i].name = historicPeople.people[vecPosHist].name;
          // Save in the historic the information of the person
          historicPeople.people[vecPosHist] = people->people[i];
      }else{
        // If not match was found. Create a new entry in the historic
        // Actualize the name of the person
        people->people[i].name = "person_" + std::to_string(this->peopleCount);
        this->historicPeople.people.push_back(people->people[i]);
        // Increment counter
        this->peopleCount++;
      }
    }

    // To the list of leg not pairs do the same
    for (int i = 0; i < vectorLegs->size(); i++){
      distance = 0.0;
      minDistance = LONG_MAX;
      vecPosHist = INT_MAX;
      for (int j = 0; j < historicPeople.people.size(); j++){
        distance = this->euclideanDistance(this->historicPeople.people[j].position_person, vectorLegs->at(i).position);
        if (distance < minDistance && isIndexAssigned(indexAssigned, j) == false){
          minDistance = distance;
          vecPosHist= j;
        }
      }
      // If there were a match actualize it, if not wait to the next scan
      if (vecPosHist != INT_MAX){
          // Add the positioon to the indexAssigned vector
          indexAssigned.push_back(vecPosHist);
          // Actualize the name of the leg
          vectorLegs->at(i).name = historicPeople.people[vecPosHist].name;
          // Save in the historic the information of the person
          historicPeople.people[vecPosHist].position_person = vectorLegs->at(i).position;
          historicPeople.people[vecPosHist].leg1 = vectorLegs->at(i);
          historicPeople.people[vecPosHist].leg2 = vectorLegs->at(i);
      }
    }
  }
  // Remove people from the historical if it takes more than the seconds especify
  this->removePast();
}

bool Correlation::isIndexAssigned(std::vector<int> vector, int index){
  for (int i = 0; i < vector.size(); i++){
    if (vector[i] == index){
      return true;
    }
  }
  return false;
}

float Correlation::euclideanDistance(geometry_msgs::Point point1, geometry_msgs::Point point2){
  float distance = 0.0;

  distance = sqrt(pow((point2.x-point1.x),2) + pow((point2.y-point1.y),2));

  return distance;
}

void Correlation::removePast(){
  for (int i = 0; i < historicPeople.people.size(); i++){
		if (ros::Time::now().toSec() - historicPeople.people[i].header.stamp.sec > this->secsHistoric){
      historicPeople.people.erase(historicPeople.people.begin() + i);
  	}
	}
}
