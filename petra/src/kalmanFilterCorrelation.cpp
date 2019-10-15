#include <petra/kalmanFilterCorrelation.h>

using namespace kalmanfiltercorrelation;

KalmanFilterCorrelation::KalmanFilterCorrelation(int seconds,  float legsDistance){
  this->peopleCount = 0;
  this->secsHistoric = seconds;
  this->maxLegsDistance = legsDistance;
}
KalmanFilterCorrelation::~KalmanFilterCorrelation(){
  this->historicPeople.clear();
}

void KalmanFilterCorrelation::restartHistoric(){
  this->peopleCount = 0;
  this->historicPeople.clear();
}

void KalmanFilterCorrelation::correlatePeople(petra::People *people, std::vector<petra::Leg> *vectorLegs){
  // First call, if the historic is empty initialize it with the people received
  if(this->historicPeople.size() == 0){
    for (int i = 0; i < people->people.size(); i++){
      // Actualize the name of the person
      people->people[i].name = "person_" + std::to_string(this->peopleCount);
      // Create new entry in the historic
      this->historicPeople.push_back(this->newEntryHistoric(people->people[i]));
      // Increment counter
      this->peopleCount++;
    }
  }else{
    float distance = 0.0;
    float minDistance = LONG_MAX;
    float vecPosHist = INT_MAX;
    std::vector<int> indexAssigned;
    // First calculate all predictions of the histotic to compare with new people
    this->calculatePredictions();
    for (int i = 0; i < people->people.size(); i++){
        distance = 0.0;
        minDistance = LONG_MAX;
        vecPosHist = INT_MAX;
        for (int j = 0; j < this->historicPeople.size(); j++){
           distance = this->euclideanDistance(people->people[i].position_person, this->historicPeople[j]->prediction);
           if (distance < minDistance && isIndexAssigned(indexAssigned, j) == false){
             minDistance = distance;
             vecPosHist= j;
           }
        }
        // When leaving the loop we have the position in the variable vecPosHist that corresponds to the historical
        // If this variable still has the value of INT_MAX is that no match was found
        // If it is different from INT_MAX, it means that a match has been found
        if (vecPosHist != INT_MAX && minDistance < this->maxLegsDistance ){
            // Add the positioon to the indexAssigned vector
            indexAssigned.push_back(vecPosHist);
            // Actualize the name of the person
            people->people[i].name = this->historicPeople[vecPosHist]->id;
            // Save in the historic the information of the person
            this->historicPeople[vecPosHist]->currentPosition = people->people[i].position_person;
            this->historicPeople[vecPosHist]->timestamp = people->people[i].header.stamp;

            // Update the info in kalmanFilter
            this->correctionKalman(this->historicPeople[vecPosHist]);

        }else{
          // If not match was found. Create a new entry in the historic
          // Actualize the name of the person
          people->people[i].name = "person_" + std::to_string(this->peopleCount);
          // Create new entry in the historic
          this->historicPeople.push_back(this->newEntryHistoric(people->people[i]));
          // Increment counter
          this->peopleCount++;
        }
    }

    // To the list of leg not pairs do the same
    for (int i = 0; i < vectorLegs->size(); i++){
        distance = 0.0;
        minDistance = LONG_MAX;
        vecPosHist = INT_MAX;
        for (int j = 0; j < this->historicPeople.size(); j++){
          distance = this->euclideanDistance(this->historicPeople[j]->prediction, vectorLegs->at(i).position);
          if (distance < minDistance && isIndexAssigned(indexAssigned, j) == false){
            minDistance = distance;
            vecPosHist= j;
          }
        }
        // If there were a match actualize it, if not wait to the next scan
        if (vecPosHist != INT_MAX && minDistance < this->maxLegsDistance ){
            // Add the positioon to the indexAssigned vector
            indexAssigned.push_back(vecPosHist);

            // Create a new entry to the People vector because this leg
            // belong to a people but with the oclusion PeTra not see both legs
            petra::Person person;
            person.header = vectorLegs->at(i).header;
            person.name = this->historicPeople[vecPosHist]->id;
            person.leg1 = vectorLegs->at(i);
            person.leg2 = vectorLegs->at(i);
      			person.position_person = vectorLegs->at(i).position;
            // Add the person to the vector people msg
            people->people.push_back(person);

            // Save in the historic the information of the person
            this->historicPeople[vecPosHist]->currentPosition = vectorLegs->at(i).position;
            this->historicPeople[vecPosHist]->timestamp = ros::Time::now();

            // Remove the leg from the vector of legs, because now have it
            // in the vector of people
            vectorLegs->erase(vectorLegs->begin() + i);

            // Update the info in kalmanFilter
            this->correctionKalman(this->historicPeople[vecPosHist]);
        }
      }
    indexAssigned.clear();
  }

  // Correct all kalmanFilter
  this->correctionAllKalman();

  // Remove people from the historical if it takes more than the seconds especify
  this->removePast();
}


PersonKalman* KalmanFilterCorrelation::newEntryHistoric(petra::Person person){
  PersonKalman* personHistoric = new PersonKalman();
  personHistoric->id = person.name;
  personHistoric->currentPosition = person.position_person;
  personHistoric->timestamp = person.header.stamp;
  personHistoric->KF = new KalmanFilter(4,2,0,CV_32F);
  personHistoric->measurement = cv::Mat(2,1, CV_32F, Scalar(0));
  personHistoric->measurement.setTo(Scalar(0));

  // Init KalmanFilter
  personHistoric->KF->transitionMatrix = (cv::Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);
  // Is necesary multiply by 1000 because of kalman doesnt work with float numbers
  personHistoric->KF->statePre.at<float>(0) = personHistoric->currentPosition.x * 1000;
  personHistoric->KF->statePre.at<float>(1) = personHistoric->currentPosition.y * 1000;
  personHistoric->KF->statePre.at<float>(2) = 0;
  personHistoric->KF->statePre.at<float>(3) = 0;
  personHistoric->KF->statePost.at<float>(0) = personHistoric->currentPosition.x * 1000;
  personHistoric->KF->statePost.at<float>(1) = personHistoric->currentPosition.y * 1000;
  personHistoric->KF->statePost.at<float>(2) = 0;
  personHistoric->KF->statePost.at<float>(3) = 0;

  setIdentity(personHistoric->KF->measurementMatrix);
  setIdentity(personHistoric->KF->errorCovPost, Scalar::all(10));
  setIdentity(personHistoric->KF->errorCovPre, Scalar::all(10));

  // First predict, to update the internal statePre variable
  this->predictionKalman(personHistoric);

  return personHistoric;
}

void KalmanFilterCorrelation::predictionKalman(PersonKalman* person){

  Mat prediction = person->KF->predict();
  Point predictPt(prediction.at<float>(0),prediction.at<float>(1));
  // Is necesary divide by 1000 because of the previous multiplication
  person->prediction.x = predictPt.x / 1000.0;
  person->prediction.y = predictPt.y / 1000.0;
}

void KalmanFilterCorrelation::correctionKalman(PersonKalman* person){
  person->measurement(0) = person->currentPosition.x * 1000;
  person->measurement(1) = person->currentPosition.y * 1000;
  // Update phase
  person->KF->correct(person->measurement);
}

void KalmanFilterCorrelation::correctionAllKalman(){
  for (int i = 0; i < this->historicPeople.size(); i++){
    historicPeople[i]->measurement(0) = historicPeople[i]->currentPosition.x * 1000;
    historicPeople[i]->measurement(1) = historicPeople[i]->currentPosition.y * 1000;
    // Update phase
    historicPeople[i]->KF->correct(historicPeople[i]->measurement);
  }
}

// Method that calculates all prediction of the list
void KalmanFilterCorrelation::calculatePredictions(){
  for (int i = 0; i < this->historicPeople.size(); i++){
    this->predictionKalman(this->historicPeople[i]);
  }
}

bool KalmanFilterCorrelation::isIndexAssigned(std::vector<int> vector, int index){
  for (int i = 0; i < vector.size(); i++){
    if (vector[i] == index){
      return true;
    }
  }
  return false;
}

float KalmanFilterCorrelation::euclideanDistance(geometry_msgs::Point point1, geometry_msgs::Point point2){
  float distance = 0.0;

  distance = sqrt(pow((point2.x-point1.x),2) + pow((point2.y-point1.y),2));

  return distance;
}

void KalmanFilterCorrelation::removePast(){
  for (int i = 0; i < this->historicPeople.size(); i++){
    if (ros::Time::now().toSec() - historicPeople[i]->timestamp.sec > this->secsHistoric){
      historicPeople.erase(historicPeople.begin() + i);
  	}
	}
}
