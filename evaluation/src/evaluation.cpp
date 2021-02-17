#include <evaluation/evaluation.h>

using namespace evaluation;

//using namespace cv;
using namespace std;

int main(int argc, char** argv){

  ros::init(argc, argv, "evaluation");
  ros::NodeHandle nh("~");
  Evaluation* e = new Evaluation(nh);
  ros::spin();

  e->print_data();

  return 0;
}

Evaluation::Evaluation(ros::NodeHandle nh): 
tf_(), target_frame("/map"), private_nh_("~"){

  std::vector<double> vector_double_aux;
  std::vector<int> vector_int_aux;
  for (int i = 0; i < 15; i ++){
    nh.getParam("point_" + std::to_string(i+1), vector_double_aux);
    this->vector_map_positions.push_back(vector_double_aux);
  }

  for (int i = 0; i < 20; i ++){
    nh.getParam("rosbag_" + std::to_string(i+1), vector_int_aux);
    this->vector_rosbags_positions.push_back(vector_int_aux);
  }

  
  private_nh_.getParam("rosbag_file", this->rosbag);
  private_nh_.getParam("threshold", this->threshold);
  private_nh_.getParam("method", this->method);
  private_nh_.getParam("csv_directory", this->csv_directory);
  

  people_sub.subscribe(nh, "/people", 100);
  tf_filter = new tf::MessageFilter<petra::People>(people_sub, tf_, target_frame, 100);
  tf_filter->registerCallback( boost::bind(&Evaluation::pointTransformCallback, this, _1) );

  odom_sub.subscribe(nh, "/robotnik_base_control/odom", 100);
  tf_filter_odom = new tf::MessageFilter<nav_msgs::Odometry>(odom_sub, tf_, target_frame, 100);
  tf_filter_odom->registerCallback( boost::bind(&Evaluation::odomTransformCallback, this, _1) );


  // ESTO PARA LOS ROBSGAS QUE TIENEN PERSONAS
  // Get bag name
  int rosbag = std::stoi(this->get_bag_name());
  // get people positions at the rosbag
  this->position_people_rosbag = this->vector_rosbags_positions[rosbag-1];

  // For each person at the rosbag new vector is created
  // The first position in the vector es the REAL position of the person at scene
  // The rest of the estimations are detected at execution time
  for (int i = 0; i< this->position_people_rosbag.size(); i++){

    std::vector<std::vector<double> > vector_positions;
    std::vector<double> vector_position;

    // X Coordinate
    vector_position.push_back(this->vector_map_positions[this->position_people_rosbag[i] - 1][0]);
    // Y Corrdinate
    vector_position.push_back(this->vector_map_positions[this->position_people_rosbag[i] - 1][1]);

    // Vector where save all the positions of the person
    vector_positions.push_back(vector_position);

    // Global vector, save all the people data
    output_position.push_back(vector_positions);

  }

}
/*
 * Get the robot position in map frame
 */
void Evaluation::odomTransformCallback(const boost::shared_ptr<const nav_msgs::Odometry>& odom){
  geometry_msgs::PointStamped point_person_before; 
  geometry_msgs::PointStamped point_person_after;
  point_person_before.header = odom->header;
  point_person_before.point = odom->pose.pose.position;
  try 
    {
      tf_.transformPoint(target_frame, point_person_before , point_person_after);
    }
    catch (tf::TransformException &ex) {
      printf ("Failure %s\n", ex.what()); 
    }
    
    this->robot_position = point_person_after;
}

/*
 * Get the people position in map frame
 */

void Evaluation::pointTransformCallback(const boost::shared_ptr<const petra::People>& people){

  std::vector<geometry_msgs::PointStamped> people_positions;
  geometry_msgs::PointStamped point_person_before; 
  geometry_msgs::PointStamped point_person_after;

  double x = 0.0;
  double y = 0.0;
  float distance = 0.0;
  std::vector<double> vector_aux;
  bool found = false;

  for (int i = 0; i < people->people.size(); i++){
    point_person_before.header = people->header;
    point_person_before.point = people->people[i].position_person;
    try 
    {
      tf_.transformPoint(target_frame, point_person_before , point_person_after);
    }
    catch (tf::TransformException &ex) {
      printf ("Failure %s\n", ex.what()); 
    }

    people_positions.push_back(point_person_after);
  }

  // Once we have the point in the map frame
  // We compare if this person is in the allowed threshold of anybody person
  for (int i = 0; i < people_positions.size(); i++ ){
    
    // Search if the person was in the range of any person. 
    for (int j = 0; j < output_position.size(); j++){

      // REAL position of the person
      x = output_position[j][0][0];
      y = output_position[j][0][1];

      distance = this->euclideanDistance(people_positions[i].point.x, people_positions[i].point.y, x, y);

      // If the position match with some of the predefined, it is added to the vector, showing that it belongs to that person
      if (distance < this->threshold){
        vector_aux.clear();
        vector_aux.push_back(people_positions[i].point.x);
        vector_aux.push_back(people_positions[i].point.y);  
        output_position[j].push_back(vector_aux);
        found = true;
        break;
      }
    }
    // If the position not match withe anybody , it is write in a new column to represent a new person detected
    if (found == false){
      std::vector<std::vector<double> > vector_aux2;
      vector_aux.clear();
      vector_aux.push_back(people_positions[i].point.x);
      vector_aux.push_back(people_positions[i].point.y);  
      vector_aux2.push_back(vector_aux);
      output_position.push_back(vector_aux2);
    }
  }
}

void Evaluation::print_data(){
  // Open CSV
  std::ofstream myfile;
  myfile.open(this->csv_directory+"/"+this->method+"_"+this->get_bag_name()+".csv");

  // General information
  myfile << "Robot position:, " + std::to_string(this->robot_position.point.x) + "," + std::to_string(this->robot_position.point.y) + "\n";
  myfile << "Number of people in the rosbag: " + std::to_string(position_people_rosbag.size()) + "\n";

  int contador_bien_detectadas=0;
  for (int i = 0; i < position_people_rosbag.size(); i++){
    if (output_position[i].size()>1){
      contador_bien_detectadas++;
    } 
  }
  myfile << "Number of people detected: " + std::to_string(contador_bien_detectadas + (output_position.size() - position_people_rosbag.size())) + "\n";
  myfile << "Number of people rigth detected: " + std::to_string(contador_bien_detectadas) + "\n";
  myfile << "Number of people not detected: " + std::to_string(position_people_rosbag.size() - contador_bien_detectadas) + "\n";
  myfile << "Number of poeple wrong detected: " + std::to_string(output_position.size() - position_people_rosbag.size()) + "\n";

  if (output_position.size()>0){
    std::string num = "Number of times detected each person: ,";
    for (int i = 0; i < output_position.size(); i++){
      if (i < position_people_rosbag.size()){
        num = num + std::to_string(output_position[i].size()-1) + ",,";
      }else{
        num = num + std::to_string(output_position[i].size()) + ",,";
      }
    }

    myfile << num + "\n";


    // Header
    std::string cabecera = ",";
    int contador = 0;
    for (int i = 0; i < output_position.size(); i++){
      if (i < position_people_rosbag.size()){
        cabecera = cabecera + "Person at " + std::to_string(position_people_rosbag[i]) + ",,";// Ponemos dos comas para "ocupar 2 columnas para los datos de despues"
      }else{
        cabecera = cabecera + "Person ERROR " + std::to_string(contador) + ",,";
        contador++;
      }
    }
    cabecera = cabecera + "\n";
    
    myfile << cabecera;

    // Body 
    std::string cuerpo = ",";
    // Take the logest vector to use it as reference
    int referencia = -1;
    int cantidad = 0;
    for (int i = 0; i < output_position.size(); i++){
      if (output_position[i].size() > cantidad){
        referencia=i;
        cantidad=output_position[i].size();
      }
    }
    // We start to write the values
    for (int i = 0; i < output_position[referencia].size(); i++){
      cuerpo=",";
      for(int j = 0; j < output_position.size(); j++){
        if(output_position[j].size() > i){// If it is larger has elements to write
          cuerpo = cuerpo + std::to_string(output_position[j][i][0]) +","+ std::to_string(output_position[j][i][1]) +",";
        }else{ // Not has elements to write
          cuerpo = cuerpo + ",,"; // For empty cells
        }
      }
      myfile << cuerpo + "\n";
    }
  }
  myfile.close();
}


std::string Evaluation::get_bag_name(){
  std::string archivo = this->rosbag;
  size_t found = archivo.find_last_of("/");
  archivo.replace(0,found+1,"");
  found = archivo.find_last_of(".");
  archivo.replace(found,4,"");
  return archivo;
}

float Evaluation::euclideanDistance(double x1, double y1, double x2, double y2){
  float distance = 0.0;

  distance = sqrt(pow((x2-x1),2) + pow((y2-y1),2));

  return distance;
}
