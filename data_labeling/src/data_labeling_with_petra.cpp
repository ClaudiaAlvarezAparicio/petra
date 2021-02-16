#include <data_labeling/data_labeling_with_petra.h>

using namespace data_labeling_petra;

using namespace cv;
using namespace std;

int main(int argc, char** argv){

  ros::init(argc, argv, "data_labeling_with_petra");
  ros::NodeHandle nh;
  DataLabelingPetra* dl = new DataLabelingPetra(nh);
  ros::spin();

  dl->save_npy();

  return 0;
}

DataLabelingPetra::DataLabelingPetra(ros::NodeHandle nh): private_nh_("~"){
  // Save params
  private_nh_.getParam("rosbag_file", this->rosbag);
  private_nh_.getParam("npy_directory", this->directory);
  private_nh_.getParam("scan_topic", this->scan_topic);

  // Sync PeTra and Scan
  people_sub.subscribe(nh, "/people", 1);
  scan_sub.subscribe(nh, this->scan_topic, 1);
  
  sync.reset(new Sync(MySyncPolicy(10), people_sub, scan_sub));      
  sync->registerCallback(boost::bind(&DataLabelingPetra::peopleScanCallback, this, _1, _2));
}


void DataLabelingPetra::peopleScanCallback(const petra::PeopleConstPtr& petra, const sensor_msgs::LaserScanConstPtr& scan){

    if (petra->people.size() > 0){       
      // Matrix initialization
      this->initialize_matrix();

      for (int i = 0; i < petra->people.size(); i++){
      
        // Si se han identificado las piernas de la persona, etiquetamos en base a esa info
        if (petra->people[i].leg1.position.x != 0 || petra->people[i].leg1.position.y || petra->people[i].leg2.position.x != 0 || petra->people[i].leg2.position.y != 0){
          range_person = 0.15;
          this->classify_scan_data(scan, petra->people[i].leg1.position);
          this->classify_scan_data(scan, petra->people[i].leg2.position);
        }else{
          // Si no etiquetamos con el centro de la persona
          range_person = 0.30;
          this->classify_scan_data(scan, petra->people[i].position_person);
        }
      }

      this->global_raw.push_back(this->matrix_to_vector(matrix_raw));
      this->global_label.push_back(this->matrix_to_vector(matrix_label));

    }
}


/*
 * Method that returns the lidar scan with the same timestamp than petra
 */
sensor_msgs::LaserScan DataLabelingPetra::getLaserScan(std_msgs::Header header_petra){
  sensor_msgs::LaserScan scan;
  int position = 0;
  for (int i = 0; i< this->historicScan.size(); i++){
    if(this->historicScan[i].header.stamp == header_petra.stamp){
      scan = historicScan[i];
      position = i;
    }
  }
  // Remove the previous scans
  this->historicScan.erase(this->historicScan.begin(), this->historicScan.begin()+position);
  // return scan msgs
  return scan;
}

void DataLabelingPetra::save_npy(){
  const long unsigned longs [] = {this->global_raw.size(), LENGTH_MATRIX, LENGTH_MATRIX,1};

  std::vector<int> global_vector_raw = this->vector_to_global_vector(this->global_raw);
  std::vector<int> global_vector_label = this->vector_to_global_vector(this->global_label);

  npy::SaveArrayAsNumpy(this->directory+"/raw/"+this->get_name_npy_file(), false, 4, longs, global_vector_raw);
  npy::SaveArrayAsNumpy(this->directory+"/label/"+this->get_name_npy_file(), false, 4, longs, global_vector_label);
}

std::string DataLabelingPetra::get_name_npy_file(){
  std::string archivo = this->rosbag;
  size_t found = archivo.find_last_of("/");
  archivo.replace(0,found,"");
  found = archivo.find_last_of(".");
  archivo.replace(found,4,".npy");
  return archivo;
}

std::vector<int> DataLabelingPetra::vector_to_global_vector(std::vector<std::vector<int> > global_vector){
  std::vector<int> vector_out;
  for (int i = 0; i < global_vector.size(); i++){
    for (int j = 0; j < global_vector[i].size();j++){
      vector_out.push_back(global_vector[i][j]);
    }
  }
  return vector_out;
}



/*
 * Method that save
 * In the matrix_raw, like 1 all the points of the laser, like 0 the rest
 * In the matrix_label with 1s the points of the laser that compose a person
 */
void DataLabelingPetra::classify_scan_data(const sensor_msgs::LaserScanConstPtr& scan_info , geometry_msgs::Point point_person){
    std::vector<float> vector_scan = scan_info->ranges;
    geometry_msgs::Point point_laser;
    bool in_range = false;
    int j, k;
    int counter= 0;

    for (int i= 0; i < vector_scan.size(); i++){
      j = k = 0;
      // Is the range is good
      if (this->is_good_value(vector_scan[i]) == true){
        // Calculate the xy point from the angle/range
        point_laser = this->get_point_xy(vector_scan[i],i, scan_info);

        // Calculate if the point is in range of the person
        in_range = this-> is_in_range(point_person, point_laser);

        // Calculate the position of the point in the matrix
        this->get_point_in_matrix(point_laser,&j,&k);
        // If the point is in range it is write in both matrix
        // if not only in the matrix_raw
        if(j>=0 and j<LENGTH_MATRIX and k>=0 and k<LENGTH_MATRIX){
          //std::cout << "J,K: " << j << "   " << k << '\n';
          if (in_range == true){
            matrix_raw[j][k] = 1;
            matrix_label[j][k] = 1;
            counter++;
          }else{
            matrix_raw[j][k] = 1;
          }
        }
      }
    }   
}


/*
 * Method that calculate if the point of the laser is in range of the person
 * Returns true if it is in range if not it return false
 */
bool DataLabelingPetra::is_in_range(geometry_msgs::Point point_person, geometry_msgs::Point point_laser){
  float distancia_euclidea = 0;

  // Calculate de euclidean distance between point of person and the point of laser
  distancia_euclidea = sqrt(pow(point_laser.x - point_person.x , 2) +pow(point_laser.y - point_person.y , 2));

  if (distancia_euclidea < range_person){
    vector_points.push_back(point_laser);
    return true;
  }else{
    return false;
  }
}

/*
 * Method that transform the range provided by the laser to polar coordinates
 * Return a Point object with the point in cartesian coordinates (x,y)
 */
geometry_msgs::Point DataLabelingPetra::get_point_xy(float range, int index, const sensor_msgs::LaserScanConstPtr& scan_data){

  geometry_msgs::Point point_xy;
  float polar_d = range;
  float polar_angle = 0;
  float polar_angle_radians = 0;
  float cartesian_x = 0;
  float cartesian_y = 0;
  float alfa_radians = 0;

  // alfa is the complementary angle in the polar coordinates
  alfa_radians = (scan_data->angle_increment * (index + 1)) + scan_data->angle_min;

  // Calculate the point (x,y)
  cartesian_x = polar_d * cos(alfa_radians);
  cartesian_y = polar_d * sin(alfa_radians);

  // Add the values to the object to return it
  point_xy.x = cartesian_x;
  point_xy.y = cartesian_y;

  return point_xy;
}

/*
 * Method that calculates the point xy of the laser in a ij position in the matrix
 */
void DataLabelingPetra::get_point_in_matrix(geometry_msgs::Point point_laser, int *i, int *j){
  int half_matrix = LENGTH_MATRIX / 2;
  //std::cout << "/* puntos x, y */" << point_laser.x << "   " << point_laser.y << '\n';
  // transform the values to cm and divided it by 2 to do the transformation to matrix
  int point_x = floor((point_laser.x * 100)/2);
  int point_y = floor((point_laser.y * 100)/2);

  *i = point_x;
  *j = half_matrix - point_y;
}

/*
 * Method that transform the matrix recived to a vector and return it
 */
std::vector<int> DataLabelingPetra::matrix_to_vector(int matriz[LENGTH_MATRIX][LENGTH_MATRIX]){
  std::vector<int> vector_out;

  for (int i = 0; i < LENGTH_MATRIX; i++){
    for (int j = 0; j < LENGTH_MATRIX;j++){
      vector_out.push_back(matriz[i][j]);
    }
  }
  return vector_out;
}

/*
 * Method that initialize both matrix to 0s
 */
void DataLabelingPetra::initialize_matrix(){
  for (int i = 0; i < LENGTH_MATRIX; i++){
    for (int j = 0; j < LENGTH_MATRIX; j++){
      matrix_raw[i][j] = 0;
      matrix_label[i][j] = 0;
    }
  }
}
/*
 * Method that determines if the value tha recives is not equal to inf, nan, -inf or -nan, and if it is in range
 * Return true if is a good value and false if is not
 */
bool DataLabelingPetra::is_good_value(float num){

  bool good_value = true;
  if (isnan(num) || isnan(-num)){
    good_value = false;
  }

  if (isinf(num) || isinf(-num)){
    good_value = false;
  }

  // If the value is greater than 5.12m, we ignore it
  if (num > 10.24){
    good_value = false;
  }

  return good_value;
}
