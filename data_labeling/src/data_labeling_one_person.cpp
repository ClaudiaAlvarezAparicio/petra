#include <data_labeling/data_labeling_one_person.h>

using namespace data_labeling_one_person;

using namespace cv;
using namespace std;

int main(int argc, char** argv){

  ros::init(argc, argv, "data_labeling_one_person");
  ros::NodeHandle nh;
  DataLabelingOnePerson dl(nh);
  ros::spin();

  return 0;
}

DataLabelingOnePerson::DataLabelingOnePerson(ros::NodeHandle nh): 
tf_(),range_person(0.50), target_frame("/hokuyo_laser_link"), private_nh_("~"){
  std::string tag_name;
  // Get the tag_name param
  private_nh_.getParam("tag_name",tag_name);

  // Publishers and subscribers
  pub_data = nh.advertise<data_labeling::DataLabelingBW>("data_labeling", 100);
  point_sub.subscribe(nh, tag_name, 100);
  scan_sub= nh.subscribe("/scan", 10, &DataLabelingOnePerson::scanCallback,this);  
  tf_filter = new tf::MessageFilter<geometry_msgs::PointStamped>(point_sub, tf_, target_frame, 100);
  tf_filter->registerCallback( boost::bind(&DataLabelingOnePerson::pointTransformCallback, this, _1) );
  markers_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 20);
  // Matrix initialization
  this->initialize_matrix();
}
/*
 * Laser Callback
 */
void DataLabelingOnePerson::scanCallback(const sensor_msgs::LaserScan& scan){
	scan_data = scan;
}

/*
 * KIO RTLS Callback
 */
void DataLabelingOnePerson::pointTransformCallback(const boost::shared_ptr<const geometry_msgs::PointStamped>& point_kio){
    geometry_msgs::PointStamped point_person;
    try 
    {
      tf_.transformPoint(target_frame, *point_kio, point_person);
      // Matrix initialization
      this->initialize_matrix();
      vector_points.clear();
      this->classify_scan_data(scan_data, point_person);
    }
    catch (tf::TransformException &ex) {
      printf ("Failure %s\n", ex.what()); 
    }
}

/*
 * Method that save
 * In the matrix_raw, like 1 all the points of the laser, like 0 the rest
 * In the matrix_label with 1s the points of the laser that compose a person
 */
void DataLabelingOnePerson::classify_scan_data(sensor_msgs::LaserScan scan_info , geometry_msgs::PointStamped point_person){
    std::vector<float> vector_scan = scan_info.ranges;
    geometry_msgs::Point point_laser;
    bool in_range = false;
    int j, k, count_points_label = 0;

    for (int i= 0; i < vector_scan.size(); i++){
      j = k = 0;
      // Is the range is good
      if (this->is_good_value(vector_scan[i]) == true){
        // Calculate the xy point from the angle/range
        point_laser = this->get_point_xy(vector_scan[i],i);

        // Calculate if the point is in range of the person
        in_range = this-> is_in_range(point_person, point_laser);

        // Calculate the position of the point in the matrix
        this->get_point_in_matrix(point_laser,&j,&k);
        // If the point is in range it is write in both matrix
        // if not only in the matrix_raw
        if(j>=0 and j<LENGTH_MATRIX and k>=0 and k<LENGTH_MATRIX){
          if (in_range == true){
            matrix_raw[j][k] = 1;
            matrix_label[j][k] = 1;
          }else{
            matrix_raw[j][k] = 1;           
          } 
        }
      }
    }

    // Publish points to visualize in Rviz
    this->publish_points(vector_points, scan_info.header.stamp);

    count_points_label = this->remove_lines(&matrix_label[0]);

    // Save the data and publish it if the points of the label image are more than 5
    if(count_points_label > 5 ){
      data_labeling::DataLabelingBW data;
      data.data_raw = this->matrix_to_vector(matrix_raw);
      data.data_label = this->matrix_to_vector(matrix_label);
      pub_data.publish(data);
    }
}

/*
 * Method that remove the lines of the matrix that receives as parameter
 * Return the number of points color un white in the matrix received after remove the lines
 */
int DataLabelingOnePerson::remove_lines(int matrix_input[LENGTH_MATRIX][LENGTH_MATRIX]){
    int count_points_label=0;
    cv::Mat src, dst;
    // Create a cv Mat with the information of the matrix
    src.create( LENGTH_MATRIX, LENGTH_MATRIX, CV_8UC1);
    for(int i = 0; i < LENGTH_MATRIX; i++){
      for(int j = 0; j < LENGTH_MATRIX; j++){
        if(matrix_input[i][j] == 1){
          src.at<uchar>(Point(i,j)) = 255;
        }else{
          src.at<uchar>(Point(i,j)) = 0;
        }
      }
    }  

    // Apply to the matrix the edge detection algorithm, Canny
    Canny(src, dst, 50, 200, 3);
    vector<Vec4i> lines;
    // dst -> Output of the edge detector
    // lines -> A vector that will store the parameters (x_{start}, y_{start}, x_{end}, y_{end}) of the detected lines
    // 1-> The resolution of the parameter r in pixels
    // CV_PI/360 -> the resolution in degrees
    // 15 -> the minimum number of intersections to “detect” a line
    // 10 -> The minimum number of points that can form a line. Lines with less than this number of points are disregarded.
    // 10 -> The maximum gap between two points to be considered in the same line.
    HoughLinesP(dst, lines, 1, CV_PI/360, 15, 10, 5 );
    for( size_t i = 0; i < lines.size(); i++ ){
       Vec4i l = lines[i];
       line(src, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0), 10, CV_AA);
    }

    // Rewrite the points in the matrix receives 
    for(int i = 0; i < LENGTH_MATRIX; i++){
      for(int j = 0; j < LENGTH_MATRIX; j++){
        if(src.at<uchar>(Point(i,j)) == 255){
          matrix_input[i][j] = 1;
          count_points_label++;
        }else{
           matrix_input[i][j] = 0;
        }
        
      }
    }  

    return count_points_label;
}

/*
 * Method that calculate if the point of the laser is in range of the person
 * Returns true if it is in range if not it return false
 */
bool DataLabelingOnePerson::is_in_range(geometry_msgs::PointStamped point_person, geometry_msgs::Point point_laser){
  float distancia_euclidea = 0;

  // Calculate de euclidean distance between point of person and the point of laser
  distancia_euclidea = sqrt(pow(point_laser.x - point_person.point.x , 2) +pow(point_laser.y - point_person.point.y , 2));
  
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
geometry_msgs::Point DataLabelingOnePerson::get_point_xy(float range, int index){
  
  geometry_msgs::Point point_xy;
  float polar_d = range;
  float polar_angle = 0;
  float polar_angle_radians = 0;
  float cartesian_x = 0;
  float cartesian_y = 0;
  float alfa_radians = 0;

  // alfa is the complementary angle in the polar coordinates
  alfa_radians = (scan_data.angle_increment * (index + 1)) + scan_data.angle_min;

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
void DataLabelingOnePerson::get_point_in_matrix(geometry_msgs::Point point_laser, int *i, int *j){
  int half_matrix = LENGTH_MATRIX / 2;
  // transform the values to cm and divided it by 2 to do the transformation to matrix
  int point_x = floor((point_laser.x * 100)/2);
  int point_y = floor((point_laser.y * 100)/2);  

  *i = point_x;
  *j = half_matrix - point_y;
}

/*
 * Method that transform the matrix recived to a vector and return it
 */
std::vector<int> DataLabelingOnePerson::matrix_to_vector(int matriz[LENGTH_MATRIX][LENGTH_MATRIX]){
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
void DataLabelingOnePerson::initialize_matrix(){
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
bool DataLabelingOnePerson::is_good_value(float num){
  
  bool good_value = true;
  if (isnan(num) || isnan(-num)){
    good_value = false;
  }

  if (isinf(num) || isinf(-num)){
    good_value = false;
  }

  // If the value is greater than 5.12m, we ignore it
  if (num > 5.12){
    good_value = false;
  }

  return good_value;
}
/*
 * Method that publish the points containing in the vector that receives like parameter in the
 * topic /visualization_marker
 */
void DataLabelingOnePerson::publish_points(std::vector<geometry_msgs::Point> vector_points, ros::Time time){
  for (int i = 0; i < vector_points.size(); i++){
    visualization_msgs::Marker m;
        m.header.stamp = time;
        m.header.frame_id = target_frame;
        m.ns = "POINT";
        m.id = i;
        m.type = m.SPHERE;
        m.pose.position.x = vector_points[i].x;
        m.pose.position.y = vector_points[i].y;

        m.scale.x = 0.05;
        m.scale.y = 0.05;
        m.scale.z = 0.05;
        m.color.a = 1;
        m.lifetime = ros::Duration(0.5);
        m.color.b = 1;
        markers_pub.publish(m);
  }
}
