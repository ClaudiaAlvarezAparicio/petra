#include <petra/networkPrediction.h>

using namespace networkprediction;

NetworkPrediction::NetworkPrediction(std::string networkModel, std::string outNetworkLayer){
  this->networkModel = networkModel;
  this->outNetworkLayer = outNetworkLayer;

  // Load graph
  this->status = ReadBinaryProto(Env::Default(), ros::package::getPath("petra") + "/model/" + this->networkModel , &graph_def);
  if (!this->status.ok()) {
    std::cout << this->status.ToString() << "\n";
    ros::shutdown();
  }

  // Open Connection
  this->openConnection();
}

NetworkPrediction::~NetworkPrediction(){
  this->closeConnection();
}

void NetworkPrediction::openConnection(){

  this->options = SessionOptions();
  this->options.config.mutable_gpu_options()->set_allow_growth(true);
  this->status = NewSession(this->options, &session);
  if (!this->status.ok()) {
    std::cout << this->status.ToString() << "\n";
    ros::shutdown();
  }

  // Add the graph to the session
  this->status = this->session->Create(this->graph_def);
  if (!this->status.ok()) {
    std::cout << this->status.ToString() << "\n";
    ros::shutdown();
  }
}
void NetworkPrediction::closeConnection(){
  this->session->Close();
  delete this->session;
}

cv::Mat NetworkPrediction::createImage(sensor_msgs::LaserScan scan){
  std::vector<float> vector_scan = scan.ranges;
  cv::Mat image = cv::Mat::zeros(cv::Size(LENGTH_MATRIX,LENGTH_MATRIX), CV_32FC1);
  geometry_msgs::Point point_laser;
  int j, k = 0;
  for (int i= 0; i < vector_scan.size(); i++){
    j, k = 0;
    if (this->isGoodValue(vector_scan[i]) == true){
      // Calculate the xy point from the angle/range
      point_laser = this->getPointXY(vector_scan[i],i, scan.angle_increment, scan.angle_min);
      // Calculate the position of the point in the matrix
      this->getPointInMatrix(point_laser,&j,&k);
      // write the point in the cv::Mat
      if(j>=0 and j<LENGTH_MATRIX and k>=0 and k<LENGTH_MATRIX){
        image.at<float>(j,k) = 1.0;
      }
    }
  }

  // Some operations to get a better predict from the model
  Scalar mean, std;
  cv::meanStdDev(image,mean,std);
  cv::subtract(image,mean,image);
  cv::divide(image, std, image);

  return image;
}

cv::Mat NetworkPrediction::prediction(sensor_msgs::LaserScan scan){

  cv::Mat image= this->createImage(scan);
  cv::normalize(image, image, 0, 255, NORM_MINMAX, CV_32FC1);
  // Create the Tensor with the input shape
  Tensor image_tensor (tensorflow::DT_FLOAT, tensorflow::TensorShape{1,LENGTH_MATRIX,LENGTH_MATRIX,1});
  // Allocate memory
  tensorflow::StringPiece tmp_data = image_tensor.tensor_data();
  memcpy(const_cast<char*>(tmp_data.data()), (image.data), LENGTH_MATRIX * LENGTH_MATRIX * sizeof(float));


  // Define input and output layers
  std::vector<std::pair<string, tensorflow::Tensor>> inputs = {{"input_1", image_tensor}};
  std::vector<Tensor> outputs;
  this->status = session->Run(inputs, {this->outNetworkLayer}, {}, &outputs);

  if (!this->status.ok()) {
     LOG(ERROR) << "Running model failed: " << this->status;
     ros::shutdown();
  }

  cv::Mat image_output(outputs[0].dim_size(1), outputs[0].dim_size(2), CV_32FC1, outputs[0].flat<float>().data());
  cv::normalize(image_output, image_output, 0, 255, NORM_MINMAX, CV_32FC1);
  return image_output;
}


/*
 * Method that determines if the value that recives is not equal to inf, nan, -inf or -nan, and if it is in range
 * Return true if is a good value and false if is not
 */
bool NetworkPrediction::isGoodValue(float num){
  bool goodValue = true;

  if (std::isnan(num) || std::isnan(-num)){
    goodValue = false;
  } else if (std::isinf(num) || std::isinf(-num)){
    goodValue = false;
  } else if (num > 10.24){ // TODO para 5 else if (num > 5.12){
    goodValue = false;
  }

  return goodValue;
}

/*
 * Method that transform the range provided by the laser to polar coordinates
 * Return a Point object with the point in cartesian coordinates (x,y)
 */
geometry_msgs::Point NetworkPrediction::getPointXY(float range, int index, float angleIncrement, float angleMin){

  geometry_msgs::Point pointXY;
  float polarD = range;
  float polarAngle = 0;
  float polarAngleRadians = 0;
  float cartesianX = 0;
  float cartesianY = 0;
  float alfaRadians = 0;

  // alfa is the complementary angle in the polar coordinates
  alfaRadians = (angleIncrement * (index + 1)) + angleMin;

  // Calculate the point (x,y)
  cartesianX = polarD * cos(alfaRadians);
  cartesianY = polarD * sin(alfaRadians);

  // Add the values to the object to return it
  pointXY.x = cartesianX;
  pointXY.y = cartesianY;

  return pointXY;
}

/*
 * Method that calculates the point xy of the laser in a ij position in the matrix
 */
void NetworkPrediction::getPointInMatrix(geometry_msgs::Point pointLaser, int *i, int *j){
  int halfMatrix = LENGTH_MATRIX / 2;
  // transform the values to cm and divided it by 2 to do the transformation to matrix
  int pointX = floor((pointLaser.x * 100)/2);
  int pointY = floor((pointLaser.y * 100)/2);

  *i = pointX;
  *j = halfMatrix - pointY;
}
