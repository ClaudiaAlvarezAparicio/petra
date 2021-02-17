#include <petra/processImage.h>

using namespace processimage;

std::vector<petra::Leg> ProcessImage::getLegsFromImage(cv::Mat image, std::string frame){
  std::vector<petra::Leg> vectorLegs;


  // Prepare the image for findContours
  image.convertTo(image,CV_8UC1);
  cv::threshold( image, image, 200, 255, CV_THRESH_BINARY);


  //int dilate = 3;
  //Mat element_dilate = getStructuringElement(cv::MORPH_CROSS,
  //           cv::Size(2 * dilate + 1, 2 * dilate + 1),
  //           cv::Point(dilate, dilate));

  // Apply dilation on the image
  //cv::dilate(image,image,element_dilate);

  //Find the contours of the image
  std::vector<std::vector<cv::Point> > contours;
  cv::findContours( image, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE );

  // Add info to the leg message
  // Get the moments
  std::vector<Moments> mu(contours.size() );
  for( size_t i = 0; i < contours.size(); i++ ){
      mu[i] = moments( contours[i] );
  }

  std::vector<Point2f> mc( contours.size() );
  // Get the centroids
  for( size_t i = 0; i < contours.size(); i++ ){
    geometry_msgs::Point centerFrame;
    petra::Leg leg;
    //add 1e-5 to avoid division by zero
    mc[i] = Point2f( static_cast<float>(mu[i].m10 / (mu[i].m00 + 1e-5)),
                     static_cast<float>(mu[i].m01 / (mu[i].m00 + 1e-5)) );
    centerFrame = this->getPointInFrameHokuyo(mc[i].x, mc[i].y);
    leg.header.frame_id = frame;
    leg.name = "leg";
    leg.position.x = centerFrame.x;
    leg.position.y = centerFrame.y;

    vectorLegs.push_back(leg);
  }
  return vectorLegs;
}

geometry_msgs::Point ProcessImage::getPointInFrameHokuyo(float pixelX, float pixelY){
  geometry_msgs::Point pointFrame;
  int halfMatrix = LENGTH_MATRIX / 2;
  float x,y = 0.0;
  x = pixelY;
  y = pixelX - halfMatrix;
  x = (x * 2) / 100.0;
  y = - (y * 2) / 100.0;

  pointFrame.x = x;
  pointFrame.y = y;

  return pointFrame;
}
