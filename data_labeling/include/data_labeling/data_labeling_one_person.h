#ifndef DATA_LABELING_ONE_PERSON_HH
#define DATA_LABELING_ONE_PERSON_HH

#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "sensor_msgs/LaserScan.h"
#include "data_labeling/DataLabelingBW.h"
#include <visualization_msgs/Marker.h>


#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#define LENGTH_MATRIX 256

namespace data_labeling_one_person{

  class DataLabelingOnePerson{

    public:
        DataLabelingOnePerson(ros::NodeHandle nh);
        ~DataLabelingOnePerson(){} 

    private:
        ros::NodeHandle private_nh_;
        message_filters::Subscriber<geometry_msgs::PointStamped> point_sub;
        ros::Subscriber scan_sub;
        ros::Publisher pub_data;
        ros::Publisher markers_pub;
        tf::TransformListener tf_;
        tf::MessageFilter<geometry_msgs::PointStamped> * tf_filter;
        sensor_msgs::LaserScan scan_data;
        std::string target_frame;
        float range_person;
        int matrix_raw[LENGTH_MATRIX][LENGTH_MATRIX];
        int matrix_label[LENGTH_MATRIX][LENGTH_MATRIX];
        std::vector<geometry_msgs::Point> vector_points;    

        void scanCallback(const sensor_msgs::LaserScan& scan);
        void pointTransformCallback(const boost::shared_ptr<const geometry_msgs::PointStamped>& point_kio);
        void classify_scan_data(sensor_msgs::LaserScan scan_info, geometry_msgs::PointStamped point_person);
        bool is_in_range(geometry_msgs::PointStamped point_person, geometry_msgs::Point point_laser);
        geometry_msgs::Point get_point_xy(float range, int index);
        void get_point_in_matrix(geometry_msgs::Point point_laser, int *i, int *j);
        void initialize_matrix();
        std::vector<int> matrix_to_vector(int matriz[LENGTH_MATRIX][LENGTH_MATRIX]);
        bool is_good_value(float num);
        void publish_points(std::vector<geometry_msgs::Point> vector_points, ros::Time time);
        int remove_lines(int [LENGTH_MATRIX][LENGTH_MATRIX]); 
  };
};

#endif