#ifndef DATA_LABELING_PETRA_HH
#define DATA_LABELING_PETRA_HH

#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "sensor_msgs/LaserScan.h"
#include "data_labeling/DataLabelingBW.h"
#include <visualization_msgs/Marker.h>
#include "petra/People.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "npy.hpp"

#define LENGTH_MATRIX 256

namespace data_labeling_petra{

  class DataLabelingPetra{

    public:
        DataLabelingPetra(ros::NodeHandle nh);
        ~DataLabelingPetra(){}
        void label_scans();
        void save_npy();

    private:
        ros::NodeHandle private_nh_;
        message_filters::Subscriber<geometry_msgs::PointStamped> point_sub;
        ros::Subscriber scan_sub, petra_sub;

        ros::Publisher pub_data;
        ros::Publisher markers_pub;
        tf::TransformListener tf_;
        tf::MessageFilter<geometry_msgs::PointStamped> * tf_filter;
        std::string target_frame;
        std::string rosbag;
        std::string directory;

        std::string scan_topic;
        
        float range_person;
        int matrix_raw[LENGTH_MATRIX][LENGTH_MATRIX];
        int matrix_label[LENGTH_MATRIX][LENGTH_MATRIX];
        std::vector<geometry_msgs::Point> vector_points;

        std::vector<sensor_msgs::LaserScan> historicScan;
        std::vector<std::vector<int> > global_raw;
        std::vector<std::vector<int> > global_label;

        void scanCallback(const sensor_msgs::LaserScan& scan);
        void petraCallback(const petra::People& petra);
        sensor_msgs::LaserScan getLaserScan(std_msgs::Header header_petra);

        int classify_scan_data(sensor_msgs::LaserScan scan_info, geometry_msgs::Point point_person);
        bool is_in_range(geometry_msgs::Point point_person, geometry_msgs::Point point_laser);
        geometry_msgs::Point get_point_xy(float range, int index, sensor_msgs::LaserScan scan_data);
        void get_point_in_matrix(geometry_msgs::Point point_laser, int *i, int *j);
        void initialize_matrix();
        std::vector<int> matrix_to_vector(int matriz[LENGTH_MATRIX][LENGTH_MATRIX]);
        bool is_good_value(float num);
        void publish_points(std::vector<geometry_msgs::Point> vector_points, ros::Time time);
        std::string get_name_npy_file();
        std::vector<int> vector_to_global_vector(std::vector<std::vector<int> > global_vector);
        int remove_lines(int [LENGTH_MATRIX][LENGTH_MATRIX]); 
  };
};

#endif
