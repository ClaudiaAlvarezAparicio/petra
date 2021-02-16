#ifndef DATA_LABELING_PETRA_HH
#define DATA_LABELING_PETRA_HH

#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "sensor_msgs/LaserScan.h"
#include <visualization_msgs/Marker.h>
#include "petra/People.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "npy.hpp"

using namespace ros;
using namespace message_filters;

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
        message_filters::Subscriber<petra::People> people_sub;
        message_filters::Subscriber<sensor_msgs::LaserScan>  scan_sub;
        typedef sync_policies::ApproximateTime<petra::People, sensor_msgs::LaserScan> MySyncPolicy;
        typedef Synchronizer<MySyncPolicy> Sync;
        boost::shared_ptr<Sync> sync;

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

        void peopleScanCallback(const petra::PeopleConstPtr& petra, const sensor_msgs::LaserScanConstPtr& scan);
        void classify_scan_data(const sensor_msgs::LaserScanConstPtr& scan_info, geometry_msgs::Point point_person);
        bool is_in_range(geometry_msgs::Point point_person, geometry_msgs::Point point_laser);
        geometry_msgs::Point get_point_xy(float range, int index, const sensor_msgs::LaserScanConstPtr& scan_data);
        void get_point_in_matrix(geometry_msgs::Point point_laser, int *i, int *j);
        void initialize_matrix();
        std::vector<int> matrix_to_vector(int matriz[LENGTH_MATRIX][LENGTH_MATRIX]);
        bool is_good_value(float num);
        std::string get_name_npy_file();
        std::vector<int> vector_to_global_vector(std::vector<std::vector<int> > global_vector);
  };
};

#endif
