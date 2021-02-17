#ifndef EVALUATION_HH
#define EVALUATION_HH

#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "petra/People.h"
#include "petra/Person.h"
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/PointStamped.h"
#include <fstream>
#include "nav_msgs/Odometry.h"


namespace evaluation{

  class Evaluation{

    public:
        Evaluation(ros::NodeHandle nh);
        ~Evaluation(){} 
        void print_data();

    private:
        ros::NodeHandle private_nh_;
        message_filters::Subscriber<petra::People> people_sub;
        message_filters::Subscriber<nav_msgs::Odometry> odom_sub;

        tf::TransformListener tf_;
        tf::MessageFilter<petra::People> * tf_filter;
        tf::MessageFilter<nav_msgs::Odometry> * tf_filter_odom;

        std::string target_frame;
        std::vector<std::vector<double> > vector_map_positions;
        std::vector<std::vector<int> > vector_rosbags_positions;
        std::string rosbag;
        std::vector<int> position_people_rosbag;
        double threshold;
        std::string method;
        std::string csv_directory;
        std::vector<std::vector<std::vector<double> > > output_position;

        geometry_msgs::PointStamped robot_position;
 
        void pointTransformCallback(const boost::shared_ptr<const petra::People>& people);
        void odomTransformCallback(const boost::shared_ptr<const nav_msgs::Odometry>& odom);
        std::string get_bag_name();
        float euclideanDistance(double x1, double y1, double x2, double y2);

  };
};

#endif