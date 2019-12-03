#include <ros/ros.h>
#include <numeric>
#include <std_msgs/String.h>
#include <sstream>
#include <string>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <sensor_msgs/image_encodings.h>
#include <cmath>
#include <vector>
#include <fstream>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>
//#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>



class obj_cor
{
    public:
        obj_cor(ros::NodeHandle* nodehandle);

    private:
        ros::NodeHandle nh_;
        message_filters::Subscriber<sensor_msgs::CameraInfo>* image_sub;
        message_filters::Subscriber<sensor_msgs::Image>* info_sub;
        message_filters::TimeSynchronizer<sensor_msgs::CameraInfo, sensor_msgs::Image>* sync;
        ros::Subscriber bb_sub;

        ros::Publisher marker_pub;
        
        int xmin;
        int xmax;
        int ymin;
        int ymax;

        int x_center;
        int y_center;

        int depth;
        int depths;
        int depth_mean;

        int detected_num;

        float x_cor;
        float y_cor;
        float z_cor;
        float depthSI;

        std::vector<int> vec;
        std::vector<int> depth_vec;

        bool detected;
        
        void initializeSubscribers(); 
        void initializePublishers();

        void depth_callback(const sensor_msgs::CameraInfo::ConstPtr& mSPtrCameraInfo, const sensor_msgs::Image::ConstPtr& msg);
        void bb_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);

};
