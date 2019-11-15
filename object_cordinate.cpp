#include <ros/ros.h>
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
#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>

using namespace std;
using namespace cv;
using namespace std;

bool first_check = false;

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
float depthSI;

vector<int> vec;
vector<int> depth_vec;

bool detected;

void Callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
    //cout<<"==========start==========="<<endl;
    for(int i=0;i<msg->bounding_boxes.size();i++){

	    if(msg->bounding_boxes[i].Class == "mouse"){
		    //cout<<"Bouding Boxes (header):" << msg->header <<endl;
		    //cout<<"Bouding Boxes (image_header):" << msg->image_header <<endl;
		   // cout<<"Bouding Boxes (Class):" << msg->bounding_boxes[i].Class <<endl;
		   // cout<<"Bouding Boxes (xmin):" << msg->bounding_boxes[i].xmin <<endl;
		   // cout<<"Bouding Boxes (xmax):" << msg->bounding_boxes[i].xmax <<endl;
		  //  cout<<"Bouding Boxes (ymin):" << msg->bounding_boxes[i].ymin <<endl;
		  //  cout<<"Bouding Boxes (ymax):" << msg->bounding_boxes[i].ymax <<endl;
        xmin = msg->bounding_boxes[i].xmin;
        xmax = msg->bounding_boxes[i].xmax;
        ymin = msg->bounding_boxes[i].ymin;
        ymax = msg->bounding_boxes[i].ymax;

        x_center = (xmin+xmax)/2;
        y_center = (ymin+ymax)/2;
        detected_num += 1;
		  }

    }

    if(detected_num > 0){
      detected = true;
    }
    else{
      detected = false;
    }
    detected_num = 0;
    //cout<<"==========end==========="<<endl;
}


void Depth_Image_Callback(const sensor_msgs::ImageConstPtr& msg)
{

}



void callback(const sensor_msgs::CameraInfo::ConstPtr& mSPtrCameraInfo, const sensor_msgs::Image::ConstPtr& msg)
{
  if(detected){
    cv_bridge::CvImagePtr cv_depth_ptr;
    try
    {
      cv_depth_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);//now cv_ptr is the matrix, do not forget "TYPE_" before "16UC1"
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Print out the depth information
    depth_vec.clear();
    for(int i=-2; i < 3; ++i){
      for(int j=-2; j < 3; ++j){
        depth = cv_depth_ptr->image.at<short int>(cv::Point(x_center+i,y_center+j));//you can change 240,320 to your interested pixel
        depth_vec.push_back(depth);
      }
    }

    depths = accumulate( depth_vec.begin(), depth_vec.end(), 0.0f )/ depth_vec.size();

    vec.push_back(depths);
    if(vec.size() == 50){
      vec.erase(vec.begin());
    }
    else{
      depth_mean = accumulate( vec.begin(), vec.end(), 0.0f )/ vec.size();
    }
    if(depth != 0 && depth_mean != 0){
      depthSI = depth * 0.001f;

      auto x = (x_center - mSPtrCameraInfo->K.at(2)) / mSPtrCameraInfo->K.at(0);
      auto y = (y_center - mSPtrCameraInfo->K.at(5)) / mSPtrCameraInfo->K.at(4);

      x_cor = depthSI * x;
      y_cor = depthSI * y;

      }
    ROS_INFO("x: %f", x_cor);
    ROS_INFO("y: %f", y_cor);
    ROS_INFO("z: %f", depthSI);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    transform.setOrigin( tf::Vector3(-x_cor*5, -depthSI*2, -y_cor*5) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "mouse", "camera"));
  }
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "sub1");
  ros::NodeHandle nh;
  first_check = true;
  message_filters::Subscriber<sensor_msgs::CameraInfo> image_sub(nh, "/camera/depth/camera_info", 10);
  message_filters::Subscriber<sensor_msgs::Image> info_sub(nh, "/camera/depth/image_rect_raw", 1);
  ros::Subscriber sub = nh.subscribe("/darknet_ros/bounding_boxes", 50, Callback);
  ros::Subscriber sub_ = nh.subscribe("/camera/depth/image_rect_raw", 100, Depth_Image_Callback);
  message_filters::TimeSynchronizer<sensor_msgs::CameraInfo, sensor_msgs::Image> sync(image_sub, info_sub, 20);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
  ros::Rate loop_rate(30);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("/camera/visualization_marker", 10);
  while (ros::ok())
    {
      visualization_msgs::Marker points, line_strip, line_list;
      points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/camera_link";
      points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
      points.ns = line_strip.ns = line_list.ns = "points_and_lines";
      points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
      points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

      points.id = 0;
      line_strip.id = 1;
      line_list.id = 2;

      points.type = visualization_msgs::Marker::POINTS;
      line_strip.type = visualization_msgs::Marker::LINE_STRIP;
      line_list.type = visualization_msgs::Marker::LINE_LIST;
  // %EndTag(TYPE)%

  // %Tag(SCALE)%
      // POINTS markers use x and y scale for width/height respectively
      points.scale.x = 0.1;
      points.scale.y = 0.1;

      // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
      line_strip.scale.x = 0.1;
      line_list.scale.x = 0.1;
  // %EndTag(SCALE)%

  // %Tag(COLOR)%
      // Points are green
      points.color.g = 1.0f;
      points.color.a = 1.0;


      geometry_msgs::Point p;
      p.x = x_cor;
      p.y = depthSI;
      p.z = y_cor;
      points.points.push_back(p);
      //if(depth != 0){
      marker_pub.publish(points);
      //}
      
      ros::spinOnce();

      loop_rate.sleep();
      
    }

  ros::spin();
  return 0;
}
