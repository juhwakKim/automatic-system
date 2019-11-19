#include "compressed.h"

compress::compress(image_transport::ImageTransport* IT):it_(*IT)
{ // constructor
    ROS_INFO("in class constructor of ExampleRosClass");
    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    initializePublishers();

    
    // can also do tests/waits to make sure all required services, topics, etc are alive
}

void compress::imageCallback(const sensor_msgs::ImageConstPtr& msg_)
{
  try
  {
      cv::Mat detection_image = cv_bridge::toCvShare(msg_, "bgr8")->image;
      cv_bridge::CvImage cvImage;
      cvImage.header.stamp = ros::Time::now();
      cvImage.header.frame_id = "detection_images";
      cvImage.encoding = sensor_msgs::image_encodings::BGR8;
      cvImage.image = detection_image;
      ros::Rate loop_rate(30);
      while (ros::ok()) {
        pub.publish(*cvImage.toImageMsg());
        ros::spinOnce();
        loop_rate.sleep();
      }
      ROS_INFO("[YoloObjectDetector] Node started.");

  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg_->encoding.c_str());
  }
}

void compress::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    sub = it_.subscribe("darknet_ros/detection_image", 1, &compress::imageCallback,this);  
    // add more subscribers here, as needed
}

void compress::initializePublishers()
{
  ROS_INFO("Initializing Publishers");
  pub = it_.advertise("darknet_ros/img_", 1);
  
}
int main (int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    compress com_obj(&it);
    ros::spin();
}
    
