#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

class compress
{
  
  public:
    compress(image_transport::ImageTransport* IT);

    
  private:
    image_transport::ImageTransport it_;
    image_transport::Subscriber sub;
    image_transport::Publisher pub;

    void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    void initializePublishers();

    void imageCallback(const sensor_msgs::ImageConstPtr& msg_);

};
