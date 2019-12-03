#include "object_cordinate.h"

using namespace std;


obj_cor::obj_cor(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    ROS_INFO("in class constructor of ExampleRosClass");
    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    initializePublishers();
    detected = false;
    
    // can also do tests/waits to make sure all required services, topics, etc are alive
}

void obj_cor::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    bb_sub = nh_.subscribe("/darknet_ros/bounding_boxes", 1, &obj_cor::bb_callback,this);
    image_sub = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_, "/camera/depth/camera_info", 10);
    info_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh_, "/camera/depth/image_rect_raw", 1);
    sync = new message_filters::TimeSynchronizer<sensor_msgs::CameraInfo, sensor_msgs::Image>(*image_sub, *info_sub, 20);
    sync->registerCallback(boost::bind(&obj_cor::depth_callback, this, _1, _2));  

}

void obj_cor::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    marker_pub = nh_.advertise<visualization_msgs::Marker>("/camera/visualization_marker", 10);
}

void obj_cor::bb_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
    for(int i=0;i<msg->bounding_boxes.size();i++){

	    if(msg->bounding_boxes[i].Class == "bottle"){

        xmin = msg->bounding_boxes[i].xmin;
        xmax = msg->bounding_boxes[i].xmax;
        ymin = msg->bounding_boxes[i].ymin;
        ymax = msg->bounding_boxes[i].ymax;
        x_cor = msg->bounding_boxes[i].x_cor;
        y_cor = msg->bounding_boxes[i].y_cor;
        z_cor = msg->bounding_boxes[i].z_cor;

        ROS_INFO("x: %f", x_cor);
        ROS_INFO("y: %f", y_cor);
        ROS_INFO("z: %f", z_cor);

        static tf::TransformBroadcaster br;
        tf::Transform transform;
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
        transform.setOrigin( tf::Vector3(-x_cor*5, -z_cor*2, y_cor*5) );
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "bottle", "odom"));  
    
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

}


void obj_cor::depth_callback(const sensor_msgs::CameraInfo::ConstPtr& mSPtrCameraInfo, const sensor_msgs::Image::ConstPtr& msg)
{
  if(detected){
    // cv_bridge::CvImagePtr cv_depth_        ptr;
    // try
    // {
    //   cv_depth_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);//now cv_ptr is the matrix, do not forget "TYPE_" before "16UC1"
    // }
    // catch (cv_bridge::Exception& e)
    // {
    //   ROS_ERROR("cv_bridge exception: %s", e.what());
    //   return;
    // }

    // // Print out the depth information
    // depth_vec.clear();
    // if(x_center != 0 && y_center != 0){
    //   for(int i=-2; i < 3; ++i){
    //     for(int j=-2; j < 3; ++j){
    //       depth = cv_depth_ptr->image.at<short int>(cv::Point(x_center+i,y_center+j));//you can change 240,320 to your interested pixel
    //       depth_vec.push_back(depth);
    //     }
    //   }
    // }
    
    // depths = std::accumulate( depth_vec.begin(), depth_vec.end(), 0.0f )/ depth_vec.size();

    // vec.push_back(depths);
    // if(vec.size() == 50){
    //   vec.erase(vec.begin());
    // }
    // else{
    //   depth_mean = std::accumulate( vec.begin(), vec.end(), 0.0f )/ vec.size();
    // }
    // if(depth != 0 && depth_mean != 0){
    //   depthSI = depth * 0.001f;

    //   x_cor = depthSI * (x_center - mSPtrCameraInfo->K.at(2)) / mSPtrCameraInfo->K.at(0);
    //   y_cor = depthSI * (y_center - mSPtrCameraInfo->K.at(5)) / mSPtrCameraInfo->K.at(4);

    //   }
 

  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sub1");
  ros::NodeHandle nh;

  obj_cor oc(&nh);

  ros::spin();
}
