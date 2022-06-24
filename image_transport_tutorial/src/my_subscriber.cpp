#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <string>
#include <iostream>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    std::cout<<msg->data.data()<<std::endl;
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(10);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  // image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
  image_transport::Subscriber sub = it.subscribe("/jhu_daVinci/left/decklink/jhu_daVinci_left/image_raw", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");
}
