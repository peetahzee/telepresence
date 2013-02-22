#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <cv_bridge/cv_bridge.h>
#include "usc_mrp/SetUserView.h"
#include <string>

image_transport::Publisher pub_user;
image_transport::Publisher pub_original;
image_transport::Publisher pub_crop1;
image_transport::Publisher pub_crop2;

std::string userView = "original";

int cropLevels[2] = { 50, 100 };

bool setUserView(usc_mrp::SetUserView::Request &req, usc_mrp::SetUserView::Response &res) {
  // ros::param::set(req.topicName, req.value);
  userView = req.viewName.c_str();
  ROS_INFO("User view set to [%s].", req.viewName.c_str());
}

void publishCvMat(image_transport::Publisher pub, cv_bridge::CvImagePtr origPtr, cv::Mat mat) {
  cv_bridge::CvImage out_msg;
  
  out_msg.header = origPtr->header;
  out_msg.encoding = "bgr8";
  out_msg.image = mat;

  pub.publish(out_msg.toImageMsg());
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  pub_original.publish(cv_ptr->toImageMsg());
  pub_user.publish(cv_ptr->toImageMsg());
  //cv::imshow("view", cv_ptr->image);

  cv::Mat image(cv_ptr->image);
  cv::Mat croppedImage = image(cv::Rect(cropLevels[0], cropLevels[0], 640 - cropLevels[0] * 2, 480 - cropLevels[0] * 2));
  publishCvMat(pub_crop1, cv_ptr, croppedImage);
  //cv::imshow("new view", croppedImage);

  croppedImage = image(cv::Rect(cropLevels[1], cropLevels[1], 640 - cropLevels[1] * 2, 480 - cropLevels[1] * 2));
  publishCvMat(pub_crop2, cv_ptr, croppedImage);
  //cv::imshow("new view 2", croppedImage);  
  
  //cv::waitKey(3);

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("/usc_mrp/setUserView", setUserView);
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_color", 1, imageCallback);

  pub_user = it.advertise("usc_mrp/camera/user", 1);
  pub_original = it.advertise("usc_mrp/camera/original", 1);
  pub_crop1 = it.advertise("usc_mrp/camera/crop1", 1);
  pub_crop2 = it.advertise("usc_mrp/camera/crop2", 1);

  ros::spin();
  
  /*
  cv::namedWindow("view");
  cv::namedWindow("new view");
  cv::namedWindow("new view 2");
  cv::startWindowThread();
  cv::destroyWindow("view");
  cv::destroyWindow("new view");
  cv::destroyWindow("new view 2");
  */
}