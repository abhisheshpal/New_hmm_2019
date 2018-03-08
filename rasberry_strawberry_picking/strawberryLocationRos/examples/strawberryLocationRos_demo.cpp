#include <csignal>
#include <stdio.h>
#include <string>
#include <time.h>

// #include <image_transport/image_transport.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/ml.hpp"
#include "opencv2/opencv.hpp"

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseArray.h>

#include "strawberryLocationRos/Segmentor.hpp"
#include "strawberryLocationRos/Camera.hpp"

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

sfm::Segmentor* seg;
sfm::Camera* mCamera;
ros::Publisher sb_pub;
geometry_msgs::PoseArray sbLocationsPoses;

void callback(const ImageConstPtr& imageMsg, const ImageConstPtr& depthMsg)
{
  cv::Mat image =  cv_bridge::toCvShare(imageMsg, "bgr8")->image;
  cv::imshow("image",image);
  cv::waitKey(1);

  cv::Mat depth = cv_bridge::toCvShare(depthMsg,"16UC1")->image;

  // detect the strawberry here
  vector<cv::Rect> sbs;
  seg->computingBoundingBox(image,sbs);

  // get 3D position of the strawberry
  vector<cv::Point3d> sbLocations;
  seg->getStrawberryLocation(depth,sbs,mCamera,sbLocations);

  cv::Mat imageSB = image.clone();
  if(sbs.size() > 0)
  {
    for(int i = 0 ; i < sbs.size(); ++i)
    {
      cv::Rect roi = sbs[i];
      cv::rectangle(imageSB, roi.tl(), roi.br(), cv::Scalar(255, 0, 0), 3, 8, 0);
    }
  }
  cv::imshow("Detected Strawberries",imageSB);
  cv::waitKey(1);

  //convert the strawberry locactions to PoseArray msgs
  sbLocationsPoses.poses.clear(); // Clear last block perception result
  sbLocationsPoses.header.stamp = ros::Time::now();
  sbLocationsPoses.header.frame_id = "/camera";

  if(sbLocations.size() > 0)
  {
    for(int i = 0 ; i < sbLocations.size(); ++i)
    {
      geometry_msgs::Pose pose;
      pose.position.x = sbLocations[i].x;
      pose.position.y = sbLocations[i].y;
      pose.position.z = sbLocations[i].z;

      sbLocationsPoses.poses.push_back(pose);
    }
    sb_pub.publish(sbLocationsPoses);
  }


}



int main(int argc, char **argv)
{
    if(argc != 3)
    {
      cerr<<"./a.out trainFile cameraParamFile"<<endl;
      return 0;
    }

    seg = new sfm::Segmentor(argv[1]);
    mCamera = new sfm::Camera(argv[2]);

    ros::init(argc,argv, "image_listener");
    ros::NodeHandle nh;
    cv::namedWindow("image",1);
    cv::namedWindow("Detected Strawberries",1);
    cv::startWindowThread();

    sb_pub = nh.advertise<geometry_msgs::PoseArray>("/StrawberryPoses", 1000);

    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "camera/image", 10);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "camera/depth",10);
    typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(5), image_sub, depth_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();
    cv::destroyWindow("image");
    cv::destroyWindow("Detected Strawberries");
    return 0;
}
