#include <csignal>
#include <stdio.h>
#include <string>
#include <time.h>
#include <algorithm>
#include <vector>

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

//#include "strawberryLocationRos/Segmentor.hpp"
#include "strawberryLocationRos/Camera.hpp"

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;
// sfm::Segmentor* seg;
sfm::Camera* mCamera;
ros::Publisher sb_pub;
geometry_msgs::PoseArray sbLocationsPoses;

cv::Mat image, depth;

// cv::Mat K = (cv::Mat_<double>(3,3) << 614.9827, 0, 320, 0, 614.6468, 240, 0, 0, 1);

void onMouse(int event, int x, int y, int, void*)
{
	if ( event != EVENT_LBUTTONDOWN )
		return;

	if(image.empty())
		return;

  // get the window of clicked point
  int regS = 10;
  cv::Rect region(x-regS/2,y-regS/2,regS,regS);
  cv::Mat depthROI = depth(region).clone();

  // get the depth from the region
  vector<double> D;
  for (int i = 0; i < depthROI.total(); ++i)
  {
      double tmpD =  (double)depthROI.at<ushort>(i);
      if (tmpD > 500 && tmpD < 1500)
      {
          D.push_back(tmpD);
      }

  }

	double finalDepth = 0;
	if(D.size() > 0)
	{
		sort(D.begin(), D.end());
	 	finalDepth = D[(int)D.size()/2];
	}
  cout<<"The final depth is "<<finalDepth<<endl;

  // cv::Mat mouseClickPoint = (cv::Mat_<double>(3,1)<<x,y,1);

	vector<cv::KeyPoint> kp_in, kp_out;
	cv::KeyPoint p_in(x,y,1);
	kp_in.push_back(p_in);

	mCamera->undistortPoints(mCamera, kp_in, kp_out);

	cv::Mat pc_undistort = (cv::Mat_<double>(3,1) << kp_out[0].pt.x, kp_out[0].pt.y, 1);
  cv::Mat pc = finalDepth * mCamera->mK.inv() * pc_undistort;

  cout<<"Final p_cam = "<<pc<<endl;

  cv::Mat drawImg = image.clone();
  cv::rectangle(drawImg, region.tl(), region.br(), Scalar(255, 0, 0), 3, 8, 0);

  cv::imshow("Selected depth Region", drawImg);
  cv::waitKey(1);

  sbLocationsPoses.poses.clear(); // Clear last block perception result
  sbLocationsPoses.header.stamp = ros::Time::now();
  sbLocationsPoses.header.frame_id = "/camera";

  geometry_msgs::Pose pose;
  pose.position.x = pc.at<double>(0);
  pose.position.y = pc.at<double>(1);
  pose.position.z = pc.at<double>(2);

  sbLocationsPoses.poses.push_back(pose);
  sb_pub.publish(sbLocationsPoses);

  return;
}

void callback(const ImageConstPtr& imageMsg, const ImageConstPtr& depthMsg)
{
  cv::Mat img  =  cv_bridge::toCvShare(imageMsg, "bgr8")->image;
  image = img.clone();

  cv::Mat dep = cv_bridge::toCvShare(depthMsg,"16UC1")->image;
  depth = dep.clone();

  cv::imshow("image",image);
  cv::waitKey(1000);
  setMouseCallback( "image", onMouse, 0);
}

int main(int argc, char **argv)
{

    mCamera = new sfm::Camera(argv[1]);

    ros::init(argc,argv, "image_listener");
    ros::NodeHandle nh;
    cv::namedWindow("image",1);
		cv::namedWindow("Selected depth Region",1);
    cv::startWindowThread();

    sb_pub = nh.advertise<geometry_msgs::PoseArray>("/StrawberryPoses", 1);

    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "camera/image", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "camera/depth",1);
    typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(5), image_sub, depth_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();
    cv::destroyWindow("image");
		cv::destroyWindow("Selected depth Region");
    return 0;
}
