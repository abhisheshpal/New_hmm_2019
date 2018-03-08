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
int quantity=0;

cv::Mat image, depth;
  Mat imgOriginal;
  Mat imgCircles;
// cv::Mat K = (cv::Mat_<double>(3,3) << 614.9827, 0, 320, 0, 614.6468, 240, 0, 0, 1);

void coordinate_conversion(int x, int y, int area)
{
  // get the window of clicked point
  int regS = 7;
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
   cout<<"The final depth is "<<finalDepth<<"test"<<endl;

  // cv::Mat mouseClickPoint = (cv::Mat_<double>(3,1)<<x,y,1);

	vector<cv::KeyPoint> kp_in, kp_out;
	cv::KeyPoint p_in(x,y,1);
	kp_in.push_back(p_in);

	mCamera->undistortPoints(mCamera, kp_in, kp_out);

	cv::Mat pc_undistort = (cv::Mat_<double>(3,1) << kp_out[0].pt.x, kp_out[0].pt.y, 1);
  cv::Mat pc = finalDepth * mCamera->mK.inv() * pc_undistort;

  

  // cv::Mat drawImg = image.clone();
  // cv::rectangle(drawImg, region.tl(), region.br(), Scalar(255, 0, 0), 3, 8, 0);

  // cv::imshow("Selected depth Region", drawImg);
  cv::waitKey(1);


 if (pc.at<double>(2)>0&&pc.at<double>(2)<740/*&&pc.at<double>(2)<850*/)
 {
  cout<<"Final p_cam = "<<pc<<endl;
  quantity++;
  float cut_position=0;
  float Ds=pc.at<double>(2)*area*5E-05+18.854;
  if (Ds>20){
  geometry_msgs::Pose pose;
  pose.position.x = pc.at<double>(0);
  pose.position.y = pc.at<double>(1);
  pose.position.z = pc.at<double>(2);
  pose.orientation.y = cut_position;
  pose.orientation.z = Ds*1.1;
  sbLocationsPoses.poses.push_back(pose);
          Point CircleCenter;
        CircleCenter=Point(x,y);
        int Radius;
        Scalar Color; //Scalar is widely used in opencv for passing pixel values.
        int Thickness;
        int Shift;

        Radius=10;
        Color=CV_RGB(0,0,255);
        Thickness=3;
        Shift = 0;
   
        circle(imgCircles,CircleCenter,Radius,Color,Thickness,CV_AA,Shift); 
        imgOriginal = imgOriginal + imgCircles;
        imshow("viewQ", imgOriginal);
          }
}
}

void callback(const ImageConstPtr& imageMsg, const ImageConstPtr& depthMsg)
{
    Mat img  =  cv_bridge::toCvShare(imageMsg, "bgr8")->image;
  image = img.clone();

    Mat dep = cv_bridge::toCvShare(depthMsg,"16UC1")->image;
  depth = dep.clone();
    
  
   // Mat imgHSV;

    imgOriginal=cv_bridge::toCvShare(imageMsg, "bgr8")->image;
   // cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
 
    Mat imgThresholded;
    imgCircles = Mat::zeros( imgOriginal.size(), CV_8UC3 );;

 
    inRange(imgOriginal, Scalar(0, 0, 10), Scalar(10, 20, 60), imgThresholded); 

    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(3,3)) );
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)) ); 

     //morphological closing (removes small holes from the foreground)
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(6, 6)) ); 
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(6, 6)) );

    imshow("view", imgThresholded);

    vector<vector<Point> > contours;
    findContours(imgThresholded,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
    int ctsSum = contours.size();  
    int pixelx;
    int pixely;

     for(int i=0;i<ctsSum;i++){  
        Moments mom = moments(Mat(contours[i])); 
        pixelx=int(mom.m10/mom.m00);
        pixely=int(mom.m01/mom.m00);

        if ((pixelx>10&&pixelx<580)&&(pixely>10&&pixely<440)&&mom.m00>100)//||(pixelx>=120&&pixelx<180&&pixely>250&&pixely<440)&&mom.m00>50)
        {
              printf ("area=%f\n", mom.m00);

        coordinate_conversion(pixelx, pixely, mom.m00);
          }
        }  
     if (quantity!=0)
  {
   sbLocationsPoses.header.stamp = ros::Time::now();
  sbLocationsPoses.header.frame_id = "/camera";

  sb_pub.publish(sbLocationsPoses);
  quantity=0;
    sbLocationsPoses.poses.clear(); // Clear last block perception result

  }

  //cv::imshow("image",image);
  cv::waitKey(1000);
 // setMouseCallback( "image", onMouse, 0);
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
