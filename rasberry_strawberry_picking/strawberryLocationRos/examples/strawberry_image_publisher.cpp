#include <csignal>
#include <stdio.h>
#include <string>
#include <time.h>

// include OpenCV header file
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


using namespace std;


int main(int argc, char **argv)
{
  if(argc != 3  )
  {
    cerr<<"./a.out imageFolderName/ depthfolderName/ "<<endl;
    exit(1);
  }

  ros::init(argc,argv,"image_publisher");
  ros::NodeHandle nh;

  // cv::namedWindow("Published_Image");
  // cv::namedWindow("Published_Depth");
  // cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Publisher imgPub = it.advertise("camera/image", 1);
  image_transport::Publisher depthPub = it.advertise("camera/depth",1);
  sensor_msgs::ImagePtr imgMsg ,depthMsg;
  ros::Rate loop_rate(10);


  string imagefolderName = string(argv[1]);
  string depthfolderName = string(argv[2]);
  
  cv::Mat image,depth;
  int numImg = 0;
  while (nh.ok() && numImg < 100)
  {
    string imageName = imagefolderName + to_string(numImg) + ".jpg";
    string depthName = depthfolderName + to_string(numImg) + ".png";
    cout<<imageName<<endl;
    image = cv::imread(imageName);
    depth = cv::imread(depthName);
    
    //cv::imshow("view",image);
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.seq = numImg;

    imgMsg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
    depthMsg = cv_bridge::CvImage(header, "16UC1", depth).toImageMsg();

    imgPub.publish(imgMsg);
    depthPub.publish(depthMsg);

     //cv::imshow("Published_Image", image);
     cv::imshow("Published_Depth",depth);
     cv::waitKey(3000);

    ros::spinOnce();
    loop_rate.sleep();

    numImg++;
  }
  // cv::destroyWindow("Published_Image");
  // cv::destroyWindow("Published_Depth");


  return 0;
} // main
