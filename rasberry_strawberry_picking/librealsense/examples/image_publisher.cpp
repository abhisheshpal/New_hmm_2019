#include <sstream>
#include <iostream>
#include <iomanip>
#include <thread>
#include <algorithm>
#include <map>
#include <memory>
#include <string>

// include the librealsense C++ header file
#include <librealsense/rs.hpp>

// include OpenCV header file
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "example.hpp"


using namespace std;
using namespace cv;


// pcl::visualization::CloudViewer viewer("point clouds");


int main(int argc, char** argv)
{
    if(argc < 1)
    {
        cerr<<"Need folder name argument: ./a.out "<<endl;

        return 0;
    }
    //string folderName = string(argv[1]);

   //cout<<folderName<<endl;
    rs::context ctx;
    rs::device * dev = ctx.get_device(0);


    std::vector<rs::stream> supported_streams;

    for (int i = (int)rs::capabilities::depth; i < (int)rs::capabilities::fish_eye; i++)
        if (dev->supports((rs::capabilities)i))
            supported_streams.push_back((rs::stream)i);


    // Configure all supported streams
    for (auto & stream : supported_streams)
        dev->enable_stream(stream, rs::preset::best_quality);
    // ///////
    // dev->enable_stream(rs::stream::depth, 480, 360, rs::format::z16, 30);
    // dev->enable_stream(rs::stream::color, 640, 480, rs::format::rgb8, 30);
    // dev->enable_stream(rs::stream::infrared, 480, 360, rs::format::y8, 30);
    // dev->enable_stream(rs::stream::infrared2, 480, 360, rs::format::y8, 30);
    ///////


    // Compute field of view for each enabled stream
    for (auto & stream : supported_streams)
    {
        if (!dev->is_stream_enabled(stream)) continue;
        auto intrin = dev->get_stream_intrinsics(stream);
        std::cout << "Capturing " << stream << " at " << intrin.width << " x " << intrin.height;
        std::cout << std::setprecision(1) << std::fixed << ", fov = " << intrin.hfov() << " x " << intrin.vfov() << ", distortion = " << intrin.model() << std::endl;
    }


    ///////config color
    dev->set_option(rs::option::color_enable_auto_exposure, 1);
    dev->set_option(rs::option::color_enable_auto_white_balance, 1);
    /////////////// configure outdoor setting for parameters
    //dev->set_option(rs::option::r200_lr_gain, 100);
    //dev->set_option(rs::option::r200_lr_exposure, 25);
    //////////////////////////
    // configure indoor setting for parameters
    //dev->set_option(rs::option::r200_lr_auto_exposure_enabled, 1);
    dev->set_option(rs::option::r200_lr_exposure, 100);
    /////

    // configure filtering setting for 3D point clound
    ///////
    rs::apply_depth_control_preset(dev, 4);
    ///////

    // Start streaming
    dev->start();

    // Camera warmup - Dropped frames to allow stabilization
    for(int i = 0; i < 40; i++)
        dev->wait_for_frames();

    // Get sizes of different images
    const int depth_width = dev->get_stream_width(rs::stream::depth);
    const int depth_height = dev->get_stream_height(rs::stream::depth);
    int image_width = dev->get_stream_width(rs::stream::color);
    int image_height = dev->get_stream_height(rs::stream::color);
    int image_rect_width = dev->get_stream_width(rs::stream::rectified_color);
    int image_rect_height = dev->get_stream_height(rs::stream::rectified_color);
    int infrared_width = dev->get_stream_width(rs::stream::infrared);
    int infrared_height = dev->get_stream_height(rs::stream::infrared);
    ///////
    const int depth2rectColor_width = dev->get_stream_width(rs::stream::depth_aligned_to_rectified_color);
    const int depth2rectColor_height = dev->get_stream_height(rs::stream::depth_aligned_to_rectified_color);
    ///////

    // print out the intrinsics and extrinsics for different streams
    auto intrin = dev->get_stream_intrinsics(rs::stream::rectified_color);
    std::cout << "Capturing " << rs::stream::rectified_color << " at " << intrin.width << " x " << intrin.height;
    std::cout << std::fixed << std::setw( 11 ) << std::setprecision( 6 ) << std::setfill( '0' )
              << ", Focal length = " << intrin.fx << " x " << intrin.fy
              << ", principal point = " << intrin.ppx << " x " << intrin.ppy
              << ", distortion = " << intrin.coeffs[0] << " " << intrin.coeffs[1] << " " << intrin.coeffs[2] << " "
              << intrin.coeffs[3] << " " << intrin.coeffs[4] << std::endl;

    auto intrinD = dev->get_stream_intrinsics(rs::stream::depth);
    std::cout << "Capturing " << rs::stream::depth << " at " << intrinD.width << " x " << intrinD.height;
    std::cout << std::fixed << std::setw( 11 ) << std::setprecision( 6 ) << std::setfill( '0' )
              << ", Focal length = " << intrinD.fx << " x " << intrinD.fy
              << ", principal point = " << intrinD.ppx << " x " << intrinD.ppy
              << ", distortion = " << intrinD.coeffs[0] << " " << intrinD.coeffs[1] << " " << intrinD.coeffs[2] << " "
              << intrinD.coeffs[3] << " " << intrinD.coeffs[4] << std::endl;

    auto intrinLI = dev->get_stream_intrinsics(rs::stream::infrared);
    std::cout << "Capturing " << rs::stream::infrared << " at " << intrinLI.width << " x " << intrinLI.height;
    std::cout << std::fixed << std::setw( 11 ) << std::setprecision( 6 ) << std::setfill( '0' )
              << ", Focal length = " << intrinLI.fx << " x " << intrinLI.fy
              << ", principal point = " << intrinLI.ppx << " x " << intrinLI.ppy
              << ", distortion = " << intrinLI.coeffs[0] << " " << intrinLI.coeffs[1] << " " << intrinLI.coeffs[2] << " "
              << intrinLI.coeffs[3] << " " << intrinLI.coeffs[4] << std::endl;
    // print out depth to rectified color
    auto intrinD2RC = dev->get_stream_intrinsics(rs::stream::depth_aligned_to_rectified_color);
    std::cout << "Capturing " << rs::stream::depth_aligned_to_rectified_color << " at " << intrinD2RC.width << " x " << intrinD2RC.height;
    std::cout << std::fixed << std::setw( 11 ) << std::setprecision( 6 ) << std::setfill( '0' )
              << ", Focal length = " << intrinD2RC.fx << " x " << intrinD2RC.fy
              << ", principal point = " << intrinD2RC.ppx << " x " << intrinD2RC.ppy
              << ", distortion = " << intrinD2RC.coeffs[0] << " " << intrinD2RC.coeffs[1] << " " << intrinD2RC.coeffs[2] << " "
              << intrinD2RC.coeffs[3] << " " << intrinD2RC.coeffs[4] << std::endl;

    rs::extrinsics rect_color_to_depth = dev->get_extrinsics(rs::stream::rectified_color, rs::stream::depth);
    std::cout << "Capturing extrinsics from " << rs::stream::rectified_color << " to " << rs::stream::depth;
    std::cout << std::fixed << std::setw( 11 ) << std::setprecision( 8 ) << std::setfill( '0' )
              << ", Rotation = " << rect_color_to_depth.rotation[0] << ", " << rect_color_to_depth.rotation[1] << ", " << rect_color_to_depth.rotation[2] << ", "
              << rect_color_to_depth.rotation[3] << ", " << rect_color_to_depth.rotation[4] << ", " << rect_color_to_depth.rotation[5] << ", "
              << rect_color_to_depth.rotation[6] << ", " << rect_color_to_depth.rotation[7] << ", " << rect_color_to_depth.rotation[8]
              << ", Translation = " << rect_color_to_depth.translation[0] << ", " << rect_color_to_depth.translation[1] << ", " << rect_color_to_depth.translation[2] << std::endl;

    rs::extrinsics infrared_to_depth = dev->get_extrinsics(rs::stream::infrared, rs::stream::depth);
    std::cout << "Capturing extrinsics from " << rs::stream::infrared << " to " << rs::stream::depth;
    std::cout << std::fixed << std::setw( 11 ) << std::setprecision( 8 ) << std::setfill( '0' )
              << ", Rotation = " << infrared_to_depth.rotation[0] << ", " << infrared_to_depth.rotation[1] << ", " << infrared_to_depth.rotation[2] << ", "
              << infrared_to_depth.rotation[3] << ", " << infrared_to_depth.rotation[4] << ", " << infrared_to_depth.rotation[5] << ", "
              << infrared_to_depth.rotation[6] << ", " << infrared_to_depth.rotation[7] << ", " << infrared_to_depth.rotation[8]
              << ", Translation = " << infrared_to_depth.translation[0] << ", " << infrared_to_depth.translation[1] << ", " << infrared_to_depth.translation[2] << std::endl;

    // initialize buffer to store

    ////////////////////////////////////////
    ros::init(argc,argv,"image_publisher");
    ros::NodeHandle nh;

    cv::namedWindow("view_pub");
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Publisher imgPub = it.advertise("camera/image", 1);
    image_transport::Publisher depthPub = it.advertise("camera/depth",1);
    sensor_msgs::ImagePtr imgMsg ,depthMsg;
    ros::Rate loop_rate(30);
    ///////////////////////////////////////

    // loop to get the data
    int nameIdx = 0;
    while (nh.ok())
    {
        // Retrieve image data for point cloud
        ////////
        // const uint16_t * depth_image = (const uint16_t *)dev->get_frame_data(rs::stream::depth);
        const uint16_t * depth_image = (const uint16_t *)dev->get_frame_data(rs::stream::depth_aligned_to_rectified_color);
        // const uint8_t * color_image = (const uint8_t *)dev->get_frame_data(rs::stream::color);

        // histogram image
        // uint8_t hist_rgb_image[depth_width*depth_height*3];
        uint8_t hist_rgb_image[depth2rectColor_width*depth2rectColor_height*3];
        // make_depth_histogram(hist_rgb_image, depth_image, depth_width, depth_height);
        make_depth_histogram(hist_rgb_image, depth_image, depth2rectColor_width, depth2rectColor_height);
        // Mat cv_hist_image(Size(depth_width, depth_height), CV_8UC3, (void*)(hist_rgb_image), Mat::AUTO_STEP);
        Mat cv_hist_image(Size(depth2rectColor_width, depth2rectColor_height), CV_8UC3, (void*)(hist_rgb_image), Mat::AUTO_STEP);
        ///////
        cv::cvtColor(cv_hist_image, cv_hist_image, cv::COLOR_BGR2RGB);



        // color images
        Mat cv_color_image(Size(image_width, image_height), CV_8UC3, (void*)(dev->get_frame_data(rs::stream::color)), Mat::AUTO_STEP);
        Mat cv_rect_image(Size(image_rect_width, image_rect_height), CV_8UC3, (void*)(dev->get_frame_data(rs::stream::rectified_color)), Mat::AUTO_STEP);
        // IR and depth images
        Mat cv_infrared(Size(infrared_width, infrared_height), CV_8U, (void*)(dev->get_frame_data(rs::stream::infrared)), Mat::AUTO_STEP);
        Mat cv_infrared2(Size(infrared_width, infrared_height), CV_8U, (void*)(dev->get_frame_data(rs::stream::infrared2)), Mat::AUTO_STEP);
        ///////
        // Mat cv_depth_image(Size(depth_width, depth_height), CV_16U, (void*)(dev->get_frame_data(rs::stream::depth)), Mat::AUTO_STEP);
        Mat cv_depth_image(Size(depth2rectColor_width, depth2rectColor_height), CV_16U, (void*)(dev->get_frame_data(rs::stream::depth_aligned_to_rectified_color)), Mat::AUTO_STEP);
        ///////

        cv::cvtColor(cv_color_image, cv_color_image, cv::COLOR_BGR2RGB);
        cv::cvtColor(cv_rect_image, cv_rect_image, cv::COLOR_BGR2RGB);

        // Retrieve camera parameters for mapping between depth and color
        rs::intrinsics depth_intrin = dev->get_stream_intrinsics(rs::stream::depth);
        rs::extrinsics depth_to_color = dev->get_extrinsics(rs::stream::depth, rs::stream::color);
        rs::intrinsics color_intrin = dev->get_stream_intrinsics(rs::stream::color);
        float scale = dev->get_depth_scale();

        // const float scale = dev->get_depth_scale();
        // cout << std::fixed << std::setw( 11 ) << std::setprecision( 6 ) << std::setfill( '0' ) << scale << endl;

        // viewer.showCloud(pc_ptr); // show 3D point cloud

        ///////
        // publish the image data
        if(cv_color_image.empty() || cv_depth_image.empty())continue;

        imgMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_rect_image).toImageMsg();
        depthMsg = cv_bridge::CvImage(std_msgs::Header(), "16UC1", cv_depth_image).toImageMsg();

        imgPub.publish(imgMsg);
        depthPub.publish(depthMsg);
        cv::waitKey(1);

        ostringstream convert;
        convert << nameIdx;
        //string pointcloudName = folderName + "points/" + convert.str() + ".pcd";

        // save color images
        //string colorName = folderName + "color/" + convert.str()+".jpg";
         //cout<<colorName<<endl;
         //imwrite(colorName, cv_color_image);

        // save rectified color images
        // string rect_colorName = folderName + "rect_color/" + convert.str()+".jpg";
        // imwrite(rect_colorName, cv_rect_image);

        // save depth data
         //string depthdName = folderName + "depth/" + convert.str()+".png";
         //imwrite(depthdName, cv_depth_image);

        // save infrared images
        // string infraredName = folderName + "infrared/" + convert.str()+".jpg";
        // imwrite(infraredName, cv_infrared);

        // save infrared2 images
        // string infrared2Name = folderName + "infrared2/" + convert.str()+".jpg";
        // imwrite(infrared2Name,cv_infrared2);
        ///////


        // Display the image in GUI
        namedWindow("Display Image", WINDOW_AUTOSIZE );
        imshow("view_pub", cv_hist_image);
        waitKey(1);
        dev->wait_for_frames();

        ros::spinOnce();
        loop_rate.sleep();
        nameIdx++;

    }

    return 0;
}
