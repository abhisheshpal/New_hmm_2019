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

// PCL
// #include <pcl/io/ply_io.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
// #include <pcl/point_cloud.h>
// #include <pcl/visualization/cloud_viewer.h>

#include "example.hpp"


using namespace std;
using namespace cv;

// pcl::visualization::CloudViewer viewer("point clouds");


int main(int argc, char** argv)
{
    if(argc < 2)
    {
        cerr<<"Need folder name argument: ./a.out folderName/"<<endl;

        return 0;
    }
    string folderName = string(argv[1]);


    rs::context ctx;
    rs::device * dev = ctx.get_device(0);


    std::vector<rs::stream> supported_streams;

    for (int i = (int)rs::capabilities::depth; i < (int)rs::capabilities::fish_eye; i++)
        if (dev->supports((rs::capabilities)i))
            supported_streams.push_back((rs::stream)i);


    // Configure all supported streams
//    for (auto & stream : supported_streams)
//        dev->enable_stream(stream, rs::preset::best_quality);
    // ///////
     dev->enable_stream(rs::stream::depth, 480, 360, rs::format::z16, 30);
     dev->enable_stream(rs::stream::color, 640, 480, rs::format::rgb8, 30);
     dev->enable_stream(rs::stream::infrared, 480, 360, rs::format::y8, 30);
     dev->enable_stream(rs::stream::infrared2, 480, 360, rs::format::y8, 30);
    ///////


    // Compute field of view for each enabled stream
    for (auto & stream : supported_streams)
    {
        if (!dev->is_stream_enabled(stream)) continue;
        auto intrin = dev->get_stream_intrinsics(stream);
        std::cout << "Capturing " << stream << " at " << intrin.width << " x " << intrin.height;
        std::cout << std::setprecision(1) << std::fixed << ", fov = " << intrin.hfov() << " x " << intrin.vfov() << ", distortion = " << intrin.model() << std::endl;
    }


    ///////
    // configure outdoor setting for parameters
   dev->set_option(rs::option::r200_lr_gain, 100);
   dev->set_option(rs::option::r200_lr_exposure, 20);
    ///////
    // configure indoor setting for parameters
    // dev->set_option(rs::option::color_enable_auto_exposure, 1);
    // dev->set_option(rs::option::color_enable_auto_white_balance, 1);
    // dev->set_option(rs::option::r200_lr_auto_exposure_enabled, 1);
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


    // loop to get the data
    int nameIdx = 0;
    while (1)
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

        namedWindow("Display Depth Image Aligned to Rectified Color Image", WINDOW_AUTOSIZE );
        imshow("Display Depth Image Aligned to Rectified Color Image", cv_hist_image);
        waitKey(1);

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

        //define point cloud data set
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
       //
      //  for(int dy=0; dy<depth_intrin.height; ++dy)
      //   {
      //       for(int dx=0; dx<depth_intrin.width; ++dx)
      //       {
      //           // Retrieve the 16-bit depth value and map it into a depth in meters
      //           uint16_t depth_value = depth_image[dy * depth_intrin.width + dx];
      //           float depth_in_meters = depth_value * scale;
       //
      //           // Skip over pixels with a depth value of zero, which is used to indicate no data
      //           if(depth_value == 0) continue;
       //
      //           // Map from pixel coordinates in the depth image to pixel coordinates in the color image
      //           rs::float2 depth_pixel = {(float)dx, (float)dy};
      //           rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);
      //           rs::float3 color_point = depth_to_color.transform(depth_point);
      //           rs::float2 color_pixel = color_intrin.project(color_point);
       //
      //           // Use the color from the nearest color pixel, or pure white if this point falls outside the color image
      //           const int cx = (int)std::round(color_pixel.x), cy = (int)std::round(color_pixel.y);
      //           int r = 0, g = 0, b = 0;
      //           if(cx < 0 || cy < 0 || cx >= color_intrin.width || cy >= color_intrin.height)
      //           {
      //               r = 255; b = 255; g = 255;
      //           }
      //           else
      //           {
      //               b = cv_color_image.at<cv::Vec3b>(cy,cx)[0];
      //               g = cv_color_image.at<cv::Vec3b>(cy,cx)[1];
      //               r = cv_color_image.at<cv::Vec3b>(cy,cx)[2];
      //           }
       //
      //           pcl::PointXYZRGB p = pcl::PointXYZRGB(r,g,b);
      //           p.x = depth_point.x;
      //           p.y = depth_point.y;
      //           p.z = depth_point.z;
       //
      //           pc_ptr->push_back(p);
       //
      //       }
      //   }

        // pc_ptr->width = (int)pc_ptr->points.size();
        // pc_ptr->height = 1;

        // viewer.showCloud(pc_ptr); // show 3D point cloud

        ///////
        //save point cloud data
        ostringstream convert;
        convert << nameIdx;
        //string pointcloudName = folderName + "points/" + convert.str() + ".pcd";
        //pcl::io::savePCDFileBinary(pointcloudName, *pc_ptr);


        // save color images
        //string colorName = folderName + "color/" + convert.str()+".jpg";
        //imwrite(colorName, cv_color_image);

        // save rectified color images
        string rect_colorName = folderName + "rect_color/" + convert.str()+".jpg";
        imwrite(rect_colorName, cv_rect_image);

        // save depth data
        string depthdName = folderName + "depth/" + convert.str()+".png";
        imwrite(depthdName, cv_depth_image);

        // save infrared images
        string infraredName = folderName + "infrared/" + convert.str()+".jpg";
        imwrite(infraredName, cv_infrared);

        // save infrared2 images
        string infrared2Name = folderName + "infrared2/" + convert.str()+".jpg";
        imwrite(infrared2Name,cv_infrared2);
        ///////


        // Display the image in GUI
        // namedWindow("Display Image", WINDOW_AUTOSIZE );

        // imshow("Display Image", cv_rect_image);

        // namedWindow("Display Image", WINDOW_AUTOSIZE );
        // imshow("Display Image", cv_color_image);

        // if (waitKey(1) != -1)
        //     break;

        dev->wait_for_frames();

        nameIdx++;

    }

    return 0;
}



        // //save point cloud data
        // ostringstream convert;
        // convert << nameIdx;
        // string pointcloudName = folderName + "points/" + convert.str() + ".pcd";
        // pcl::io::savePCDFileBinary(pointcloudName, *pc_ptr);


        // // save color images
        // string colorName = folderName + "color/" + convert.str()+".jpg";
        // imwrite(colorName, cv_color_image);

        // // save rectified color images
        // string rect_colorName = folderName + "rect_color/" + convert.str()+".jpg";
        // imwrite(rect_colorName, cv_rect_image);

        // // save depth data
        // string depthdName = folderName + "depth/" + convert.str()+".png";
        // imwrite(depthdName, cv_depth_image);

        // // save infrared images
        // string infraredName = folderName + "infrared/" + convert.str()+".jpg";
        // imwrite(infraredName, cv_infrared);

        // // save infrared2 images
        // string infrared2Name = folderName + "infrared2/" + convert.str()+".jpg";
        // imwrite(infrared2Name,cv_infrared2);
