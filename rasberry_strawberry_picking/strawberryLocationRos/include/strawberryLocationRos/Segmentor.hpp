#pragma once

#include <fstream>
#include <iostream>
#include <math.h>
#include <numeric>
#include <set>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#include "gSLICr/NVTimer.h"
#include "gSLICr/gSLICr_Lib/gSLICr.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/ml.hpp"
#include "opencv2/opencv.hpp"

#include <eigen3/Eigen/Dense>

#include "strawberryLocationRos/Camera.hpp"

using namespace std;
using namespace cv;
using namespace cv::ml;

namespace sfm
{
    class Segmentor
    {
      public:
        Segmentor(const string &trainfile);

        void computing(const cv::Mat &img);
        void computingBoundingBox(const Mat &img, std::vector<cv::Rect>& outRects);
        void getStrawberryLocation(const cv::Mat& depth, const std::vector<cv::Rect> &rois,  Camera* mCamera, std::vector<cv::Point3d>& sbLocations);

        void tsegment(Mat &frame, const Mat &bw);

        void load_image(const Mat &inimg, gSLICr::UChar4Image *outimg);

        void load_image(const gSLICr::UChar4Image *inimg, Mat &outimg);

        void load_image_bw(const gSLICr::IntImage *inimg, Mat &outimg);

        // gSLICr settings
        gSLICr::objects::settings my_settings;
        gSLICr::engines::core_engine *gSLICr_engine;
        gSLICr::IntImage *bw_contour;
        int *data_ptr;
        gSLICr::objects::spixel_info *spx_map;
        Mat boundry_draw_frame;
        Scalar color_selection, color_conservative, color_nonconservative;
        Mat oldFrame, frame;
        Mat means;
        Mat labels;
        gSLICr::UChar4Image *in_img;
        gSLICr::UChar4Image *out_img;
        Size imgSize;

        std::vector<Mat> good_centres_conservative;
        std::vector<Mat> good_centres_conservative_conv;

        StopWatchInterface *my_timer;

        int total_clusters;
    };
}
