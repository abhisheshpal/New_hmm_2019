/*
 * The Camera class contains the intrinsic and distortion parameter for the camera model.
 * We use a pinhole camera model
 */
#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <iostream>
#include <iomanip>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

using namespace std;

namespace sfm
{
    class Camera
    {
public:
        Camera();
        ~Camera();

        Camera(const string& strSettingsFile);

        void loadParam(const string& strSettingsFile);

        void undistortPoints(Camera* cam, const vector<cv::KeyPoint>& inKP, vector<cv::KeyPoint>& outKP);

        void fillOthers();

        cv::Mat getK();

        cv::Mat getKinv();

        cv::Mat getDistCoef();

        double getFocal();

        double getcx();

        double getcy();

public:
        // camera info
        cv::Mat mK;
        cv::Mat mKinv;
        double mFocal;
        double mcx;
        double mcy;
        cv::Mat mDistCoef;
        double mbf;
    };
}// namespace

#endif // ifndef CAMERA_HPP
