#include "strawberryLocationRos/Camera.hpp"

#include <iostream>

namespace sfm
{
    Camera::Camera()
    {
        mK        = cv::Mat::eye(3, 3, CV_64F);
        mDistCoef = cv::Mat::zeros(5, 1, CV_64F);
    }

    Camera::~Camera()
    {}

    Camera::Camera(const string& strSettingsFile)
    {
        loadParam(strSettingsFile);
    }

    void Camera::loadParam(const string& strSettingsFile)
    {
        cv::FileStorage fSettings(strSettingsFile.c_str(), cv::FileStorage::READ);

        if ( !fSettings.isOpened() )
        {
            cerr << "Failed to open settings file at: " << strSettingsFile << endl;
            exit(-1);
        }

        double fx = fSettings["Camera.fx"];
        double fy = fSettings["Camera.fy"];
        double cx = fSettings["Camera.cx"];
        double cy = fSettings["Camera.cy"];

        cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
        K.at<double>(0, 0) = fx;
        K.at<double>(1, 1) = fy;
        K.at<double>(0, 2) = cx;
        K.at<double>(1, 2) = cy;
        K.copyTo(mK);

        mFocal = (fx + fy) / 2.0;
        mcx    = cx;
        mcy    = cy;

        cv::Mat DistCoef(5, 1, CV_64F);
        DistCoef.at<double>(0) = fSettings["Camera.k1"];
        DistCoef.at<double>(1) = fSettings["Camera.k2"];
        DistCoef.at<double>(2) = fSettings["Camera.p1"];
        DistCoef.at<double>(3) = fSettings["Camera.p2"];
        const double k3 = fSettings["Camera.k3"];
        if (k3 != 0)
        {
            // DistCoef.resize(5);
            DistCoef.at<double>(4) = k3;
        }
        else
        {
            DistCoef.at<double>(4) = 0.0f;
        }
        DistCoef.copyTo(mDistCoef);

        mbf = fSettings["Camera.bf"];

        cout << endl << "Camera Parameters: " << endl;
        cout << "- fx: " << fx << endl;
        cout << "- fy: " << fy << endl;
        cout << "- cx: " << cx << endl;
        cout << "- cy: " << cy << endl;
        cout << "- k1: " << DistCoef.at<double>(0) << endl;
        cout << "- k2: " << DistCoef.at<double>(1) << endl;
        if (DistCoef.rows == 5)cout << "- k3: " << DistCoef.at<double>(4) << endl;
        cout << "- p1: " << DistCoef.at<double>(2) << endl;
        cout << "- p2: " << DistCoef.at<double>(3) << endl;

        mKinv = K.inv();
    } // Camera::loadParam

    void Camera::undistortPoints(Camera* cam, const vector<cv::KeyPoint>& inKP, vector<cv::KeyPoint>& outKP)
    {
        if(cam->mDistCoef.at<float>(0)==0.0)
        {
            outKP=inKP;
            return;
        }
        int N = inKP.size();

        // Fill matrix with points
        cv::Mat mat(N,2,CV_32F);
        for(int i=0; i<N; i++)
        {
            mat.at<float>(i,0)=inKP[i].pt.x;
            mat.at<float>(i,1)=inKP[i].pt.y;
        }

        // Undistort points
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,cam->mK,cam->mDistCoef,cv::Mat(),cam->mK);
        mat=mat.reshape(1);

        // Fill undistorted keypoint vector
        outKP.resize(N);
        for(int i=0; i<N; i++)
        {
            cv::KeyPoint kp = inKP[i];
            kp.pt.x=mat.at<float>(i,0);
            kp.pt.y=mat.at<float>(i,1);
            outKP[i]=kp;
        }
    }

    void Camera::fillOthers()
    {
        mFocal = (mK.at<double>(0,0) + mK.at<double>(1,1)) / 2.0;
        mcx    = mK.at<double>(0,2);
        mcy    = mK.at<double>(1,2);
    }

    cv::Mat Camera::getK()
    {
        return
            mK;
    }

    cv::Mat Camera::getKinv()
    {
        return
            mKinv;
    }

    cv::Mat Camera::getDistCoef()
    {
        return
            mDistCoef;
    }

    double Camera::getFocal()
    {
        return
            mFocal;
    }

    double Camera::getcx()
    {
        return
            mcx;
    }

    double Camera::getcy()
    {
        return
            mcy;
    }
}// namespace
