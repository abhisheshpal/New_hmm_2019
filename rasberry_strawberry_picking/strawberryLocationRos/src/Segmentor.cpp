#include "strawberryLocationRos/Segmentor.hpp"

namespace sfm
{
    Segmentor::Segmentor(const string &trainfile)
    {
        FileStorage fs(trainfile, FileStorage::READ);

        my_settings.img_size.y = fs["gSLICr_settings"]["img_size_y"];
        my_settings.img_size.x = fs["gSLICr_settings"]["img_size_x"];
        my_settings.no_segs = fs["gSLICr_settings"]["no_segs"];
        my_settings.spixel_size = fs["gSLICr_settings"]["superpixels_size"];
        my_settings.coh_weight = fs["gSLICr_settings"]["coh_weight"];
        my_settings.no_iters = fs["gSLICr_settings"]["no_iters"];
        my_settings.color_space = gSLICr::CIELAB;    // gSLICr::CIELAB for Lab, or gSLICr::RGB for RGB
        my_settings.seg_method = gSLICr::GIVEN_SIZE; // or gSLICr::GIVEN_NUM for given number
        my_settings.do_enforce_connectivity = true;  // wheter or not run the enforce connectivity step

        int conservative_count = fs["conservative_count"];
        Mat temp;
        for (int i = 1; i < conservative_count; i++)
        {
            fs["good_centres_conservative" + to_string(i)] >> temp;
            good_centres_conservative.push_back(temp);
            fs["good_centres_conservative_conv" + to_string(i)] >> temp;
            good_centres_conservative_conv.push_back(temp);
        }

        // instantiate a core_engine
        gSLICr_engine = new gSLICr::engines::core_engine(my_settings);

        // gSLICr takes gSLICr::UChar4Image as input and out put
        in_img = new gSLICr::UChar4Image(my_settings.img_size, true, true);
        out_img = new gSLICr::UChar4Image(my_settings.img_size, true, true);

        imgSize = Size(my_settings.img_size.x, my_settings.img_size.y);
        boundry_draw_frame.create(imgSize, CV_8UC3);

        sdkCreateTimer(&my_timer);
        int total_superpixels =
            (my_settings.img_size.x * my_settings.img_size.y) / (my_settings.spixel_size * my_settings.spixel_size);

        // The variables of the EM
        total_clusters = fs["EM_settings"]["EM_total_clusters"];

        fs.release();

        color_conservative = Scalar(0, 255, 0);
        color_nonconservative = Scalar(0, 0, 255);
        color_selection = Scalar(255, 0, 0);
    }

    void Segmentor::computing(const cv::Mat &img)
    {
        resize(img, frame, imgSize);

        bw_contour = new gSLICr::IntImage(my_settings.img_size, true, true);
        load_image(frame, in_img);
        sdkResetTimer(&my_timer);
        sdkStartTimer(&my_timer);
        gSLICr_engine->Process_Frame(in_img);
        sdkStopTimer(&my_timer);
        cout << "\rsegmentation in:[" << sdkGetTimerValue(&my_timer) << "]ms\n" << flush;
        boundry_draw_frame = frame.clone();
        Mat c_frame = frame.clone();
        gSLICr_engine->slic_seg_engine->spixel_map->UpdateHostFromDevice();
        spx_map = gSLICr_engine->slic_seg_engine->spixel_map->GetData(MEMORYDEVICE_CPU);
        gSLICr::IntImage *index_image = gSLICr_engine->slic_seg_engine->Get_Seg_Mask();
        data_ptr = index_image->GetData(MEMORYDEVICE_CPU);

        // Also, prepare the input for the EM
        int total_superpixels =
            (my_settings.img_size.x * my_settings.img_size.y) / (my_settings.spixel_size * my_settings.spixel_size);

        vector<Point3f> superpixels;
        superpixels.resize(total_superpixels);
        for (int i = 0; i < total_superpixels; i++)
        {
            superpixels[i].x = (spx_map + i)->color_info.x;
            superpixels[i].y = (spx_map + i)->color_info.y;
            superpixels[i].z = (spx_map + i)->color_info.z;
        }

        // Start the EM
        Ptr<EM> em_model = EM::create();
        em_model->setClustersNumber(total_clusters);
        em_model->setCovarianceMatrixType(EM::COV_MAT_DIAGONAL);

        Mat samples = Mat(superpixels).reshape(1);
        em_model->setTermCriteria(TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 300, 0.1));
        em_model->trainEM(samples, noArray(), labels, noArray());
        means = em_model->getMeans();

        // Working with the covariances instead of the thresholds for euclidean
        // nearest neighbor search.
        std::vector<Mat> covs;
        em_model->getCovs(covs);

        gSLICr::UChar4Image *clusters = new gSLICr::UChar4Image(gSLICr::Vector2i(total_superpixels, 1), true, true);
        gSLICr::Vector4u *cluster_ptr = clusters->GetData(MEMORYDEVICE_CPU);

        for (int i = 0; i < total_superpixels; i++)
            cluster_ptr[i].b = labels.at<int>(i);
        gSLICr_engine->Push_Cluster_Data(clusters);

        // Create Windows
        //        namedWindow("conservative segmentation", 0);

        std::set<int> current_good_cluster_index_conservative;
        for (int i = 0; i < good_centres_conservative.size(); i++)
        {
            double min_value = 99999;
            int min_index = -1;

            Mat sigma1_mat = good_centres_conservative_conv[i];
            Mat mean1_mat = good_centres_conservative[i];

            for (int j = 0; j < total_clusters; j++)
            {
                Mat sigma2_mat = covs[j];
                Mat mean2_mat = means.row(j);

                // Map the OpenCV matrix with Eigen:
                Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> sigma1(
                    sigma1_mat.ptr<double>(), sigma1_mat.rows, sigma1_mat.cols);
                Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> sigma2(
                    sigma2_mat.ptr<double>(), sigma2_mat.rows, sigma2_mat.cols);
                Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> mean1(
                    mean1_mat.ptr<double>(), mean1_mat.rows, mean1_mat.cols);
                Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> mean2(
                    mean2_mat.ptr<double>(), mean2_mat.rows, mean2_mat.cols);

                Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> temp1 =
                    sigma2.inverse() * sigma1;
                Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> temp2 = mean2 - mean1;
                Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> temp3 =
                    temp2 * sigma2.inverse() * temp2.transpose();

                double temp = temp1.trace() + temp3(0) + log(sigma2.determinant() / sigma1.determinant());

                double mahDist =
                    sqrt((Mat((Mat(mean1_mat - mean2_mat)) * sigma1_mat.inv() * (Mat(mean1_mat - mean2_mat)).t()))
                             .at<double>(0, 0));
                // cout<<"mahalonobis distance:"<<mahDist<<endl;

                if (temp < min_value && mahDist < 1.3)
                {
                    min_value = temp;
                    min_index = j;
                }
            }
            // cout << "minimum value consv " << i << ": " << min_value << endl;
            current_good_cluster_index_conservative.insert(min_index);
        }
        boundry_draw_frame = frame.clone();
        Mat bw_contour_frame(my_settings.img_size.y, my_settings.img_size.x, CV_8UC1, Scalar(0));

        for (std::set<int>::iterator it = current_good_cluster_index_conservative.begin();
             it != current_good_cluster_index_conservative.end(); ++it)
        {
            if (*it != -1)
                gSLICr_engine->Draw_Bw_Contour(bw_contour, *it, 1);
        }
        load_image_bw(bw_contour, bw_contour_frame);

        Mat element = getStructuringElement(MORPH_RECT, Size(2 * 7 + 1, 2 * 7 + 1), Point(7, 7));
        morphologyEx(bw_contour_frame, bw_contour_frame, MORPH_OPEN, element);

        Mat conn_labels, conn_stats, centroids;
        connectedComponentsWithStats(bw_contour_frame, conn_labels, conn_stats, centroids);
        Mat manualbox = frame.clone();
        int ccs = 0;

        for (int ci = 0; ci < conn_stats.rows; ci++)
        {
            if (conn_stats.at<int>(ci, cv::CC_STAT_AREA) <= frame.rows * frame.cols * .6)
            {
                Point2f top_left(conn_stats.at<int>(ci, CC_STAT_LEFT), conn_stats.at<int>(ci, CC_STAT_TOP));
                Point2f bottom_right(top_left.x + conn_stats.at<int>(ci, CC_STAT_WIDTH),
                                     top_left.y + conn_stats.at<int>(ci, CC_STAT_HEIGHT));
                rectangle(manualbox, top_left, bottom_right, Scalar(255, 0, 0), 3, 8, 0);
                ccs++;
            }
        }

        if (ccs > 0)
        {
            imshow("Detected", manualbox);
            tsegment(c_frame, bw_contour_frame);
            waitKey(10);
        }

        delete bw_contour;
    }

    void Segmentor::computingBoundingBox(const Mat &img, std::vector<cv::Rect> &outRects)
    {
        resize(img, frame, imgSize);

        Size simg = img.size();
        float ratioX = (float)simg.width / (float)imgSize.width;
        float ratioY = (float)simg.height / (float)imgSize.height;

        bw_contour = new gSLICr::IntImage(my_settings.img_size, true, true);
        load_image(frame, in_img);
        sdkResetTimer(&my_timer);
        sdkStartTimer(&my_timer);
        gSLICr_engine->Process_Frame(in_img);
        sdkStopTimer(&my_timer);
        cout << "\rsegmentation in:[" << sdkGetTimerValue(&my_timer) << "]ms\n" << flush;
        boundry_draw_frame = frame.clone();
        Mat c_frame = frame.clone();
        gSLICr_engine->slic_seg_engine->spixel_map->UpdateHostFromDevice();
        spx_map = gSLICr_engine->slic_seg_engine->spixel_map->GetData(MEMORYDEVICE_CPU);
        gSLICr::IntImage *index_image = gSLICr_engine->slic_seg_engine->Get_Seg_Mask();
        data_ptr = index_image->GetData(MEMORYDEVICE_CPU);

        // Also, prepare the input for the EM
        int total_superpixels =
            (my_settings.img_size.x * my_settings.img_size.y) / (my_settings.spixel_size * my_settings.spixel_size);

        vector<Point3f> superpixels;
        superpixels.resize(total_superpixels);
        for (int i = 0; i < total_superpixels; i++)
        {
            superpixels[i].x = (spx_map + i)->color_info.x;
            superpixels[i].y = (spx_map + i)->color_info.y;
            superpixels[i].z = (spx_map + i)->color_info.z;
        }

        // Start the EM
        Ptr<EM> em_model = EM::create();
        em_model->setClustersNumber(total_clusters);
        em_model->setCovarianceMatrixType(EM::COV_MAT_DIAGONAL);

        Mat samples = Mat(superpixels).reshape(1);
        em_model->setTermCriteria(TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 300, 0.1));
        em_model->trainEM(samples, noArray(), labels, noArray());
        means = em_model->getMeans();

        // Working with the covariances instead of the thresholds for euclidean
        // nearest neighbor search.
        std::vector<Mat> covs;
        em_model->getCovs(covs);

        gSLICr::UChar4Image *clusters = new gSLICr::UChar4Image(gSLICr::Vector2i(total_superpixels, 1), true, true);
        gSLICr::Vector4u *cluster_ptr = clusters->GetData(MEMORYDEVICE_CPU);

        for (int i = 0; i < total_superpixels; i++)
            cluster_ptr[i].b = labels.at<int>(i);
        gSLICr_engine->Push_Cluster_Data(clusters);

        // Create Windows
        //        namedWindow("conservative segmentation", 0);

        std::set<int> current_good_cluster_index_conservative;
        for (int i = 0; i < good_centres_conservative.size(); i++)
        {
            double min_value = 99999;
            int min_index = -1;

            Mat sigma1_mat = good_centres_conservative_conv[i];
            Mat mean1_mat = good_centres_conservative[i];

            for (int j = 0; j < total_clusters; j++)
            {
                Mat sigma2_mat = covs[j];
                Mat mean2_mat = means.row(j);

                // Map the OpenCV matrix with Eigen:
                Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> sigma1(
                    sigma1_mat.ptr<double>(), sigma1_mat.rows, sigma1_mat.cols);
                Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> sigma2(
                    sigma2_mat.ptr<double>(), sigma2_mat.rows, sigma2_mat.cols);
                Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> mean1(
                    mean1_mat.ptr<double>(), mean1_mat.rows, mean1_mat.cols);
                Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> mean2(
                    mean2_mat.ptr<double>(), mean2_mat.rows, mean2_mat.cols);

                Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> temp1 =
                    sigma2.inverse() * sigma1;
                Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> temp2 = mean2 - mean1;
                Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> temp3 =
                    temp2 * sigma2.inverse() * temp2.transpose();

                double temp = temp1.trace() + temp3(0) + log(sigma2.determinant() / sigma1.determinant());

                double mahDist =
                    sqrt((Mat((Mat(mean1_mat - mean2_mat)) * sigma1_mat.inv() * (Mat(mean1_mat - mean2_mat)).t()))
                             .at<double>(0, 0));
                // cout<<"mahalonobis distance:"<<mahDist<<endl;

                if (temp < min_value && mahDist < 1.3)
                {
                    min_value = temp;
                    min_index = j;
                }
            }
            // cout << "minimum value consv " << i << ": " << min_value << endl;
            current_good_cluster_index_conservative.insert(min_index);
        }
        boundry_draw_frame = frame.clone();
        Mat bw_contour_frame(my_settings.img_size.y, my_settings.img_size.x, CV_8UC1, Scalar(0));

        for (std::set<int>::iterator it = current_good_cluster_index_conservative.begin();
             it != current_good_cluster_index_conservative.end(); ++it)
        {
            if (*it != -1)
                gSLICr_engine->Draw_Bw_Contour(bw_contour, *it, 1);
        }
        load_image_bw(bw_contour, bw_contour_frame);

        Mat element = getStructuringElement(MORPH_RECT, Size(2 * 7 + 1, 2 * 7 + 1), Point(7, 7));
        morphologyEx(bw_contour_frame, bw_contour_frame, MORPH_OPEN, element);

        Mat conn_labels, conn_stats, centroids;
        connectedComponentsWithStats(bw_contour_frame, conn_labels, conn_stats, centroids);
        int ccs = 0;

        for (int ci = 0; ci < conn_stats.rows; ci++)
        {
            if (conn_stats.at<int>(ci, cv::CC_STAT_AREA) <= frame.rows * frame.cols * .6)
            {
                Point2f top_left(conn_stats.at<int>(ci, CC_STAT_LEFT), conn_stats.at<int>(ci, CC_STAT_TOP));
                Point2f bottom_right(top_left.x + conn_stats.at<int>(ci, CC_STAT_WIDTH),
                                     top_left.y + conn_stats.at<int>(ci, CC_STAT_HEIGHT));
                top_left.x = top_left.x * ratioX;
                top_left.y = top_left.y * ratioY;
                bottom_right.x = bottom_right.x * ratioX;
                bottom_right.y = bottom_right.y * ratioY;

                outRects.push_back(cv::Rect(top_left, bottom_right));
                ccs++;
            }
        }
        cout << "Total of " << ccs << " Strawberries " << endl;
        delete bw_contour;
    }

    void Segmentor::getStrawberryLocation(const cv::Mat& depth, const std::vector<cv::Rect> &rois, Camera* mCamera, std::vector<cv::Point3d>& sbLocations)
    {
      cout<<"Calculating the position of the Strawberries ... "<<endl;
      for(int i = 0 ; i < rois.size(); ++i)
      {
        cv::Rect roi = rois[i];

        cv::Mat depthROI = depth(roi).clone();
        double totalD = 0;
        double cnt = 0;
        double d=0;
        for (int i = 0; i < depthROI.total(); ++i)
        {
            double tmpD =  (double)depthROI.at<ushort>(i);
            if (tmpD > 500 && tmpD < 1500)
            {
                totalD += tmpD;
                cnt++;
            }

        }

        d = totalD / cnt / 1000.0; // convert to meters

        if(cnt != 0 && d > 0.5 && d < 1.5)
        {
            // ignore the distortion part since realsense has no distortion
            cv::Point2d center = cv::Point2f(roi.x + roi.width / 2.0, roi.y + roi.height / 2.0);

            //get the position
            cv::Mat pos(3,1,CV_64F);
            pos.at<double>(0) = center.x;
            pos.at<double>(1) = center.y;
            pos.at<double>(2) = 1.0;

            pos = mCamera->mKinv * pos;
            pos = pos * d;
            sbLocations.push_back(cv::Point3d(pos));
        }
      }
    }

    void Segmentor::tsegment(Mat &frame, const Mat &bw)
    {
        for (int r = 0; r < frame.rows; r++)
        {
            for (int c = 0; c < frame.cols; c++)
            {
                if (bw.at<uchar>(r, c) == 0)
                {
                    frame.at<Vec3b>(r, c)[0] = 0;
                    frame.at<Vec3b>(r, c)[1] = 0;
                    frame.at<Vec3b>(r, c)[2] = 0;
                }
            }
        }
        namedWindow("seg", 0);
        imshow("seg", frame);
    }

    void Segmentor::load_image(const Mat &inimg, gSLICr::UChar4Image *outimg)
    {
        gSLICr::Vector4u *outimg_ptr = outimg->GetData(MEMORYDEVICE_CPU);

        for (int y = 0; y < outimg->noDims.y; y++)
            for (int x = 0; x < outimg->noDims.x; x++)
            {
                int idx = x + y * outimg->noDims.x;
                outimg_ptr[idx].b = inimg.at<Vec3b>(y, x)[0];
                outimg_ptr[idx].g = inimg.at<Vec3b>(y, x)[1];
                outimg_ptr[idx].r = inimg.at<Vec3b>(y, x)[2];
            }
    }

    void Segmentor::load_image(const gSLICr::UChar4Image *inimg, Mat &outimg)
    {
        const gSLICr::Vector4u *inimg_ptr = inimg->GetData(MEMORYDEVICE_CPU);

        for (int y = 0; y < inimg->noDims.y; y++)
            for (int x = 0; x < inimg->noDims.x; x++)
            {
                int idx = x + y * inimg->noDims.x;
                outimg.at<Vec3b>(y, x)[0] = inimg_ptr[idx].b;
                outimg.at<Vec3b>(y, x)[1] = inimg_ptr[idx].g;
                outimg.at<Vec3b>(y, x)[2] = inimg_ptr[idx].r;
            }
    }

    void Segmentor::load_image_bw(const gSLICr::IntImage *inimg, Mat &outimg)
    {
        const int *inimg_ptr = inimg->GetData(MEMORYDEVICE_CPU);

        for (int y = 0; y < inimg->noDims.y; y++)
            for (int x = 0; x < inimg->noDims.x; x++)
            {
                int idx = x + y * inimg->noDims.x;
                outimg.at<unsigned char>(y, x) = inimg_ptr[idx];
            }
    }
}
