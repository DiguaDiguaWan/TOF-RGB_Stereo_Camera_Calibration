#include <stdio.h>
#include <iostream>
#include <stdint.h>
#include "opencv2/opencv.hpp"
#include "CornerDetAC/CornerDetAC.h"
#include "ChessboradStruct/ChessboradStruct.h"

std::string leftfile="./image_data/rgb";
std::string rightfile="./image_data/tof(ir)";

std::string intrinsicsfile="./camera_param/intrinsics.yml";
std::string extrinsicsfile="./camera_param/extrinsics.yml";

int sizeOfCheckerboard_w=11;
int sizeOfCheckerboard_h=8;


int main()
{
    std::vector<std::string> leftList, rightList;
    cv::glob(leftfile, leftList);
    cv::glob(rightfile, rightList);

    if (leftList.size() == 0 || rightList.size() == 0) {
        if (leftList.size() == 0)
            std::cout << "The path on the left is incorrect, please check" << std::endl;
        if (rightList.size() == 0)
            std::cout << "The path on the right is incorrect, please check" << std::endl;
        if (rightList.size() != leftList.size())
            std::cout << "The num of image is incorrect, please check" << std::endl;
        return -1;
    }
    int found = 0;
    int sum = 0;
    std::vector<std::vector<cv::Point2f>> chessboardCornerLeft, chessboardCornerRight;
    cv::Size imgSize = cv::imread(leftList[0]).size();

    for (int index = 0; index < leftList.size(); index++) {
        cv::Mat left = cv::imread(leftList[index]);
        cv::Mat right = cv::imread(rightList[index], -1);
        cv::Mat leftgray,rightgray;
        
        //ir归一化到uint8
        cv::Mat right64f(right.size(), CV_64FC1);
        right64f = right.clone();
        double rightmax, rightmin;
        minMaxIdx(right64f, &rightmin, &rightmax);
        right64f = ((right64f - rightmin) / (rightmax - rightmin)) * 255;
        right64f.convertTo(rightgray, CV_8UC1);

        //rgb处理到单通道灰度
        cv::cvtColor(left, leftgray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point> corners_p_l, corners_p_r;
        std::vector<cv::Mat> chessboards_l, chessboards_r;
        CornerDetAC corner_detector_l(leftgray);
        CornerDetAC corner_detector_r(rightgray);
        ChessboradStruct chessboardstruct;
        Corners corners_s_l, corners_s_r;
        //找角点
        corner_detector_l.detectCorners(leftgray, corners_p_l, corners_s_l, 0.01);
        corner_detector_r.detectCorners(rightgray, corners_p_r, corners_s_r, 0.01);
        //从角点中找棋盘格
        chessboardstruct.chessboardsFromCorners(corners_s_l, chessboards_l, 0.6);
        chessboardstruct.chessboardsFromCorners(corners_s_r, chessboards_r, 0.6);
        sum++;
        if (chessboards_l[0].cols * chessboards_l[0].rows < sizeOfCheckerboard_h * sizeOfCheckerboard_w ||
            chessboards_r[0].cols * chessboards_r[0].rows < sizeOfCheckerboard_h * sizeOfCheckerboard_w)
            continue;
        else {
            if (chessboards_l[0].cols == sizeOfCheckerboard_h)
                chessboards_l[0] = chessboards_l[0].t();
            if (chessboards_r[0].cols == sizeOfCheckerboard_h)
                chessboards_r[0] = chessboards_r[0].t();
        }
        int chessboards_error_times = 0;
        
        sort:
        if (chessboards_error_times > 4) {
            std::cout << "loss chessboards" << std::endl;
            continue;
        }
        if (corners_s_l.p[chessboards_l[0].at<int>(0, 0)].y -
            corners_s_l.p[chessboards_l[0].at<int>(sizeOfCheckerboard_h - 1, sizeOfCheckerboard_w - 1)].y >= 0) {

            cv::flip(chessboards_l[0], chessboards_l[0], 0);
            std::cout << "chessboardserro0" << std::endl;
            chessboards_error_times++;
            goto sort;
        }
        if ((corners_s_l.p[chessboards_l[0].at<int>(0, 0)] -
             corners_s_l.p[chessboards_l[0].at<int>(sizeOfCheckerboard_h - 1, sizeOfCheckerboard_w - 1)]).x >= 0) {
            cv::flip(chessboards_l[0], chessboards_l[0], 1);
            std::cout << "chessboardserro1" << std::endl;
            chessboards_error_times++;
            goto sort;
        }
        if (corners_s_r.p[chessboards_r[0].at<int>(0, 0)].x -
            corners_s_r.p[chessboards_r[0].at<int>(sizeOfCheckerboard_h - 1, sizeOfCheckerboard_w - 1)].x >= 0) {
            cv::flip(chessboards_r[0], chessboards_r[0], 1);
            std::cout << "chessboardserro2" << std::endl;
            chessboards_error_times++;
            goto sort;
        }
        if (corners_s_r.p[chessboards_r[0].at<int>(0, 0)].y -
            corners_s_r.p[chessboards_r[0].at<int>(sizeOfCheckerboard_h - 1, sizeOfCheckerboard_w - 1)].y >= 0) {

            cv::flip(chessboards_r[0], chessboards_r[0], 0);
            std::cout << "chessboardserro3" << std::endl;
            chessboards_error_times++;
            goto sort;
        }
        if (corners_s_l.p[chessboards_l[0].at<int>(0, 0)].y >=
            corners_s_l.p[chessboards_l[0].at<int>(sizeOfCheckerboard_h - 1, 0)].y) {
            std::cout << "chessboardserro4" << std::endl;
            cv::flip(chessboards_l[0], chessboards_l[0], 0);
            chessboards_error_times++;
            goto sort;
        }
        if (corners_s_r.p[chessboards_r[0].at<int>(0, 0)].y >=
            corners_s_r.p[chessboards_r[0].at<int>(sizeOfCheckerboard_h - 1, 0)].y) {
            std::cout << "chessboardserro5" << std::endl;
            cv::flip(chessboards_r[0], chessboards_r[0], 0);
            chessboards_error_times++;
            goto sort;
        }
        if (corners_s_l.p[chessboards_l[0].at<int>(0, 0)].x >=
            corners_s_l.p[chessboards_l[0].at<int>(0, sizeOfCheckerboard_w - 1)].x) {
            std::cout << "chessboardserro6" << std::endl;
            cv::flip(chessboards_l[0], chessboards_l[0], 1);
            chessboards_error_times++;
            goto sort;
        }
        if (corners_s_r.p[chessboards_r[0].at<int>(0, 0)].x >=
            corners_s_r.p[chessboards_r[0].at<int>(0, sizeOfCheckerboard_w - 1)].x) {
            std::cout << "chessboardserro7" << std::endl;
            cv::flip(chessboards_r[0], chessboards_r[0], 1);
            chessboards_error_times++;
            goto sort;
        }

        std::vector<cv::Point2f> vcbl, vcbr;
        cv::Point2f lastleftpoint = corners_s_l.p[chessboards_l[0].at<int>(0, 0)];
        cv::Point2f lastrightpoint = corners_s_r.p[chessboards_r[0].at<int>(0, 0)];
        for (int i = 0; i < sizeOfCheckerboard_w; i++)
            for (int j = 0; j < sizeOfCheckerboard_h; j++) {
                if (chessboards_l[0].cols > i && chessboards_l[0].rows > j) {
                    vcbl.push_back(corners_s_l.p[chessboards_l[0].at<int>(j, i)]);
                    cv::circle(leftgray, corners_s_l.p[chessboards_l[0].at<int>(j, i)], 3,
                               cv::Scalar(i * 20, 255, j * 20), -1);
                    cv::line(leftgray, lastleftpoint, corners_s_l.p[chessboards_l[0].at<int>(j, i)], 255, 1);
                    lastleftpoint = corners_s_l.p[chessboards_l[0].at<int>(j, i)];
                }
                if (chessboards_r[0].cols > i && chessboards_r[0].rows > j) {
                    vcbr.push_back(corners_s_r.p[chessboards_r[0].at<int>(j, i)]);
                    cv::circle(rightgray, corners_s_r.p[chessboards_r[0].at<int>(j, i)], 3,
                               cv::Scalar(i * 20, 255, j * 20), -1);
                    cv::line(rightgray, lastrightpoint, corners_s_r.p[chessboards_r[0].at<int>(j, i)], 255, 1);
                    lastrightpoint = corners_s_r.p[chessboards_r[0].at<int>(j, i)];
                }
            }
        chessboardCornerLeft.push_back(vcbl);
        chessboardCornerRight.push_back(vcbr);

        found++;
        std::cout << "found num : all num " << found << " : " << sum << std::endl;

        cv::imshow("ir",rightgray);
        cv::imshow("rgb",leftgray);
        cv::waitKey(0);

    }
    std::vector<std::vector<cv::Point3f> > objectPoints;
    std::vector<cv::Point3f> opt;
    for (int i = 0; i <sizeOfCheckerboard_w; i++) {
        for (int j = 0; j < sizeOfCheckerboard_h; j++) {
            cv::Point3f p3f(i * 30, j * 30, 0);
            opt.push_back(p3f);
        }
    }
    for (int i = 0; i < found; i++)
        objectPoints.push_back(opt);
    cv::Mat cm1, cm2, dst1, dst2, R, T, E, F;
    cm1 = initCameraMatrix2D(objectPoints, chessboardCornerLeft, imgSize, 0);
    cm2 = initCameraMatrix2D(objectPoints, chessboardCornerRight, imgSize, 0);

    std::cout << "Init Camera Matrix:" << std::endl;
    std::cout << "M1" << std::endl << cm1 << std::endl;
    std::cout << "M2" << std::endl << cm2 << std::endl;

    cv::stereoCalibrate(objectPoints, chessboardCornerLeft, chessboardCornerRight,
                        cm1, dst1, cm2, dst2, imgSize, R, T, E, F,
                        cv::CALIB_FIX_ASPECT_RATIO +
                        cv::CALIB_ZERO_TANGENT_DIST +
                        cv::CALIB_USE_INTRINSIC_GUESS +
                        //CALIB_SAME_FOCAL_LENGTH +
                        cv::CALIB_RATIONAL_MODEL +
                        //CALIB_FIX_FOCAL_LENGTH +
                        cv::CALIB_FIX_K3 + cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5,
                        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-6));

    std::cout << "Calibration Output:" << std::endl;
    std::cout << "M1" << std::endl << cm1 << std::endl;
    std::cout << "M2" << std::endl << cm2 << std::endl;
    std::cout << "R:" << std::endl << R << std::endl;


    cv::FileStorage FS;
    // save camera data
    FS.open(intrinsicsfile, cv::FileStorage::WRITE);
    FS << "M1" << cm1;
    FS << "D1" << dst1;
    FS << "M2" << cm2;
    FS << "D2" << dst2;
    FS.release();

    FS.open(extrinsicsfile, cv::FileStorage::WRITE);
    FS << "R" << R;
    FS << "T" << T;
    FS << "E" << E;
    FS << "F" << F;
    FS.release();

    return 0;
}