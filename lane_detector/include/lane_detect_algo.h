//
// Created by adam on 18-7-17.
//

#ifndef LANE_DETECT_LANEDETECTION_H
#define LANE_DETECT_LANEDETECTION_H

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>
#include "ros/ros.h"
#include "math.h"

using namespace std;

class laneDetection
{
  private:
    int WIDTH = 1280;
    int HEIGHT = 800;

    cv::Mat cameraMatrix, dist; //Calibration Matrix.
    cv::Mat perspectiveMatrix;  //Homography Matrix.

    cv::UMat resize_img;
    cv::UMat oriImage; //The original input image.
    cv::UMat wrapImageGray;
    cv::UMat deNoise;
    cv::UMat edgeImage; // The result of applying canny edge detection.
    cv::UMat warpEdgeImage;
    cv::UMat warpOriImage;

    cv::Scalar colorLane = cv::Scalar(255, 0, 0);

    vector<cv::UMat> s_channel;
    vector<cv::UMat> b_channel;
    cv::UMat sobel_binary;
    cv::UMat s_binary;
    cv::UMat b_binary;
    cv::UMat combined_binary;
    cv::UMat sobel;
    cv::UMat s_img;
    cv::UMat b_img;

    vector<cv::UMat> imageChannels;
    cv::UMat RedBinary;
    cv::Mat mergeImage;
    cv::UMat mergeImageRGB;
    cv::UMat histImage; //Histogram visualization.
    cv::UMat maskImage; //The curve image used to blend to the original image.
    cv::UMat maskImageWarp;
    cv::UMat finalResult;
    vector<int> histogram; //Histogram of the detected features
    vector<cv::Point2f> laneL;
    vector<cv::Point2f> laneR;
    vector<cv::Point2f> curvePointsL;
    vector<cv::Point2f> curvePointsR;
    int laneLcount;
    int laneRcount;
    float imageCenter;
    
    int midPoint; //The mid position of the view.
    //重要参数：ROI区域设置
    int midHeight;
    int leftLanePos;                     //The detected left lane boundary position.
    int rightLanePos;                    //The detected right lane boundary position.
    short initRecordCount;               // To record the number of times of executions in the first 5 frames.
    const int blockNum = 9;              //Number of windows per line.
    int stepY;                           //Window moving step.
    const int windowSize = 150;          //Window Size (Horizontal).
    Eigen::Vector3d curveCoefL;          //The coefficients of the curve (left).
    Eigen::Vector3d curveCoefR;          //The coefficients of the curve (left).
    Eigen::Vector3d curveCoefRecordL[5]; //To keep the last five record to smooth the current coefficients (left).
    Eigen::Vector3d curveCoefRecordR[5]; //To keep the last five record to smooth the current coefficients (right).
    int frameRecord;
    int recordCounter;
    bool failDetectFlag; // To indicate whether the road marks is detected succesfully.
    bool averageFirstFlag = false;

    void featureDetect();
    void drawHoughLines(cv::Mat &frame, vector<cv::Vec4i> lines);
    void calHist();
    void boundaryDetection();
    void missDetect(int info);
    void laneSearch(const int &lanePos, vector<cv::Point2f> &_line, int &lanecount, vector<cv::Point2f> &curvePoints, char dir);
    void localSearch(const int &lanePos, vector<cv::Point2f> &_line, int &lanecount, vector<cv::Point2f> &curvePoints, char dir);
    
    bool laneCoefEstimate();
    void laneFitting();

  public:
    laneDetection();
    ~laneDetection();

    void laneDetctAlgo();
    cv::UMat getEdgeDetectResult();
    cv::UMat getWarpEdgeDetectResult();
    cv::UMat getRedChannel();
    cv::UMat getRedBinary();
    cv::UMat getMergeImage();
    cv::UMat getHistImage();
    cv::UMat getMaskImage();
    cv::UMat getWarpMask();
    cv::UMat getFinalResult();
    vector<float> getLaneCenterDist();
    void setInputImage(cv::UMat &image);
};

#endif //LANE_DETECT_LANEDETECTION_H
