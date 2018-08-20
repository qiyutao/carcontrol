#include "lane_detect_algo.h"

laneDetection::laneDetection()
{
    // cv::Point2f perspectiveSrc[] = {cv::Point2f(625, 195), cv::Point2f(780, 195), cv::Point2f(220, 595), cv::Point2f(710, 595)};
    // cv::Point2f perspectiveDst[] = {cv::Point2f(300, 0), cv::Point2f(980, 0), cv::Point2f(300, 800), cv::Point2f(980, 800)};


    cv::Point2f perspectiveSrc[] = {cv::Point2f(502, 518), cv::Point2f(702, 518), cv::Point2f(267, 695), cv::Point2f(882, 695)};
    cv::Point2f perspectiveDst[] = {cv::Point2f(300, 0), cv::Point2f(980, 0), cv::Point2f(300, 800), cv::Point2f(980, 800)};

    perspectiveMatrix = getPerspectiveTransform(perspectiveSrc, perspectiveDst);

    recordCounter = 0;
    initRecordCount = 0;
    failDetectFlag = false;
    frameRecord = 5;

    histogram.resize(WIDTH);
    //左移1位=x/2
    midPoint = WIDTH >> 1;
    midHeight = HEIGHT * 0.55;
    stepY = HEIGHT / blockNum;
    Eigen::Vector3d initV;
    initV << 0, 0, 0;
    for (int i = 0; i < frameRecord; i++)
    {
        curveCoefRecordL[i] = initV;
    }

    imageCenter = WIDTH / 2;
}

laneDetection::~laneDetection() = default;

//The core of lane detection algorithm.
void laneDetection::laneDetctAlgo()
{
    cv::resize(oriImage, resize_img, cv::Size(WIDTH, HEIGHT));
    warpPerspective(resize_img, warpOriImage, perspectiveMatrix, cv::Size(WIDTH, HEIGHT));  // wrapOrimage: for show on the screen 

    //Detect features.
    featureDetect();

    colorLane = cv::Scalar(255, 0, 0);
    calHist();
    boundaryDetection();
    //Lane curve fitting.
    laneSearch(leftLanePos, laneL, laneLcount, curvePointsL, 'L');
    laneSearch(rightLanePos, laneR, laneRcount, curvePointsR, 'R');
    laneCoefEstimate();
    // colorLane = cv::Scalar(255, 0, 0);
    laneFitting();
    warpPerspective(maskImage, maskImageWarp, perspectiveMatrix, maskImage.size(), cv::WARP_INVERSE_MAP);

}

void laneDetection::featureDetect()
{

    cvtColor(resize_img, wrapImageGray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(wrapImageGray, deNoise, cv::Size(5, 5), 0, 0);

    Canny(deNoise, warpEdgeImage, 20, 80, 3);
    inRange(warpEdgeImage, cv::Scalar(1), cv::Scalar(255), edgeImage);;

    // // cv::Mat RedBinary;
    // vector<cv::Mat> imageChannels;
    // split(resize_img, imageChannels);
    // inRange(imageChannels[2], cv::Scalar(200), cv::Scalar(255), RedBinary);

    // // Merge the binarized R channel image with edge detected image.
    // add(edgeImage, RedBinary, mergeImage);

    warpPerspective(edgeImage, mergeImage, perspectiveMatrix, cv::Size(1280, 800));
    // warpPerspective(mergeImage, mergeImage, perspectiveMatrix, cv::Size(1280, 800));

    vector<cv::Vec4i> lines;
    HoughLinesP(mergeImage, lines, 1, CV_PI/180, 10, 10, 10);
    drawHoughLines(mergeImage, lines);

    cvtColor(mergeImage, mergeImageRGB, CV_GRAY2RGB);

}

void laneDetection::drawHoughLines(cv::Mat &frame, vector<cv::Vec4i> lines) {
    for(size_t i = 0; i < lines.size(); i++) {
        cv::Vec4i l = lines[i];
        float angle = atan2(l[3] - l[1], l[2] - l[0]) * 180.0 / CV_PI;

        if (angle < -10 || angle > 10) {
            cv::line(frame, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255, 0, 0), 1, 4);
        }
    }
}

//Calculate the histogram of the lane features along x axis.
void laneDetection::calHist()
{
    histogram.clear();
    for (int i = 0; i < mergeImage.size().width; i++)
    {
        cv::Mat ROI = mergeImage(cv::Rect(i, HEIGHT - midHeight - 1, 1, midHeight));    // define the lower part for the ROI
        cv::Mat dst;
        divide(255, ROI, dst);
        histogram.push_back((int)(sum(dst)[0]));
    }
}

//Detect the lane boundary.
void laneDetection::boundaryDetection()
{
    //find the left lane boundary position
    vector<int>::iterator maxPtr ,maxPtr1, maxPtr2;
    int l1, l2;
    maxPtr = max_element(histogram.begin(), histogram.begin() + midPoint - 1);
    leftLanePos = distance(histogram.begin(), maxPtr);

    maxPtr = max_element(histogram.begin() + midPoint, histogram.end());
    rightLanePos = distance(histogram.begin(), maxPtr);

}

// if find missing detection to handle
void laneDetection::missDetect(int info) {
    ROS_INFO("%d\n", info);
    colorLane = cv::Scalar(0, 0, 255);
    failDetectFlag = true;
    initRecordCount = 0;
    Eigen::Vector3d frame_null;
    frame_null << 0, 0, 0;
    for(int i=0; i< frameRecord; i++) {
        curveCoefRecordL[i] = frame_null;
        curveCoefRecordR[i] = frame_null; 
    }
    recordCounter = 0;
}


void laneDetection::localSearch(const int &lanePos, vector<cv::Point2f> &_line, int &lanecount,
                               vector<cv::Point2f> &curvePoints, char dir)
{

    //TODO: right thing reduce

    //Lane search.
    const int skipStep = 2;
    int nextPosX;
    // int nextPosX = lanePos;
    int xLU = 0, yLU = 0; // left up point
    int xRB = 0, yRB = 0; // right bottom point
    int sumX = 0;
    int xcounter = 0;
    lanecount = 0;

    nextPosX = lanePos;

    for (int i = 0; i < blockNum; i++)
    {

        xLU = nextPosX - (windowSize >> 1); //The x coordinate of the upper left point.
        yLU = stepY * (blockNum - i - 1);   // The y coordinate of the upper left point.
        // xRB = xLU + windowSize;             //The x coordinate of the bottom right point.
        xRB = nextPosX + (windowSize >> 1);
        yRB = yLU + stepY - 1;              //The y coordinate of the bottom right point.
        // Avoid marginal effect.
        if ((xLU < 0))
        {
            xLU = 0;
            // xRB = xLU + windowSize;
        }
        if (xRB > (mergeImage.size().width - 1))
        {
            xRB = (mergeImage.size().width - 1);
            // xLU = xRB - windowSize;
        }
        if (xRB - xLU > 0 && xRB >= 0 && xLU >= 0)
        {
            //Detect the samples inside the window.
            sumX = 0;
            xcounter = 0;
            uchar *matPtr;
            for (int j = yLU; j <= yRB; j += skipStep)
            {
                matPtr = mergeImage.data + (j * mergeImage.size().width);
                for (int k = xLU; k <= xRB; k += skipStep)
                {
                    if (*(matPtr + k) == 255)
                    {
                        sumX += k;
                        xcounter++;
                    }
                }
            }
            if (xcounter != 0)
                sumX /= xcounter; //the average x coordinate inside the window.
            else
                sumX = nextPosX;

            //Modified the window position based on previous calculated average x coodinate.
            nextPosX = sumX;
            xLU = ((nextPosX - (windowSize >> 1)) > 0) ? (nextPosX - (windowSize >> 1)) : 0;
            xRB = ((nextPosX + windowSize) < (mergeImage.size().width)) ? (nextPosX + (windowSize >> 1)) : (mergeImage.size().width - 1);
            // xRB = ((nextPosX + 20) < (mergeImage.size().width)) ? (nextPosX + 20) : (mergeImage.size().width - 1);
            if (xRB - xLU > 0 && xRB <= mergeImage.size().width && xLU >= 0)
            {
                for (int j = yLU; j <= yRB; j += skipStep)
                {
                    matPtr = mergeImage.data + (j * mergeImage.size().width);
                    for (int k = xLU; k <= xRB; k += skipStep)
                    {
                        if (*(matPtr + k) == 255)
                        {
                            lanecount++;
                            _line.push_back(cv::Point2f(k, j));
                        }
                    }
                }
            }
        }
    }
    cout << nextPosX <<"lane"<< endl;

            if(dir=='L'){
                cout << "L: " << leftLanePos << endl;
            }
            else
                cout << "R: " << rightLanePos << endl;
}

//Fitting the lane curve.
void laneDetection::laneSearch(const int &lanePos, vector<cv::Point2f> &_line, int &lanecount,
                               vector<cv::Point2f> &curvePoints, char dir)
{
   
    _line.clear();


    if ((initRecordCount < frameRecord) || (failDetectFlag==true)) //Conduct full search.
    {
        // calHist();

        // boundaryDetection();

        localSearch(lanePos, _line, lanecount, curvePoints, dir);
    }
    else //Conduct search based on previous results.
    {

    const int skipStep = 2;
    // int nextPosX;
    int nextPosX = lanePos;
    int sumX = 0;
    int xcounter = 0;
    lanecount = 0;
        uchar* matPtr;
        int xtemp;
        for(int i=0; i<mergeImage.size().height; i++)
        {
            matPtr = mergeImage.data + (i*mergeImage.size().width);
            for(int j=-150; j<=150; j+=2)
            {
                xtemp = (curvePoints[i].x + j);
                if(xtemp>=0 && xtemp<mergeImage.size().width)
                {
                    if(*(matPtr+xtemp) == 255)
                    {
                        lanecount++;
                        _line.push_back(cv::Point2f(xtemp,i));
                        if(i<=(mergeImage.size().height/2))
                        {
                            sumX += xtemp;
                            xcounter++;
                        }
                       

                    }
                }
            }
        }
        if (xcounter!=0) sumX /= xcounter;
        else sumX = nextPosX;



        // if((sumX > 0) && (sumX < mergeImageRGB.size().width))
        // {
        //     if(dir == 'L')
        //     {
        //         leftLanePos = sumX;
        //     }
        //     else
        //     {
        //         rightLanePos = sumX;
        //     }
        // }
        
    }
}

//Using SVD to solve the coefficients of the curve.
bool laneDetection::laneCoefEstimate()
{

    //To fitting the lance curve by using least square method
    // 重要参数：进行多项式拟合的检测特征点阈值
    // int countThreshold = 3000;
    int countThreshold = 1000;

    cout <<"-----laneLcount: "<<laneLcount<< endl;
    cout <<"-----laneRcount: "<< laneRcount << endl;

    if ((laneLcount > countThreshold) && (laneRcount > countThreshold))
    {
        Eigen::VectorXd xValueL(laneLcount);
        Eigen::VectorXd xValueR(laneRcount);
        Eigen::MatrixXd leftMatrix(laneLcount, 3);
        Eigen::MatrixXd rightMatrix(laneRcount, 3);

        //left lane curve coefficients estimation
        for (int i = 0; i < laneLcount; i++)
        {
            xValueL(i) = laneL[i].x;
            leftMatrix(i, 0) = pow(laneL[i].y, 2);
            leftMatrix(i, 1) = laneL[i].y;
            leftMatrix(i, 2) = 1;
        }

        //right lane curve coefficients estimation
        for (int i = 0; i < laneRcount; i++)
        {
            xValueR(i) = laneR[i].x;
            rightMatrix(i, 0) = pow(laneR[i].y, 2);
            rightMatrix(i, 1) = laneR[i].y;
            rightMatrix(i, 2) = 1;
        }
        curveCoefL = (leftMatrix.transpose() * leftMatrix).ldlt().solve(leftMatrix.transpose() * xValueL);
        curveCoefR = (rightMatrix.transpose() * rightMatrix).ldlt().solve(rightMatrix.transpose() * xValueR);

        curveCoefRecordL[recordCounter] = curveCoefL;
        curveCoefRecordR[recordCounter] = curveCoefR;
        recordCounter = (recordCounter + 1) % frameRecord;
        if (initRecordCount < frameRecord)
            initRecordCount++;
        failDetectFlag = false;
        return true;
    }
    else
    {
        missDetect(1);
        
        return false;
    }
}

//To fit the lane.
void laneDetection::laneFitting()
{
    maskImage.create(mergeImage.size().height, mergeImage.size().width, CV_8UC3);
    maskImage = cv::Scalar(0, 0, 0);
    curvePointsL.clear();
    curvePointsR.clear();

    //To average the past frameRecord estimated coefficients.
    if (initRecordCount == frameRecord)
    {
        int count = 0;
        curveCoefL *= 0;
        curveCoefR *= 0;
        for (int i=0; i<frameRecord; i++) {
            curveCoefL += curveCoefRecordL[i];
            curveCoefR += curveCoefRecordR[i];

            cout<<"old: "<<curveCoefRecordL[i](2)<<" ";
            count+=curveCoefRecordL[i](2);
        }
        cout<<endl;
        cout<<"count: "<<count/5<<"  count/5:"<<count/frameRecord<<endl;
        curveCoefL /= frameRecord;
        curveCoefR /= frameRecord;
         cout<<"after: "<<curveCoefL(2)<<endl;
    }

    

    int xL, xR;
    for (int i = 0; i < mergeImage.size().height; i++)
    {
        xL = pow(i, 2) * curveCoefL(0) + i * curveCoefL(1) + curveCoefL(2);
        xR = pow(i, 2) * curveCoefR(0) + i * curveCoefR(1) + curveCoefR(2);
        if (xL < 0)
            xL = 0;
        if (xL >= mergeImage.size().width)
            xL = mergeImage.size().width - 1;
        if (xR < 0)
            xR = 0;
        if (xR >= mergeImage.size().width)
            xR = mergeImage.size().width - 1;
        //TODO
        curvePointsL.push_back(cv::Point2f(xL, i));
        curvePointsR.push_back(cv::Point2f(xR, i));


        
    }
    
    float slopeL = 1.0/2*curveCoefL(0) * leftLanePos - curveCoefL(1)/2*curveCoefR(0);
    float slopeR = 1.0/2*curveCoefR(0) * rightLanePos - curveCoefR(1)/2*curveCoefR(0);
    double thresh_slope = 0.5;  //两条车道线切线斜率相差阙值
    if (abs(slopeL-slopeR) > thresh_slope) {
        missDetect(2);
    }
    else {
        float slopeRR = -1.0/curveCoefR(1);
        float slopeLL = -1.0/curveCoefL(1);
        double slope = (slopeLL+slopeRR) / 2.0;
        slope = atan(slope);
        double angle = slope * 180.0 / M_PI;
        if (angle < 0)  angle = 90 + angle;
        else    angle = -90 + angle;
        // cout << "angle: " << angle << "°" << endl;
    }

    leftLanePos = curvePointsL.back().x;
    rightLanePos = curvePointsR.back().x;

    // detect the width of the real road
    if ((rightLanePos - leftLanePos)< 500) {
        missDetect(3);
    }

    
    cv::UMat curveL(curvePointsL, true);
    curveL.convertTo(curveL, CV_32S);
    cv::polylines(maskImage, curveL, false, colorLane, 50, CV_AA);
    cv::UMat curveR(curvePointsR, true);
    curveR.convertTo(curveR, CV_32S);
    cv::polylines(maskImage, curveR, false, colorLane, 50, CV_AA);

    line(maskImage, cv::Point2f(leftLanePos, 0), cv::Point2f(leftLanePos, mergeImageRGB.size().height),
         cv::Scalar(0, 255, 0), 5);
    line(maskImage, cv::Point2f(rightLanePos, 0), cv::Point2f(rightLanePos, mergeImageRGB.size().height),
         cv::Scalar(0, 255, 0), 5);
    // draw the center line
    line(maskImage, cv::Point2f(imageCenter, 0), cv::Point2f(imageCenter, mergeImageRGB.size().height),
         cv::Scalar(0, 255, 0), 5);
}


cv::UMat laneDetection::getFinalResult()
{
    addWeighted(warpOriImage, 0.5, maskImage, 1, 0, finalResult);
    return finalResult;
    // return mergeImageRGB;
}

void laneDetection::setInputImage(cv::UMat &image)
{
    oriImage = image.clone();
}

vector<float> laneDetection::getLaneCenterDist()
{
    vector<float> result;

    float laneCenter = ((rightLanePos - leftLanePos) / 2) + leftLanePos;
    float base = 2.80 / 1240.0;

    float dis_to_center = (laneCenter - imageCenter) * base; 
    float dis_to_left = (imageCenter - leftLanePos) * base;
    float dis_to_right = (rightLanePos - imageCenter) * base;

    result.push_back(dis_to_center);
    result.push_back(dis_to_left);
    result.push_back(dis_to_right);
    return result;
}

cv::UMat laneDetection::getMergeImage() {
    return mergeImageRGB;
}