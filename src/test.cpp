#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

int main(int argc, char **argv)
{
    cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
    Tcw.at<float>(0,3)=7.0;
    Tcw.at<float>(1,3)=8.0;
    Tcw.at<float>(2,3)=9.0;
    cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
    cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
    std::cout<<twc<<std::endl;
    //std::cout<<twc.at<float>(0, 1)<<std::endl;
    //std::cout<<twc.at<float>(1, 0)<<std::endl;
    //std::cout<<twc.at<float>(2, 0)<<std::endl;
    
    cv::Mat p = cv::Mat::zeros(4,1,CV_32F);
    p.at<float>(0,0) = 42;
    p.at<float>(0,1) = 30;
    p.at<float>(0,2) = 21;
    p.at<float>(0,3) = 1;
    std::cout<<Tcw*p<<std::endl;
}