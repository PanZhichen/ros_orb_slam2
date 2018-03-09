#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include <System.h>   // from ORB_SLAM2
#include <Converter.h>
#include <doPointCloud/pointDefinition.h>

const double PI = 3.1415926;
const uint8_t IMAGE_SKIP = 3;

using namespace std;
ros::Publisher voPubliser;
tf::TransformBroadcaster *tfBroadcasterPointer = NULL;
pcl::PointCloud<pcl::PointXYZI>::Ptr depthCloud(new pcl::PointCloud<pcl::PointXYZI>());
nav_msgs::Odometry fresh_odom, curr_odom, last_odom;
bool INITIALIZED=false;
bool Refresh_DepthCloud = false;
cv::Mat Twl = cv::Mat::eye(4,4,CV_32F);

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    //void GrabImage(const sensor_msgs::ImageConstPtr& msgImag,const sensor_msgs::ImageConstPtr& msgDepth);
    void GrabImage(const sensor_msgs::ImageConstPtr& msgImag);

    ORB_SLAM2::System* mpSLAM;
};

void DepthCloudHandler(const sensor_msgs::PointCloud2ConstPtr& Cloud)
{
  depthCloud->clear();
  pcl::fromROSMsg(*Cloud, *depthCloud);
  Refresh_DepthCloud = true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_orb_mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    //0131ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);
    
    //************************************************************************************************************//
    //message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nodeHandler, "/camera/rgb/image_raw", 1);
    //message_filters::Subscriber<sensor_msgs::Image> depth_sub(nodeHandler, "camera/depth_registered/image_raw", 1);
    //typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    //message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    //sync.registerCallback(boost::bind(&ImageGrabber::GrabImage,&igb,_1,_2));
    
    ros::Subscriber CloudSub = nodeHandler.subscribe<sensor_msgs::PointCloud2>("/find_depth", 2, DepthCloudHandler);
    ros::Subscriber imageSub = nodeHandler.subscribe<sensor_msgs::Image>("/camera/image_raw", 2, &ImageGrabber::GrabImage,&igb);
    
    voPubliser = nodeHandler.advertise<nav_msgs::Odometry> ("/cam_to_odom", 5);
    
    tf::TransformBroadcaster tfBroadcaster;
    tfBroadcasterPointer = &tfBroadcaster;
    //************************************************************************************************************//

    ros::spin();
    
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msgImag)
{   
    static uint8_t ImageCount = 0;
    ImageCount = (ImageCount + 1) % IMAGE_SKIP;
    if(ImageCount != 0){
      return;
    } 
  
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrImage;
    try
    {
        cv_ptrImage = cv_bridge::toCvShare(msgImag);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat Tcw=mpSLAM->TrackMonocular(cv_ptrImage->image,depthCloud,cv_ptrImage->header.stamp.toSec(),Refresh_DepthCloud);
    Refresh_DepthCloud = false;
    
    if (Tcw.empty()) {
      return;
    }
    else{
      //如果在Tracking中失败，要把Tcw的（0,0）和（0,1）处的值置为99.99；
      if((Tcw.at<int>(0,0)==99)&&(Tcw.at<int>(0,1)==99))
      {
	std::cout<<"\033[31m Wrong Estimation!!"<<"\033[0m"<<std::endl;
        Tcw = cv::Mat::eye(4,4,CV_32F);
      }
    }
    
    cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
    cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

    vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);
    
    nav_msgs::Odometry voData;
    voData.header.frame_id = "odom";
    voData.child_frame_id = "camera";
    voData.header.stamp = cv_ptrImage->header.stamp;
    voData.pose.pose.orientation.x = q[0];
    voData.pose.pose.orientation.y = q[1];
    voData.pose.pose.orientation.z = q[2];
    voData.pose.pose.orientation.w = q[3];
    voData.pose.pose.position.x = twc.at<float>(0, 0);
    voData.pose.pose.position.y = twc.at<float>(0, 1);
    voData.pose.pose.position.z = twc.at<float>(0, 2);
    voData.twist.twist.angular.x = 0.0;
    voData.twist.twist.angular.y = 0.0;
    voData.twist.twist.angular.z = 0.0;
    voPubliser.publish(voData);
    
    tf::StampedTransform voTrans;
    voTrans.frame_id_ = "odom";
    voTrans.child_frame_id_ = "camera";
    voTrans.stamp_ = cv_ptrImage->header.stamp;
    voTrans.setRotation(tf::Quaternion(q[0], q[1], q[2], q[3]));
    voTrans.setOrigin(tf::Vector3(twc.at<float>(0, 0), twc.at<float>(0, 1), twc.at<float>(0, 2)));
    tfBroadcasterPointer->sendTransform(voTrans);

}

