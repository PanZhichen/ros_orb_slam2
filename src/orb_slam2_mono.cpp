#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/init.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <opencv2/core/core.hpp>

#include <System.h>   // from ORB_SLAM2
#include <Converter.h>

template <typename T>
T getParam(const ros::NodeHandle& nh, const std::string& param_name, T default_value)
{
  T value;
  if (nh.hasParam(param_name))
  {
    nh.getParam(param_name, value);
  }
  else
  {
    ROS_WARN_STREAM("Parameter '" << param_name << "' not found, defaults to '" << default_value << "'");
    value = default_value;
  }
  return value;
}

class ORBSLAM2Node {
private:
  ORB_SLAM2::System* mpSLAM;
  std::string worldframe, frame;
  tf::TransformBroadcaster br;

public:
  ORBSLAM2Node(ORB_SLAM2::System* pSLAM, std::string worldframe, std::string frame):mpSLAM(pSLAM),worldframe(worldframe),frame(frame) {

  }
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "orb_slam2_mono");
    ros::NodeHandle local_nh("~");

    std::string camera_src = getParam(local_nh, "camera", std::string("/camera/image"));
    std::string worldframe = getParam(local_nh, "worldframe", std::string("/world"));
    std::string frame = getParam(local_nh, "frame", std::string("orbframe"));
    std::string path_vocabulary = getParam(local_nh, "path_vocabulary", std::string(""));
    std::string path_settings = getParam(local_nh, "path_settings", std::string(""));
    bool use_viewer = getParam(local_nh, "use_viewer", false);

    if (path_vocabulary.empty()) {
      ROS_ERROR_STREAM("path_vocabulary is not provided.");
      return -1;
    }

    if (path_settings.empty()) {
      ROS_ERROR_STREAM("path_settings is not provided.");
      return -1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ros::NodeHandle nh;

    ORB_SLAM2::System SLAM(path_vocabulary.c_str(), path_settings.c_str(), ORB_SLAM2::System::MONOCULAR, use_viewer);
    ORBSLAM2Node orbslam2_node(&SLAM, worldframe, frame);

    image_transport::ImageTransport img_t(nh);
    image_transport::Subscriber sub = img_t.subscribe(camera_src, 1, &ORBSLAM2Node::imageCallback, &orbslam2_node);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();
    ros::shutdown();

    return 0;
}

void ORBSLAM2Node::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat Tcw = mpSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());
    if (Tcw.empty()) {
      return;
    }

    cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
    cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

    vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);

    const float MAP_SCALE = 1000.0f;

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(twc.at<float>(0, 0) * MAP_SCALE, twc.at<float>(0, 1) * MAP_SCALE, twc.at<float>(0, 2) * MAP_SCALE));
    tf::Quaternion quaternion(q[0], q[1], q[2], q[3]);
    transform.setRotation(quaternion);

    br.sendTransform(tf::StampedTransform(transform, ros::Time(cv_ptr->header.stamp.toSec()), worldframe, frame));
}
