#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <vector>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include<pcl/common/transforms.h>
#include<pcl/common/eigen.h>
#include <pcl/io/pcd_io.h>

#include "doPointCloud/pointDefinition.h"
#include <ros_orb_slam2/PointCloudWithKF.h>

using namespace std;

const double PI = 3.1415926;

pcl::PointCloud<pcl::PointXYZ>::Ptr surroundCloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr syncCloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>());

const int downSizeMap = 10;

ros::Publisher *surroundCloudPubPointer = NULL;
vector<ros_orb_slam2::PointCloudWithKF> PCwithKF_vec;

void PCwithKFDataHandler(const ros_orb_slam2::PointCloudWithKFConstPtr& msg)
{
  PCwithKF_vec.push_back(*msg);
  if(msg->KF_Pose.poses.size() == 1)
  {
    syncCloud->clear();
    tempCloud->clear();
    pcl::fromROSMsg(msg->pointcloud, *syncCloud);
    
    pcl::VoxelGrid<pcl::PointXYZ> downSizeFilter;
    downSizeFilter.setInputCloud(syncCloud);
    downSizeFilter.setLeafSize(0.3, 0.3, 0.3);
    downSizeFilter.filter(*tempCloud);
    syncCloud->clear();
    
    tf::Quaternion q_wc(msg->KF_Pose.poses[0].orientation.x,msg->KF_Pose.poses[0].orientation.y,
			    msg->KF_Pose.poses[0].orientation.z,msg->KF_Pose.poses[0].orientation.w);
    tf::Vector3 t_wc(msg->KF_Pose.poses[0].position.x,msg->KF_Pose.poses[0].position.y,msg->KF_Pose.poses[0].position.z);
    pcl::PointXYZ point;
    int tempCloudNum = tempCloud->points.size();
    for (int i = 0; i < tempCloudNum; i++) {
      point = tempCloud->points[i];
      double pointDis = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

      if (pointDis > 0.3 && pointDis < 30 && point.z<4.0) {
	tf::Vector3 p_wc = tf::Matrix3x3(q_wc) * tf::Vector3(point.x,point.y,point.z) + t_wc;
	point.x = p_wc.getX();
	point.y = p_wc.getY();
	point.z = p_wc.getZ();

	//syncCloud->push_back(point);
	surroundCloud->push_back(point);
      }
    }
    tempCloud->clear();
    
//     for(uint idx=0;idx<syncCloud->size();idx++){
//       surroundCloud->push_back(syncCloud->points[idx]);
//     }
    
    static int downSizeCount=0;
    downSizeCount++;
    
    if(downSizeCount >= downSizeMap){
      downSizeCount=0;
      tempCloud->clear();
      pcl::VoxelGrid<pcl::PointXYZ> downSizeMapFilter;
      downSizeMapFilter.setInputCloud(surroundCloud);
      downSizeMapFilter.setLeafSize(0.3,0.3,0.3);
      downSizeMapFilter.filter(*tempCloud);
      pcl::PointCloud<pcl::PointXYZ>::Ptr exchange=surroundCloud;
      surroundCloud=tempCloud;
      tempCloud=exchange;
    }

    sensor_msgs::PointCloud2 surroundCloud2;
    pcl::toROSMsg(*surroundCloud, surroundCloud2);
    surroundCloud2.header.frame_id = "odom"; 
    surroundCloud2.header.stamp = msg->KF_Pose.header.stamp;
    surroundCloudPubPointer->publish(surroundCloud2);
    }
    else{
      surroundCloud->clear();
      uint count=0;
      for(uint idx0=1;idx0<msg->KF_ID.data.size();idx0++){
	uint64_t KF_ID_L = msg->KF_ID.data[idx0];
	
	for(uint idx1=0;idx1<PCwithKF_vec.size();idx1++){
	  
	  if(KF_ID_L == PCwithKF_vec[idx1].KF_ID.data[0]){
	    count++;
	    
	    syncCloud->clear();
	    tempCloud->clear();
	    pcl::fromROSMsg(PCwithKF_vec[idx1].pointcloud, *syncCloud);
	    //------------------------whether do filting or not---------------------------
// 	    pcl::PointCloud<pcl::PointXYZ>::Ptr exchange=tempCloud;
// 	    tempCloud=syncCloud;
// 	    syncCloud=exchange;
	    
	    pcl::VoxelGrid<pcl::PointXYZ> downSizeFilter;
	    downSizeFilter.setInputCloud(syncCloud);
	    downSizeFilter.setLeafSize(0.15, 0.15, 0.15);
	    downSizeFilter.filter(*tempCloud);
	    syncCloud->clear();
	    //----------------------------------------------------------------------------
	    tf::Quaternion q_wc(msg->KF_Pose.poses[idx0].orientation.x,msg->KF_Pose.poses[idx0].orientation.y,
				    msg->KF_Pose.poses[idx0].orientation.z,msg->KF_Pose.poses[idx0].orientation.w);
	    tf::Vector3 t_wc(msg->KF_Pose.poses[idx0].position.x,msg->KF_Pose.poses[idx0].position.y,msg->KF_Pose.poses[idx0].position.z);
	    pcl::PointXYZ point;
	    int tempCloudNum = tempCloud->points.size();
	    for (int i = 0; i < tempCloudNum; i++) {
	      point = tempCloud->points[i];
	      double pointDis = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

	      if (pointDis > 0.3 && pointDis < 50 && point.z<9.0) {
		tf::Vector3 p_wc = tf::Matrix3x3(q_wc) * tf::Vector3(point.x,point.y,point.z) + t_wc;
		point.x = p_wc.getX();
		point.y = p_wc.getY();
		point.z = p_wc.getZ();
		surroundCloud->push_back(point);
	      }
	    }
	  }
	}
      }

      std::cout<<"\033[33m KF Size="<<msg->KF_ID.data.size()<<"!!!!!"<<"\033[0m"<<std::endl;
      std::cout<<"\033[33m KF Found="<<count<<"!!!!!"<<"\033[0m"<<std::endl;
      tempCloud->clear();
      pcl::VoxelGrid<pcl::PointXYZ> downSizeMapFilter;
      downSizeMapFilter.setInputCloud(surroundCloud);
      downSizeMapFilter.setLeafSize(0.15,0.15,0.15);
      downSizeMapFilter.filter(*tempCloud);
      surroundCloud->clear();
      
      string path = "/home/nrsl/orb_ws/PointCloudMap";
      static uint countPC = 0;
      countPC++;
      ostringstream oss;
      oss << path << countPC << ".pcd";
      pcl::io::savePCDFileASCII (oss.str(), *tempCloud);
      
      downSizeMapFilter.setInputCloud(tempCloud);
      downSizeMapFilter.setLeafSize(0.5,0.5,0.5);
      downSizeMapFilter.filter(*surroundCloud);
      tempCloud->clear();
//       pcl::PointCloud<pcl::PointXYZ>::Ptr exchange=surroundCloud;
//       surroundCloud=tempCloud;
//       tempCloud=exchange;

      sensor_msgs::PointCloud2 surroundCloud2;
      pcl::toROSMsg(*surroundCloud, surroundCloud2);
      surroundCloud2.header.frame_id = "odom"; 
      surroundCloud2.header.stamp = msg->KF_Pose.header.stamp;
      surroundCloudPubPointer->publish(surroundCloud2);
    }
    
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloudRegister");
  ros::NodeHandle nh;
				
  ros::Subscriber PCWithKFSub = nh.subscribe<ros_orb_slam2::PointCloudWithKF> ("/PointCloud_with_KF", 5, PCwithKFDataHandler);

  ros::Publisher surroundCloudPub = nh.advertise<sensor_msgs::PointCloud2> ("/point_cloud_map", 2);
  surroundCloudPubPointer = &surroundCloudPub;

  ros::spin();

  return 0;
}