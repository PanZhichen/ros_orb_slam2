#include <cstdlib>
#include <cstdio>
#include <math.h>
#include <algorithm>
#include <map>
#include<ros/ros.h>
#include<ros/package.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl_ros/point_cloud.h>
#include<pcl/common/transforms.h>
#include<pcl/common/eigen.h>

ros::Publisher TransPointPub;

struct vlpTocamera
{
  float x,y,z,roll,pitch,yaw;
}vlptocamera;
pcl::PointCloud<pcl::PointXYZ>::Ptr vlp_PC(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr trans_vlp_PC(new pcl::PointCloud<pcl::PointXYZ>());
void readConfig()
{
	std::string pkg_loc = ros::package::getPath("ros_orb_slam2");
	std::ifstream infile(pkg_loc + "/conf/config_file.txt");

	infile >> vlptocamera.x >> vlptocamera.y >> vlptocamera.z >> vlptocamera.roll
	       >> vlptocamera.pitch >> vlptocamera.yaw ;

	infile.close();
}

void PointCloudHandler(const sensor_msgs::PointCloud2ConstPtr& msg_pc)
{
  vlp_PC->clear();
  trans_vlp_PC->clear();
  pcl::fromROSMsg(*msg_pc, *vlp_PC);
  
  Eigen::Affine3f transf = pcl::getTransformation(vlptocamera.x,vlptocamera.y,vlptocamera.z,
						  vlptocamera.roll,vlptocamera.pitch,vlptocamera.yaw);
  pcl::transformPointCloud(*vlp_PC, *trans_vlp_PC, transf);
  
  vlp_PC->clear();
  pcl::PointXYZ temp;
  for(u_int i=0; i<trans_vlp_PC->points.size(); i++)
  {
    if(trans_vlp_PC->points[i].z >= 0)
    {
      temp.x=trans_vlp_PC->points[i].x;
      temp.y=trans_vlp_PC->points[i].y;
      temp.z=trans_vlp_PC->points[i].z;
      vlp_PC->points.push_back(temp);
    }
  }
  
  sensor_msgs::PointCloud2 trans_pc;
  pcl::toROSMsg(*vlp_PC,trans_pc);
  //trans_pc.header.frame_id="/velodyne";
  trans_pc.header.frame_id="/camera_init";
  trans_pc.header.stamp=msg_pc->header.stamp;
  TransPointPub.publish(trans_pc);
}

int main(int argc, char** argv)
{
  readConfig();
  
  ros::init(argc, argv, "trans_vlp");
  ros::NodeHandle n;
  
  ros::Subscriber PointCloudSub = n.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points", 1, PointCloudHandler);
  TransPointPub = n.advertise<sensor_msgs::PointCloud2>("/sync_scan_cloud_filtered",5);
  
  ros::spin();
}
