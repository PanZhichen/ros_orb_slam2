#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include<pcl/common/transforms.h>
#include<pcl/common/eigen.h>

#include "doPointCloud/pointDefinition.h"

const double PI = 3.1415926;

const int keepVoDataNum = 30;
double voDataTime[keepVoDataNum] = {0};
double voRx[keepVoDataNum] = {0};
double voRy[keepVoDataNum] = {0};
double voRz[keepVoDataNum] = {0};
double voTx[keepVoDataNum] = {0};
double voTy[keepVoDataNum] = {0};
double voTz[keepVoDataNum] = {0};
int voDataInd = -1;
int voRegInd = 0;

pcl::PointCloud<pcl::PointXYZ>::Ptr surroundCloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr syncCloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>());

int startCount = -1;
const int startSkipNum = 5;

int showCount = -1;
const int showSkipNum = 15;
const int downSizeMap = 10;

ros::Publisher *surroundCloudPubPointer = NULL;

void KFDataHandler(const geometry_msgs::PoseArray::ConstPtr& voData)
{
  double time = voData->header.stamp.toSec();

  double rx, ry, rz;
  geometry_msgs::Quaternion geoQuat = voData->pose.pose.orientation;
  //下面做点云转换的时候rx和ry取了个负号，因此这里先将rx和ry取为原来的负值
  tf::Matrix3x3(tf::Quaternion(-geoQuat.x, -geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(rx, ry, rz);

  double tx = voData->pose.pose.position.x;
  double ty = voData->pose.pose.position.y;
  double tz = voData->pose.pose.position.z;

  voDataInd = (voDataInd + 1) % keepVoDataNum;
  voDataTime[voDataInd] = time;
  voRx[voDataInd] = rx;
  voRy[voDataInd] = ry;
  voRz[voDataInd] = rz;
  voTx[voDataInd] = tx;
  voTy[voDataInd] = ty;
  voTz[voDataInd] = tz;
}

void syncCloudHandler(const sensor_msgs::PointCloud2ConstPtr& syncCloud2)
{
  if (startCount < startSkipNum) {
    startCount++;
    return;
  }

  showCount = (showCount + 1) % (showSkipNum + 1);
  if (showCount != showSkipNum) {
    return;
  }
  
  double time = syncCloud2->header.stamp.toSec();

  syncCloud->clear();
  tempCloud->clear();
  //modified at 2018/01/18: translate point cloud into the frame that has the same direction with the original VLP coordinate
  pcl::fromROSMsg(*syncCloud2, *tempCloud);
  Eigen::Affine3f transf = pcl::getTransformation(0.0,0.0,0.0,-1.57,-3.14,-1.57);
  pcl::transformPointCloud(*tempCloud, *syncCloud, transf);
  tempCloud->clear();
  /////////////////////////////////////////////
  double scaleCur = 1;
  double scaleLast = 0;
  int voPreInd = keepVoDataNum - 1;
  if (voDataInd >= 0) {
    while (voDataTime[voRegInd] <= time && voRegInd != voDataInd) {
      voRegInd = (voRegInd + 1) % keepVoDataNum;
    }

    voPreInd = (voRegInd + keepVoDataNum - 1) % keepVoDataNum;
    double voTimePre = voDataTime[voPreInd];
    double voTimeReg = voDataTime[voRegInd];

    if (voTimeReg - voTimePre < 0.5) {
      //modified at 2018/01/09
      //double scaleLast =  (voTimeReg - time) / (voTimeReg - voTimePre);
      //double scaleCur =  (time - voTimePre) / (voTimeReg - voTimePre);
      scaleLast =  (voTimeReg - time) / (voTimeReg - voTimePre);
      scaleCur =  (time - voTimePre) / (voTimeReg - voTimePre);
      if (scaleLast > 1) {
        scaleLast = 1;
      } else if (scaleLast < 0) {
        scaleLast = 0;
      }
      if (scaleCur > 1) {
        scaleCur = 1;
      } else if (scaleCur < 0) {
        scaleCur = 0;
      }
    }
  }

  double rx2 = voRx[voRegInd] * scaleCur + voRx[voPreInd] * scaleLast;
  double ry2;
  if (voRy[voRegInd] - voRy[voPreInd] > PI) {
    ry2 = voRy[voRegInd] * scaleCur + (voRy[voPreInd] + 2 * PI) * scaleLast;
  } else if (voRy[voRegInd] - voRy[voPreInd] < -PI) {
    ry2 = voRy[voRegInd] * scaleCur + (voRy[voPreInd] - 2 * PI) * scaleLast;
  } else {
    ry2 = voRy[voRegInd] * scaleCur + voRy[voPreInd] * scaleLast;
  }
  double rz2 = voRz[voRegInd] * scaleCur + voRz[voPreInd] * scaleLast;

  double tx2 = voTx[voRegInd] * scaleCur + voTx[voPreInd] * scaleLast;
  double ty2 = voTy[voRegInd] * scaleCur + voTy[voPreInd] * scaleLast;
  double tz2 = voTz[voRegInd] * scaleCur + voTz[voPreInd] * scaleLast;

  double cosrx2 = cos(rx2);
  double sinrx2 = sin(rx2);
  double cosry2 = cos(ry2);
  double sinry2 = sin(ry2);
  double cosrz2 = cos(rz2);
  double sinrz2 = sin(rz2);

  pcl::PointXYZ point;
  int syncCloudNum = syncCloud->points.size();
  for (int i = 0; i < syncCloudNum; i++) {
    point = syncCloud->points[i];
    double pointDis = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    //modified at 2018/01/11
    if (/*pointDis > 0.3 && pointDis < 5*/pointDis > 0.3 && pointDis < 15) {
      double x1 = cosrz2 * point.x - sinrz2 * point.y;
      double y1 = sinrz2 * point.x + cosrz2 * point.y;
      double z1 = point.z;

      double x2 = x1;
      double y2 = cosrx2 * y1 + sinrx2 * z1;
      double z2 = -sinrx2 * y1 + cosrx2 * z1;

      point.x = cosry2 * x2 - sinry2 * z2 + tx2;
      point.y = y2 + ty2;
      point.z = sinry2 * x2 + cosry2 * z2 + tz2;

      tempCloud->push_back(point); //modified at 2018/01/11
    }
  }

  syncCloud->clear();
  pcl::VoxelGrid<pcl::PointXYZ> downSizeFilter;
  downSizeFilter.setInputCloud(tempCloud);
  downSizeFilter.setLeafSize(0.3, 0.3, 0.3);
  downSizeFilter.filter(*syncCloud);
  
  for(int idx=0;idx<syncCloud->size();idx++){
    surroundCloud->push_back(syncCloud->points[idx]);
  }
  
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
  //modified at 2018/01/17
  surroundCloud2.header.frame_id = "camera_init"; //remove the leading slash
  surroundCloud2.header.stamp = syncCloud2->header.stamp;
  surroundCloudPubPointer->publish(surroundCloud2);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloudRegister");
  ros::NodeHandle nh;

  //modified at 2018/01/18
  ros::Subscriber KFPoseSub = nh.subscribe<geometry_msgs::PoseArray> ("/KeyFramePose", 1, KFDataHandler);

  ros::Subscriber pointCloudSub = nh.subscribe<sensor_msgs::PointCloud2>
                                ("/find_depth", 1, syncCloudHandler);

  ros::Publisher surroundCloudPub = nh.advertise<sensor_msgs::PointCloud2> ("/point_cloud_map", 1);
  surroundCloudPubPointer = &surroundCloudPub;

  ros::spin();

  return 0;
}