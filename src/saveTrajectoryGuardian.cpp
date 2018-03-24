#include<iostream>
#include<algorithm>
#include<fstream>
#include <stdlib.h> 
#include<chrono>

#include<ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

const double PI = 3.1415926;

using namespace std;

ros::Publisher guadianPubliser;
tf::TransformBroadcaster *tfBroadcasterPointer = NULL;
ofstream f;

void OdomHandler(const nav_msgs::OdometryConstPtr& msgOdom)
{
  tf::Quaternion q = tf::Quaternion(-PI/2,PI/2,0);//(-PI/2,PI/2,0.0);

  tf::Quaternion q_test(-msgOdom->pose.pose.orientation.y,-msgOdom->pose.pose.orientation.z,msgOdom->pose.pose.orientation.x,msgOdom->pose.pose.orientation.w);

  tf::Vector3 v_odom = tf::Vector3(msgOdom->pose.pose.position.x,msgOdom->pose.pose.position.y,msgOdom->pose.pose.position.z);
  
  nav_msgs::Odometry guardian;

  tf::Vector3 v_T = tf::Matrix3x3(q)*v_odom;
  guardian.header.frame_id = "odom";
  guardian.child_frame_id = "guardian";
  guardian.header.stamp = msgOdom->header.stamp;
  guardian.pose.pose.orientation.x = q_test.getX();
  guardian.pose.pose.orientation.y = q_test.getY();
  guardian.pose.pose.orientation.z = q_test.getZ();
  guardian.pose.pose.orientation.w = q_test.getW();
  guardian.pose.pose.position.x = v_T.getX();
  guardian.pose.pose.position.y = v_T.getY();
  guardian.pose.pose.position.z = v_T.getZ();
  guardian.twist.twist.angular.x = 0.0;
  guardian.twist.twist.angular.y = 0.0;
  guardian.twist.twist.angular.z = 0.0;
  guadianPubliser.publish(guardian);
  
  tf::StampedTransform voTrans;
  voTrans.frame_id_ = "odom";
  voTrans.child_frame_id_ = "camera";
  voTrans.stamp_ = msgOdom->header.stamp;
  voTrans.setRotation(q_test);
  voTrans.setOrigin(v_T);
  tfBroadcasterPointer->sendTransform(voTrans);
  
  f << setprecision(6) << msgOdom->header.stamp << setprecision(7) << " " << v_T.getX() << " " << v_T.getY() << " " << v_T.getZ()
    << " " << q_test.getX() << " " << q_test.getY() << " " << q_test.getZ() << " " << q_test.getW() << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "saveTrajectoryGuardian_node");
    
    string filename("/home/nrsl/orb_ws/GuardianTrajectory.txt");
    
    f.open(filename.c_str());
    f << fixed;
    
    ros::start();

    ros::NodeHandle nodeHandler;

    ros::Subscriber OdomSub = nodeHandler.subscribe<nav_msgs::Odometry>("/guardian_node/odom", 1, OdomHandler);
    
    guadianPubliser = nodeHandler.advertise<nav_msgs::Odometry> ("/guardian_odom", 5);
    
    tf::TransformBroadcaster tfBroadcaster;
    tfBroadcasterPointer = &tfBroadcaster;

    ros::spin();

    ros::shutdown();
    
    f.close();
    cout << endl << "trajectory saved!" << endl;

    return 0;
}




