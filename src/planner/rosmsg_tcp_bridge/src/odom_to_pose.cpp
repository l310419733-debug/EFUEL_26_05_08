#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

ros::Publisher pub_pose;

void cb(const nav_msgs::OdometryConstPtr& msg) {
  geometry_msgs::PoseStamped out;
  out.header = msg->header;
  out.pose = msg->pose.pose;
  pub_pose.publish(out);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "odom_to_pose");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/vins_fusion/camera_pose", 50, cb);
  pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/fuel/camera_pose", 50);

  ros::spin();
  return 0;
}
