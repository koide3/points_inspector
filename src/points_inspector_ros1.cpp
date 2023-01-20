#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#define ROS1
#include <points_inspector.hpp>

class PointsInspectorNode {
public:
  PointsInspectorNode() : nh("~") {
    ROS_INFO_STREAM("points_inspector");
    points_sub = nh.subscribe("/points", 10, &PointsInspectorNode::callback, this);
    ROS_INFO_STREAM("subscribing to " << points_sub.getTopic());
  }

private:
  void callback(const sensor_msgs::PointCloud2::ConstPtr& points_msg) { summarize(*points_msg); }

private:
  ros::NodeHandle nh;
  ros::Subscriber points_sub;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "points_inspector");
  PointsInspectorNode node;
  ros::spin();
  return 0;
}