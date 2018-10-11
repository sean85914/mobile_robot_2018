#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <iostream>

void cb(const std_msgs::Int16::ConstPtr& msg)
{
  std::cout << "message from Arduino is " << msg->data << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_node");
  ros::NodeHandle nh;
  std_msgs::Int16 in_msg;
  int in;
  ros::Publisher pub_in = nh.advertise<std_msgs::Int16>("input", 10);
  ros::Subscriber sub_out = nh.subscribe("out", 10, cb);
  ros::Duration(3.0).sleep();
  while (ros::ok()){
    std::cout << "user's input is ";
    std::cin >> in;
    in_msg.data = in;
    pub_in.publish(in_msg);
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }
}
