#include <ros.h>
#include <std_msgs/Int16.h>

std_msgs::Int16 out_msg;
ros::Publisher output("out", &out_msg);

void cb(const std_msgs::Int16& msg){
  out_msg.data = msg.data * 2;
  output.publish(&out_msg);
}

ros::NodeHandle nh;

ros::Subscriber<std_msgs::Int16> sub("input", &cb);

void setup()
{
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(output);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
