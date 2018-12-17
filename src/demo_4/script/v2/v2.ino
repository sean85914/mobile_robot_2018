#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

/* 
  Note:
  motor side as front
  IN1 High, IN2 LOW  > forward
  IN1 LOW,  IN2 HIGH > backward
  IN3 HIGH, IN4 LOW  > forward
  IN3 LOW,  IN4 HIGH > backward
*/
 
/* 
  Sensor output:
    touch sensor: 1 if touch and 0 otherwise
    photo sensor: 0 if detect light and 1 otherwise
*/

// Left wheel
#define IN1 2
#define IN2 3
#define ENA 6
// Right wheel
#define IN3 4
#define IN4 5
#define ENB 9
// Sensors
#define PHOTO 11
#define TOUCH 10
#define IR 12

ros::NodeHandle nh;

// Motor pwm
int pwm_r = 0, pwm_l = 0;

// Parameters
int trim_ = -15;
int* door_frequency;
int count = 0;

// Sensor data
bool photo, touch, find_door;

double bound[4] = {0.20, 0.13, 0.34, 0.26}; // [0:2] -> 1500, [2:4] -> 600
int door_idx = 0; // 0 -> 1500, 1 -> 600
double ratio;
bool has_find = false;

// callback
void cb_r(const std_msgs::Int16& msg){
  pwm_r = msg.data;
  //nh.loginfo("pwm r update");
}

void cb_l(const std_msgs::Int16& msg){
  pwm_l = msg.data;
  //nh.loginfo("pwm l update");
} 

std_msgs::Float32 ratio_data;
std_msgs::String str_state;

// Subscribers and publishers
ros::Subscriber<std_msgs::Int16> sub_right("right_pwm", &cb_r);
ros::Subscriber<std_msgs::Int16> sub_left("left_pwm", &cb_l);
ros::Publisher pub_state("state", &str_state);
ros::Publisher pub_ratio("ratio", &ratio_data);

void setup() {
  // Init ROS node handler
  nh.initNode();
  //while (!nh.connected()) {nh.spinOnce();}
  //delay(10);
  // Subscribers
  nh.subscribe(sub_right);
  nh.subscribe(sub_left);
  // Publisher
  nh.advertise(pub_state);
  nh.advertise(pub_ratio);
  // Get parameters
  //getParameter();
  // Set pinmode
  pinMode(OUTPUT, IN1);
  pinMode(OUTPUT, IN2);
  pinMode(OUTPUT, IN3);
  pinMode(OUTPUT, IN4);
  pinMode(OUTPUT, ENA);
  pinMode(OUTPUT, ENB);
  pinMode(INPUT, PHOTO);
  pinMode(INPUT, TOUCH);
  pinMode(INPUT, IR);
  // Publish first state
  str_state.data = "pi";
  //pub_state.publish(&str_state);
}

void loop() {
  photo = digitalRead(PHOTO); // 1: not light
  touch = digitalRead(TOUCH);
  if(photo and not touch){
    if(millis() < 20000)
      str_state.data = "pi";
    else str_state.data = "random";
  }
  if(!photo)
  {
    str_state.data = "move toward ball";
  }
  if(touch or (!photo and has_find))
  {
    str_state.data = "find door";
    update_ratio();
    has_find = true;
  }
  if(find_door)
  {
    str_state.data = "move toward door";
  }
  pub_state.publish(&str_state);
  if(str_state.data == "pi" or 
    str_state.data == "random") {motor_control(pwm_r, pwm_l);} // From topics
  if(str_state.data == "move toward ball"){
    motor_control(0, 0); // Stop first
    long ts = millis();
    while(!digitalRead(TOUCH) and (millis() - ts < 2000)){
      motor_control(120 + trim_, 120 - trim_);
    }
    motor_control(0, 0); // then stop again
  }
  if(str_state.data == "find door"){
    motor_control(0, 0); // Stop first
    motor_control(110, -110); 
    delay(750); // Rotate CCW about 0.5 second
    motor_control(0, 0); // then stop again
    delay(100);
    motor_control(120 + trim_, 120 - trim_);
    delay(300);
    motor_control(-110, 110);
    delay(300);
  }
  if(str_state.data == "move toward door")
  {
    motor_control(0, 0); // Stop first
    motor_control(150 + trim_, 150 - trim_);
    delay(500); // Move toward it for 0.5 second
    motor_control(0, 0); // then stop again
  }
  if(count == 5) count = 0;
  has_find = digitalRead(TOUCH);
  nh.spinOnce();
}
