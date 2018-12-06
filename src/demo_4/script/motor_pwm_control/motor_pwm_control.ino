#include <ros.h>

#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>

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

// Right wheel
#define IN1 2
#define IN2 3
#define ENA 6
// Left wheel
#define IN3 4
#define IN4 5
#define ENB 9

ros::NodeHandle nh;

// motor pwm
int pwm_r = 0, pwm_l = 0;

// callback
void cb_r(const std_msgs::Int16& msg){
  pwm_r = msg.data;
  //nh.loginfo("pwm r update");
}

void cb_l(const std_msgs::Int16& msg){
  pwm_l = msg.data;
  //nh.loginfo("pwm l update");
} 
// Messages for publishing
std_msgs::Bool touch_r, touch_l, touch_d, photo;
// Subscribers and publishers
ros::Subscriber<std_msgs::Int16> sub_right("right_pwm", &cb_r);
ros::Subscriber<std_msgs::Int16> sub_left("left_pwm", &cb_l);

void setup()
{
  // Init ros node handler
  nh.initNode();
  // Subscribers
  nh.subscribe(sub_right);
  nh.subscribe(sub_left);
  // Set pin mode
  pinMode(OUTPUT, IN1);
  pinMode(OUTPUT, IN2);
  pinMode(OUTPUT, IN3);
  pinMode(OUTPUT, IN4);
  pinMode(OUTPUT, ENA);
  pinMode(OUTPUT, ENB);
}

void loop()
{
  // Forward
  if(pwm_r >= 0 and pwm_l >= 0){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, pwm_r);
    analogWrite(ENB, pwm_l);
  }
  // Left
  else if(pwm_r >= 0 and pwm_l <= 0){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, abs(pwm_r));
    analogWrite(ENB, pwm_l);
  }
  // Right
  else if(pwm_r <= 0 and pwm_l >= 0){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, pwm_r);
    analogWrite(ENB, abs(pwm_l));
  }
  // Backward
  else{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, abs(pwm_r));
    analogWrite(ENB, abs(pwm_l));
  }
  nh.spinOnce();
}
