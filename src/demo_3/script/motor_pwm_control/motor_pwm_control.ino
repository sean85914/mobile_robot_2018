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
  Sensors:
    Touch_down: 10
    Touch_left: 11
    Touch_right: 12
    Photo: 8
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
#define TOUCH_D 10
#define TOUCH_L 11
#define TOUCH_R 12
#define PHOTO   8

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
ros::Publisher pub_touch_r("right_collision", &touch_r);
ros::Publisher pub_touch_l("left_collision", &touch_l);
ros::Publisher pub_touch_d("photo_collision", &touch_d);
ros::Publisher pub_photo_state("photo_state", &photo);

void setup()
{
  // Init ros node handler
  nh.initNode();
  // Subscribers
  nh.subscribe(sub_right);
  nh.subscribe(sub_left);
  // Publishers
  nh.advertise(pub_touch_r);
  nh.advertise(pub_touch_l);
  nh.advertise(pub_touch_d);
  nh.advertise(pub_photo_state);
  // Set pin mode
  pinMode(OUTPUT, IN1);
  pinMode(OUTPUT, IN2);
  pinMode(OUTPUT, IN3);
  pinMode(OUTPUT, IN4);
  pinMode(OUTPUT, ENA);
  pinMode(OUTPUT, ENB);
  pinMode(INPUT, TOUCH_D);
  pinMode(INPUT, TOUCH_L);
  pinMode(INPUT, TOUCH_R);
  pinMode(INPUT, PHOTO);
}

void loop()
{
  // Publish sensor data
  touch_r.data = digitalRead(TOUCH_R); pub_touch_r.publish(&touch_r); delay(50);
  touch_l.data = digitalRead(TOUCH_L); pub_touch_l.publish(&touch_l); delay(50);
  touch_d.data = digitalRead(TOUCH_D); pub_touch_d.publish(&touch_d); delay(50);
  photo.data = !digitalRead(PHOTO); pub_photo_state.publish(&photo);  delay(50);
  // Forward
  if(pwm_r >= 0 and pwm_l >= 0){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, pwm_l);
    analogWrite(ENB, pwm_r);
  }
  // Left
  else if(pwm_r >= 0 and pwm_l <= 0){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, pwm_l);
    analogWrite(ENB, abs(pwm_r));
  }
  // Right
  else if(pwm_r <= 0 and pwm_l >= 0){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, abs(pwm_l));
    analogWrite(ENB, pwm_r);
  }
  // Backward
  else{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, abs(pwm_l));
    analogWrite(ENB, abs(pwm_r));
  }
  nh.spinOnce();
}
