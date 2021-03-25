#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;

Servo servo1;
Servo servo2;
int head_yaw_offset  = -90;
int head_tilt_offset = 90;
//start tilt 90+25
//end tilt 90-15
void servo1_cb( const std_msgs::UInt16&  cmd_msg){
  servo1.write(cmd_msg.data); //set servo angle, should be from 0-180  
  nh.logdebug(cmd_msg.data);
 
}

void servo2_cb( const std_msgs::UInt16&  cmd_msg){
  servo2.write(cmd_msg.data); //set servo angle, should be from 0-180  
  nh.logdebug(cmd_msg.data);

}
ros::Subscriber<std_msgs::UInt16> sub1("/head/neck_pan_goal", servo1_cb);
ros::Subscriber<std_msgs::UInt16> sub2("/head/neck_tilt_goal", servo2_cb);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);

  servo1.attach(9); //attach it to pin 9
  servo2.attach(10);//attach it to pin10
  servo1.write(head_yaw_offset);
  servo2.write(head_tilt_offset);
  
}

void loop(){
  nh.spinOnce();
  delay(20);
}