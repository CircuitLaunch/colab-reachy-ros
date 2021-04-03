#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>

// this node assumes that you are running rosserial on your computer with the correct
// arduino port connected my case it was /dev/ttyACM0 yours will be dirrent 
ros::NodeHandle  nh;
// create simple servo objects

Servo servo1;
Servo servo2;

// offsets needed to put the head in the correct starting place
int head_start_yaw_offset  = 94;//magic value that is only spacefic to our hardware
// use 90 degrees '90' as a default
int head_start_tilt_offset = 80; //a good default to use is 0 degrees 

// a call back function that is run anytime
// a message from ros with the topic /head/neck_pan_goal which is a int16
// is run it moves the servo to that position
// ex: from ros topic "80" it moves the servo to the servos 80 degrees

// yaw servo callback
void servo1_cb( const std_msgs::UInt16&  cmd_msg){
// what actually moves the servos
  servo1.write(cmd_msg.data); //set servo angle, should be from 0-180
}
// pitch servo callaback 
void servo2_cb( const std_msgs::UInt16&  cmd_msg){
  servo2.write(cmd_msg.data); //set servo angle, should be from 0-180  
}
// setup communication within ROS robotic operating system
// to listen for a topic (a datastream that this arduino listens to)
ros::Subscriber<std_msgs::UInt16> sub1("/head/neck_pan_goal", servo1_cb);
ros::Subscriber<std_msgs::UInt16> sub2("/head/neck_tilt_goal", servo2_cb);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);

  servo1.attach(9); //attach it to pin 9
  servo2.attach(10);//attach it to pin10
  servo1.write(head_start_yaw_offset);
  servo2.write(head_start_tilt_offset);
  
}

void loop(){
  nh.spinOnce();
  delay(20);
}