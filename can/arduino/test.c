#include <ros.h>
#include <std_msgs/Float32.h>

#include <CAN.h>

ros::NodeHandle  nh;

typedef union { 
    float f; 
    struct{ 
        unsigned int bite0 : 8; 
        unsigned int bite1 : 8; 
        unsigned int bite2 : 8; 
        unsigned int bite3 : 8;   
    } raw; 
} myfloat; 


void messageCb(const std_msgs::Float32& msg){
  myfloat current;
  myfloat rpm;

  current.f = 0.85;
  rpm.f = msg.data;
  CAN.beginPacket(0x501);
  CAN.write(rpm.raw.bite0);
  CAN.write(rpm.raw.bite1);
  CAN.write(rpm.raw.bite2);
  CAN.write(rpm.raw.bite3);

  
  CAN.write(current.raw.bite0);
  CAN.write(current.raw.bite1);
  CAN.write(current.raw.bite2);
  CAN.write(current.raw.bite3);
  
  CAN.endPacket();
}

ros::Subscriber<std_msgs::Float32> sub("tester", &messageCb);

void setup(){
  nh.initNode();
  nh.subscribe(sub);
  // start the CAN bus at 1000 kbps
  if (!CAN.begin(1000E3)) {
    while (1);
  }
}

void loop(){
  nh.spinOnce();
}