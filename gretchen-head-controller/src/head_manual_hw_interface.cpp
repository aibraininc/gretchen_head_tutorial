#include "head.hpp"
#include "controller_manager/controller_manager.h"
#include "hardware_interface/actuator_state_interface.h"
#include <std_msgs/Float32MultiArray.h>
#include <ros/callback_queue.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>


Head pantilt;
ros::Publisher cmdPub;

// low-level position of joint 1,2
void jointPosesCallback(const std_msgs::Float32MultiArray& joints)
{
  for(int i = 0; i< 2; i++)
    pantilt.pos[i] = joints.data[i];
}


void jointManualCallback(const sensor_msgs::JointState& joints){
      std_msgs::Float32MultiArray cmdArray;
      cmdArray.data.push_back(joints.position[0]);
      cmdArray.data.push_back(joints.position[1]);
      cmdPub.publish(cmdArray);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "head_hw_interface");
  ros::NodeHandle nh;
  ros::CallbackQueue queue;
  nh.setCallbackQueue(&queue);
  controller_manager::ControllerManager cm(&pantilt, nh);

  ros::Subscriber posSub = nh.subscribe("joint/poses", 1, jointPosesCallback);
  ros::Subscriber jointManualSub = nh.subscribe("/joint_states", 1, jointManualCallback);
  cmdPub = nh.advertise<std_msgs::Float32MultiArray>("joint/cmd",1);

  ros::AsyncSpinner spinner(4, &queue);
  spinner.start();

  ros::Time ts = ros::Time::now();
  ros::Rate rate(10);
  while (ros::ok())
  {
    ros::Duration d = ros::Time::now() - ts;
    ts = ros::Time::now();
    pantilt.read();
    cm.update(ts, d);
    // pantilt.write();
    rate.sleep();
  }

  spinner.stop();

  return 0;
}
