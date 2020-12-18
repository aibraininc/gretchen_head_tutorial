#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <std_msgs/Float32MultiArray.h>
#include <iostream>

class Head : public hardware_interface::RobotHW
{
public:


  double cmd[2];
  double preCmd[2];
  double pos[2];
  double vel[2];
  double eff[2];

  ros::Publisher mCmdPub;

  Head()
 {
    // Intialize raw data
    pos[0] = 0.0; pos[1] = 0.0;
    vel[0] = 0.0; vel[1] = 0.0;
    eff[0] = 0.0; eff[1] = 0.0;
    cmd[0] = 0.0; cmd[1] = 0.0;
    preCmd[0] = 0.0; preCmd[1] = 0.0;


   // connect and register the joint state interface
   hardware_interface::JointStateHandle state_handle_a("pan_joint", &pos[0], &vel[0], &eff[0]); //pos vel eff outputs of the state message...
   jnt_state_interface.registerHandle(state_handle_a);
   hardware_interface::JointStateHandle state_handle_b("tilt_joint", &pos[1], &vel[1], &eff[1]); //pos vel eff outputs of the state message...
   jnt_state_interface.registerHandle(state_handle_b);


   hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle("pan_joint"), &cmd[0]); //cmd is the commanded value depending on the controller.
   hardware_interface::JointHandle pos_handle_b(jnt_state_interface.getHandle("tilt_joint"), &cmd[1]); //cmd is the commanded value depending on the controller.
   registerInterface(&jnt_state_interface);

   // connect and register the joint position interface
   jnt_pos_interface.registerHandle(pos_handle_a);
   jnt_vel_interface.registerHandle(pos_handle_a);
   jnt_eff_interface.registerHandle(pos_handle_a);
   
   jnt_pos_interface.registerHandle(pos_handle_b);
   jnt_vel_interface.registerHandle(pos_handle_b);
   jnt_eff_interface.registerHandle(pos_handle_b);

   registerInterface(&jnt_pos_interface);
   registerInterface(&jnt_vel_interface);
   registerInterface(&jnt_eff_interface);
}

  virtual ~Head()
  {}

  void write()
  {
    // 같은 명령이 계속 온다. 그러면 무시한다.
    if(cmd[0] == preCmd[0])
      return;
    std_msgs::Float32MultiArray cmdArray;
    cmdArray.data.push_back(cmd[0]);
    cmdArray.data.push_back(cmd[1]);
    mCmdPub.publish(cmdArray);
    preCmd[0] = cmd[0];
  }

  void read()
  {}
  void setPublisher(ros::Publisher& cmdPub){
    mCmdPub = cmdPub;
  }

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  hardware_interface::EffortJointInterface jnt_eff_interface;


  double orientation[4];                    ///< A pointer to the storage of the orientation value: a quaternion (x,y,z,w)
  double orientation_covariance[9];         ///< A pointer to the storage of the orientation covariance value: a row major 3x3 matrix about (x,y,z)
  double angular_velocity;               ///< A pointer to the storage of the angular velocity value: a triplet (x,y,z)
  double angular_velocity_covariance[9];    ///< A pointer to the storage of the angular velocity covariance value: a row major 3x3 matrix about (x,y,z)
  double linear_acceleration;            ///< A pointer to the storage of the linear acceleration value: a triplet (x,y,z)
  double linear_acceleration_covariance[9]; ///< A pointer to the storage of the linear acceleration covariance value: a row major 3x3 matrix about (x,y,z)

};