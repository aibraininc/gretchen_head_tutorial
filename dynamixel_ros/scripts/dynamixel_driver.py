#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

class motor_controller:
    def __init__(self):
        print "starting motor controller"
        self.jointCmdPub = rospy.Publisher("/dynamixel_workbench/joint_trajectory", JointTrajectory, queue_size = 10)
        self.jointCmdSub = rospy.Subscriber("joint/cmd", Float32MultiArray, self.writeMotor, queue_size = 10)
        self.jointStateSub = rospy.Subscriber("/dynamixel_workbench/joint_states", JointState, self.readMotor, queue_size = 10)

        #self.jointStatePub = rospy.Publisher("joint/poses_dyn", Float32MultiArray, queue_size = 10)
        self.jointStatePub = rospy.Publisher("joint/poses", Float32MultiArray, queue_size = 10)

        self.motorTotal = 2
        self.jointCmd = JointTrajectory()
        self.prevCmd = [0, 0]

    def readMotor(self, arraystate):
        #print "publishing"
        curPos = Float32MultiArray()
        for i in range(self.motorTotal):
            curPos.data.append(arraystate.position[i])
        self.jointStatePub.publish(curPos)
    def writeMotor(self, arraycmd):
        print "heard something"

        self.jointCmd = JointTrajectory()
        self.jointCmd.header.stamp = rospy.Time()
        self.jointCmd.header.frame_id = "base_link"
        self.jointCmd.joint_names.append("pan") #pan - index 0 - hardware 1
        self.jointCmd.joint_names.append("tilt") #tilt - index 1 - hardware 2

        p = JointTrajectoryPoint()
        for i in range(self.motorTotal):
            p.positions.append(arraycmd.data[i])
        p.time_from_start = rospy.Duration(1)
        self.jointCmd.points.append(p)
        self.jointCmdPub.publish(self.jointCmd)
        self.prevCmd = []
        for i in range(self.motorTotal):
            self.prevCmd.append(arraycmd.data[i])
def main():
    rospy.init_node('dynamixel_driver', anonymous=True)
    motor_controller()
    try:
        while not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
