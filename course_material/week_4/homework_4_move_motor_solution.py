#!/usr/bin/env python
import sys
sys.path.append('..')
from lib.robot import Robot
from lib.ros_environment import ROSEnvironment
import time

def startNod(robot):
    robot.center()
    time.sleep(1)

    #TODO: insert code to make the robot nod.
    #Robot looks up
    robot.up(1)
    for i in range(0,3):
        #Robot moves down
        robot.down(2)
        #Waits a bit
        time.sleep(0.5)
        #Moves up
        robot.up(2)
        #Waits a bit
        time.sleep(0.5)
    robot.center()

def startShake(robot):
    robot.center()
    time.sleep(1)

    #TODO: insert code to make the robot shake.
    #Looks left
    robot.left(1)
    for i in range(0,3):
        #Looks right
        robot.right(2)
        #Waits a bit
        time.sleep(0.5)
        #Looks left
        robot.left(2)
        #Waits a bit
        time.sleep(0.5)
    robot.center()


def main():
    # We need to initalize ROS environment for Robot and camera to connect/communicate
    ROSEnvironment()
    # Initalize robot
    robot = Robot()
    # Start robot
    robot.start()
    # Uncomment/comment
    startNod(robot)
    # Uncomment/comment
    # startShake(robot)


if __name__ == '__main__':
    main()
