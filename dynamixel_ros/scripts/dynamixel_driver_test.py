#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library
# Control table address
ADDR_PRO_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_PRO_LED_RED            = 25
ADDR_PRO_GOAL_POSITION      = 30
ADDR_PRO_PRESENT_POSITION   = 37

# Data Byte Length
LEN_PRO_LED_RED             = 1
LEN_PRO_GOAL_POSITION       = 4
LEN_PRO_PRESENT_POSITION    = 4

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL1_ID                     = 0                 # Dynamixel#1 ID : 1
DXL2_ID                     = 1                 # Dynamixel#1 ID : 2
BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 500           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 600            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]        # Goal position
dxl_led_value = [0x00, 0x01]                                                        # Dynamixel LED value for write

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupBulkWrite instance
groupBulkWrite = GroupBulkWrite(portHandler, packetHandler)

# Initialize GroupBulkRead instace for Present Position
groupBulkRead = GroupBulkRead(portHandler, packetHandler)


class motor_controller:
    def __init__(self):
        print "starting motor controller"
                # Open port
        if portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()


        # Set port baudrate
        if portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()
        # Enable Dynamixel#1 Torque
        self.dxl_comm_result, self.dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
        if self.dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(self.dxl_comm_result))
        elif self.dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(self.dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % DXL1_ID)

        # Enable Dynamixel#2 Torque
        self.dxl_comm_result, self.dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
        if self.dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(self.dxl_comm_result))
        elif self.dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(self.dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % DXL2_ID)

        # Add parameter storage for Dynamixel#1 present position
        dxl_addparam_result = groupBulkRead.addParam(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupBulkRead addparam failed" % DXL1_ID)
            quit()

        # Add parameter storage for Dynamixel#2 LED value
        dxl_addparam_result = groupBulkRead.addParam(DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupBulkRead addparam failed" % DXL2_ID)
            quit()


        self.jointCmdPub = rospy.Publisher("/dynamixel_workbench/joint_trajectory", JointTrajectory, queue_size = 10)
        self.jointCmdSub = rospy.Subscriber("joint/cmd", Float32MultiArray, self.writeMotor, queue_size = 10)
        #self.jointStateSub = rospy.Subscriber("/dynamixel_workbench/joint_states", JointState, self.readMotor, queue_size = 10)

        #self.jointStatePub = rospy.Publisher("joint/poses_dyn", Float32MultiArray, queue_size = 10)
        self.jointStatePub = rospy.Publisher("joint/poses", Float32MultiArray, queue_size = 10)

        self.motorTotal = 2
        self.jointCmd = JointTrajectory()
        self.prevCmd = [0, 0]
        self.pos_pan = 512
        self.pos_tilt = 512

        rospy.Timer(rospy.Duration(0.05), self.readMotor)

    def __del__(self):
        # body of destructor
        print "deconstructor"
        groupBulkRead.clearParam()

        # Disable Dynamixel#1 Torque
        self.dxl_comm_result, self.dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
        if self.dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(self.dxl_comm_result))
        elif self.dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(self.dxl_error))

        # Disable Dynamixel#2 Torque
        self.dxl_comm_result, self.dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
        if self.dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(self.dxl_comm_result))
        elif self.dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(self.dxl_error))

        # Close port
        portHandler.closePort()

    def readMotor(self, event):
        print "publishing"
        self.dxl_comm_result = groupBulkRead.txRxPacket()
        if self.dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(self.dxl_comm_result))

        # Check if groupbulkread data of Dynamixel#1 is available
        dxl_getdata_result = groupBulkRead.isAvailable(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupBulkRead getdata failed" % DXL1_ID)
            return 0
            #quit()

        # Get present position value for dxl1
        if dxl_getdata_result == True:
            dxl1_present_position = groupBulkRead.getData(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

        # Check if groupbulkread data of Dynamixel#2 is available
        dxl_getdata_result = groupBulkRead.isAvailable(DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupBulkRead getdata failed" % DXL2_ID)
            return 0
            #quit()

        # Get present position value for dxl2
        if dxl_getdata_result == True:
            dxl2_present_position = groupBulkRead.getData(DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
        print("[ID:%03d] Present Position : %d \t [ID:%03d] Present Position: %d" % (DXL1_ID, dxl1_present_position, DXL2_ID, dxl2_present_position))
        #print(type(dxl1_present_position))
        if(dxl1_present_position<820):
            self.pos_pan = dxl1_present_position
        if(dxl2_present_position<820):
            self.pos_tilt = dxl2_present_position
        #print(self.pos_pan)
        #print(self.pos_tilt)
        #print("[ID:%03d] Present Position : %d \t [ID:%03d] Present Position: %d" % (DXL1_ID, self.pos_pan, DXL2_ID, self.pos_tilt))
        curPos = Float32MultiArray()
        #for i in range(self.motorTotal):
        #    curPos.data.append(arraystate.position[i])
        pos_pan_rad = float(self.pos_pan-512)/float(512-205)*3.14/2
        pos_tilt_rad = float(self.pos_tilt-512)/float(512-205)*3.14/2
        curPos.data = [pos_pan_rad, pos_tilt_rad]
        #print(pos_pan_rad)
        #print(curPos.data)
        self.jointStatePub.publish(curPos)
        print "Exited Writer"

    def readMotor2(self):
        #print "publishing"
        self.dxl_comm_result = groupBulkRead.txRxPacket()
        if self.dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(self.dxl_comm_result))

        # Check if groupbulkread data of Dynamixel#1 is available
        dxl_getdata_result = groupBulkRead.isAvailable(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupBulkRead getdata failed" % DXL1_ID)
            return 0
            #quit()

        # Get present position value for dxl1
        if dxl_getdata_result == True:
            dxl1_present_position = groupBulkRead.getData(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

        # Check if groupbulkread data of Dynamixel#2 is available
        dxl_getdata_result = groupBulkRead.isAvailable(DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupBulkRead getdata failed" % DXL2_ID)
            return 0
            #quit()

        # Get present position value for dxl2
        if dxl_getdata_result == True:
            dxl2_present_position = groupBulkRead.getData(DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
        #print("[ID:%03d] Present Position : %d \t [ID:%03d] Present Position: %d" % (DXL1_ID, dxl1_present_position, DXL2_ID, dxl2_present_position))
        #print(type(dxl1_present_position))
        if(dxl1_present_position<820):
            self.pos_pan = dxl1_present_position
        else:
            print(dxl1_present_position)
        if(dxl2_present_position<820):
            self.pos_tilt = dxl2_present_position
        else:
            print(dxl2_present_position)
        #print(self.pos_pan)
        #print(self.pos_tilt)
        #print("[ID:%03d] Present Position : %d \t [ID:%03d] Present Position: %d" % (DXL1_ID, self.pos_pan, DXL2_ID, self.pos_tilt))
        curPos = Float32MultiArray()
        #for i in range(self.motorTotal):
        #    curPos.data.append(arraystate.position[i])
        pos_pan_rad = float(self.pos_pan-512)/float(512-205)*3.14/2
        pos_tilt_rad = float(self.pos_tilt-512)/float(512-205)*3.14/2
        curPos.data = [pos_pan_rad, pos_tilt_rad]
        #print(pos_pan_rad)
        #print(curPos.data)
        self.jointStatePub.publish(curPos)


    def writeMotor(self, arraycmd):
        #if(self.prevCmd[0]== arraycmd.data[0] and self.prevCmd[1] == arraycmd.data[1]):
        #    return
        print "Heard CMD, Writing to Motor"
        groupBulkWrite.clearParam()

        param_goal_position_pan = (arraycmd.data[0])
        param_goal_position_tilt = (arraycmd.data[1])

        param_goal_position_pan = param_goal_position_pan*2/3.14*float(512-205)+512
        param_goal_position_tilt = param_goal_position_tilt*2/3.14*float(512-205)+512
        #print(param_goal_position_pan)
        param_goal_position_pan = int(param_goal_position_pan)
        param_goal_position_tilt = int(param_goal_position_tilt)
        #print("[ID:%03d] Present Position : %d \t [ID:%03d] Present Position: %d" % (DXL1_ID, self.pos_pan, DXL2_ID, self.pos_tilt))
        diff_pan = self.pos_pan - param_goal_position_pan
        diff_tilt = self.pos_tilt - param_goal_position_tilt
        print("here")
        print(abs(diff_pan))
        #print("[ID:%03d] Goal Position : %d \t Present Position: %d \t [ID:%03d] Goal Position: %d \t Present Position: %d" % (DXL1_ID, param_goal_position_pan, self.pos_pan, DXL2_ID, param_goal_position_tilt, self.pos_tilt))
        print("\t\t\t[ID:%03d] [ID:%03d] \nGoal Position :\t\t %d \t %d \nPresent Position:\t %d \t %d \n" % (DXL1_ID, DXL2_ID, param_goal_position_pan, param_goal_position_tilt, self.pos_pan, self.pos_tilt))

        # Allocate goal position value into byte array
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(param_goal_position_pan)), DXL_HIBYTE(DXL_LOWORD(param_goal_position_pan)), DXL_LOBYTE(DXL_HIWORD(param_goal_position_pan)), DXL_HIBYTE(DXL_HIWORD(param_goal_position_pan))]

        # Add Dynamixel#1 goal position value to the Bulkwrite parameter storage
        dxl_addparam_result = groupBulkWrite.addParam(DXL1_ID, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION, param_goal_position)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupBulkWrite addparam failed" % DXL1_ID)
            quit()

        # Allocate goal position value into byte array
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(param_goal_position_tilt)), DXL_HIBYTE(DXL_LOWORD(param_goal_position_tilt)), DXL_LOBYTE(DXL_HIWORD(param_goal_position_tilt)), DXL_HIBYTE(DXL_HIWORD(param_goal_position_tilt))]

        # Add Dynamixel#2 LED value to the Bulkwrite parameter storage
        #dxl_addparam_result = groupBulkWrite.addParam(DXL2_ID, ADDR_PRO_LED_RED, LEN_PRO_LED_RED, [dxl_led_value[index]])
        dxl_addparam_result = groupBulkWrite.addParam(DXL2_ID, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION, param_goal_position)

        if dxl_addparam_result != True:
            print("[ID:%03d] groupBulkWrite addparam failed" % DXL2_ID)
            quit()
        print("failed here")
        # Bulkwrite goal position and LED value
        dxl_comm_result = groupBulkWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Clear bulkwrite parameter storage
        groupBulkWrite.clearParam()

        self.prevCmd = []
        for i in range(self.motorTotal):
            self.prevCmd.append(arraycmd.data[i])
def main():
    rospy.init_node('dynamixel_driver', anonymous=True)
    try:
        motor_controller()
        while not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
