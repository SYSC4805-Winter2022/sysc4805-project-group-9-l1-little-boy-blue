# Make sure to have the server side running in CoppeliaSim: 
# in a child script of a CoppeliaSim scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

import math, time, inspect
from turtle import pos

class Robot:

    DEGREE_DIFFERENCE = 3

    def __init__(self, clientID) -> None:
        self.clientID = clientID
        returnCode, self.RobotBody = sim.simxGetObjectHandle(clientID,'RobotBody',sim.simx_opmode_blocking)
        returnCode, self.LeftMotor = sim.simxGetObjectHandle(clientID,'LeftMotor',sim.simx_opmode_blocking)
        returnCode, self.LeftMotor0 = sim.simxGetObjectHandle(clientID,'LeftMotor0',sim.simx_opmode_blocking)
        returnCode, self.RightMotor = sim.simxGetObjectHandle(clientID,'RightMotor',sim.simx_opmode_blocking)
        returnCode, self.RightMotor0 = sim.simxGetObjectHandle(clientID,'RightMotor0',sim.simx_opmode_blocking)
        returnCode, self.PlowBaseJoint = sim.simxGetObjectHandle(clientID,'PlowBaseJoint',sim.simx_opmode_blocking)
        returnCode, self.PlowLeftJoint = sim.simxGetObjectHandle(clientID,'PlowLeftJoint',sim.simx_opmode_blocking)
        returnCode, self.PlowLeftBarrierJoint = sim.simxGetObjectHandle(clientID,'PlowLeftBarrierJoint',sim.simx_opmode_blocking)
        returnCode, self.PlowRightJoint = sim.simxGetObjectHandle(clientID,'PlowRightJoint',sim.simx_opmode_blocking)
        returnCode, self.PlowRightBarrierJoint = sim.simxGetObjectHandle(clientID,'PlowRightBarrierJoint',sim.simx_opmode_blocking)
        returnCode, self.ObjDetectSensorLeft = sim.simxGetObjectHandle(clientID, 'Proximity_sensor_left', sim.simx_opmode_blocking)
        returnCode, self.ObjDetectSensorMiddle = sim.simxGetObjectHandle(clientID, 'Proximity_sensor_middle', sim.simx_opmode_blocking)
        returnCode, self.ObjDetectSensorRight = sim.simxGetObjectHandle(clientID, 'Proximity_sensor_right', sim.simx_opmode_blocking)
        returnCode, self.LineDetectSensorLeft = sim.simxGetObjectHandle(clientID, 'Line_Following_sensor_left', sim.simx_opmode_blocking)
        returnCode, self.LineDetectSensorMiddle = sim.simxGetObjectHandle(clientID, 'Line_Following_sensor_middle', sim.simx_opmode_blocking)
        returnCode, self.LineDetectSensorRight = sim.simxGetObjectHandle(clientID, 'Line_Following_sensor_right', sim.simx_opmode_blocking)

    def start(self):
        """
            write your code here!
        """
        self.extend_plow()
        while not self.is_plow_(extended=True):
            continue
        # self.move_straight(2)
        # time.sleep(0.5)
        self.turn_right(170)
        self.turn_to(180)
        # self.turn_left(90)
        # self.move_straight(2)
        # time.sleep(2)
        # self.turn_left(120)
        # self.turn_left(90)
        # self.move_straight(2)
        # time.sleep(2)
        # self.turn_right(100)
        # time.sleep(2)
        # self.move_straight(2)
        # time.sleep(2)
        # while 1:
        #     print(self.determine_orientation())
        #     time.sleep(0.1)
        return

    def turn_to(self, degree):
        current_orientation = self.get_orientation()
        quadrant = current_orientation // 90
        if quadrant == 0 or quadrant == 1:
            if current_orientation <= degree <= current_orientation + 180:
                self.turn_left(degree - current_orientation)
            else:
                if current_orientation > degree:
                    _degree = current_orientation - degree
                else:
                    _degree = 360 - current_orientation + degree
                self.turn_right(_degree)
        elif quadrant == 2 or quadrant == 3:
            if current_orientation - 180 <= degree <= current_orientation:
                self.turn_right(current_orientation - degree)
            else:
                if degree > current_orientation:
                    _degree= degree - current_orientation
                else:
                    _degree = 360 - current_orientation + degree
                self.turn_left(_degree)

    def turn_left(self, relative_orientation):
        """
            relative_orientation: the orientation the robot should be in after turning
        """
        degree = lambda current_orientation, relative_orientation: current_orientation + relative_orientation
        self.__turn(relative_orientation, degree, self.move_left)

    def turn_right(self, relative_orientation):
        """
            relative_orientation: the orientation the robot should be in after turning
        """
        degree = lambda current_orientation, relative_orientation: current_orientation - relative_orientation
        self.__turn(relative_orientation, degree, self.move_right)

    def __turn(self, relative_orientation, calc_func, direction_func):
        current_orientation = self.get_orientation()
        self.stop()
        degree = calc_func(current_orientation, relative_orientation)
        min_degree = self.__to_360(degree - self.DEGREE_DIFFERENCE)
        max_degree = self.__to_360(degree + self.DEGREE_DIFFERENCE)
        direction_func(2)
        if min_degree > max_degree:
            while not (self.get_orientation() >= min_degree or self.get_orientation() <= max_degree):
                continue
        else:
            while not (min_degree <= self.get_orientation() <= max_degree):
                continue
        self.stop()
        print(inspect.stack()[1][3], self.get_orientation())
    
    def stop(self):
        self.move_straight(0)

    def extend_plow(self):
        sim.simxSetJointTargetPosition(self.clientID,self.PlowBaseJoint, 0.1,sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(self.clientID,self.PlowLeftJoint, -0.2,sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(self.clientID,self.PlowRightJoint, 0.2,sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(self.clientID,self.PlowLeftBarrierJoint, 0.09,sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(self.clientID,self.PlowRightBarrierJoint, 0.09, sim.simx_opmode_oneshot)

    def retract_plow(self):
        sim.simxSetJointTargetPosition(self.clientID,self.PlowBaseJoint, 0,sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(self.clientID,self.PlowLeftJoint, 0,sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(self.clientID,self.PlowRightJoint, 0,sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(self.clientID,self.PlowLeftBarrierJoint, 0,sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(self.clientID,self.PlowRightBarrierJoint, 0,sim.simx_opmode_oneshot)
    
    def retract_barrier(self):
        sim.simxSetJointTargetPosition(self.clientID,self.PlowLeftBarrierJoint, 0,sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(self.clientID,self.PlowRightBarrierJoint, 0,sim.simx_opmode_oneshot)

    def extend_barrier(self):
        sim.simxSetJointTargetPosition(self.clientID,self.PlowLeftBarrierJoint, 0.1,sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(self.clientID,self.PlowRightBarrierJoint, 0.1,sim.simx_opmode_oneshot)

    def is_plow_(self, extended=False, retracted=False):
        returnCode, position = sim.simxGetJointPosition(self.clientID,self.PlowRightJoint,sim.simx_opmode_blocking)
        position = round(position, 1)
        if extended:
            return position >= 0.2
        if retracted:
            return position <= 0
        return False
    
    def move_straight(self, velocity):
        """
            the diameter of the wheel is 0.2m
            maximum 2 m/s = 1146.4968152866 degrees/s
            the velocity in m/s
        """
        returnCode = sim.simxSetJointTargetVelocity(self.clientID,self.LeftMotor,velocity,sim.simx_opmode_oneshot)
        returnCode = sim.simxSetJointTargetVelocity(self.clientID,self.LeftMotor0,velocity,sim.simx_opmode_oneshot)
        returnCode = sim.simxSetJointTargetVelocity(self.clientID,self.RightMotor,velocity,sim.simx_opmode_oneshot)
        returnCode = sim.simxSetJointTargetVelocity(self.clientID,self.RightMotor0,velocity,sim.simx_opmode_oneshot)

    def move_right(self, velocity):
        """
            the velocity in m/s
        """
        returnCode = sim.simxSetJointTargetVelocity(self.clientID,self.LeftMotor,velocity,sim.simx_opmode_oneshot)
        returnCode = sim.simxSetJointTargetVelocity(self.clientID,self.LeftMotor0,velocity,sim.simx_opmode_oneshot)
        returnCode = sim.simxSetJointTargetVelocity(self.clientID,self.RightMotor,-velocity,sim.simx_opmode_oneshot)
        returnCode = sim.simxSetJointTargetVelocity(self.clientID,self.RightMotor0,-velocity,sim.simx_opmode_oneshot)

    def move_left(self, velocity):
        """
            the velocity in m/s
        """
        returnCode = sim.simxSetJointTargetVelocity(self.clientID,self.LeftMotor,-velocity,sim.simx_opmode_oneshot)
        returnCode = sim.simxSetJointTargetVelocity(self.clientID,self.LeftMotor0,-velocity,sim.simx_opmode_oneshot)
        returnCode = sim.simxSetJointTargetVelocity(self.clientID,self.RightMotor,velocity,sim.simx_opmode_oneshot)
        returnCode = sim.simxSetJointTargetVelocity(self.clientID,self.RightMotor0,velocity,sim.simx_opmode_oneshot)

    def get_orientation(self):
        """
            roll (index 0): dont look at it
            pitch (index 1): the orientation of the robot
                   around 90: front
                   around 270: back
                   around 180: left
                   around 0: right
            yaw (index 2): dont look at it
        """
        returnCode, quat = sim.simxGetObjectQuaternion(self.clientID,self.RobotBody,-1,sim.simx_opmode_blocking)
        x, y, z, w = quat
        _, degree, __ = self.quaternionToYawPitchRoll(x, y, z, w)
        return self.__to_360(degree)

    def get_position(self):
        """
            x (index 0)
            y (index 1)
            z (index 2): dont look at it
        """
        returnCode, position = sim.simxGetObjectPosition(self.clientID,self.RobotBody,-1,sim.simx_opmode_blocking)
        return round(position, 1)

    def quaternionToYawPitchRoll(self, x, y, z, w):
        roll = math.degrees(math.atan2(2*y*w - 2*x*z, 1 - 2*y*y - 2*z*z))
        pitch = math.degrees(math.atan2(2*x*w - 2*y*z, 1 - 2*x*x - 2*z*z))
        yaw = math.degrees(math.asin(2*x*y + 2*z*w))

        return round(yaw, 1), round(pitch, 1), round(roll, 1)

    def get_line_following_sensor_data(self):
        """
            (left, middle, right)
        """
        leftReturnCode, leftDetectionState, auxPackets = sim.simxReadVisionSensor(self.clientID, self.LineDetectSensorLeft, sim.simx_opmode_blocking)
        middleReturnCode, middleDetectionState, auxPackets = sim.simxReadVisionSensor(self.clientID, self.LineDetectSensorMiddle, sim.simx_opmode_blocking)
        rightReturnCode, rightDetectionState, auxPackets= sim.simxReadVisionSensor(self.clientID, self.LineDetectSensorRight, sim.simx_opmode_blocking)
        print("LineDetection: Left=" + str(leftDetectionState) + ", Middle=" + str(middleDetectionState) + ", Right=" + str(rightDetectionState))
        return (leftDetectionState, middleDetectionState, rightDetectionState)
    
    def get_object_detection_sensor_data(self):
        """
            (left, middle, right)
        """
        leftReturnCode, leftDetectionState, leftDetectedPoint, leftDetectedObjectHandle, leftDetectedSurfaceNormalVector = sim.simxReadProximitySensor(self.clientID, self.ObjDetectSensorLeft, sim.simx_opmode_streaming)
        middleReturnCode, middleDetectionState, middleDetectedPoint, middleDetectedObjectHandle, middleDetectedSurfaceNormalVector = sim.simxReadProximitySensor(self.clientID, self.ObjDetectSensorMiddle, sim.simx_opmode_streaming)
        rightReturnCode, rightDetectionState, rightDetectedPoint, rightDetectedObjectHandle, rightDetectedSurfaceNormalVector = sim.simxReadProximitySensor(self.clientID, self.ObjDetectSensorRight, sim.simx_opmode_streaming)
        print("ObjDetection: Left=" + str(leftDetectionState) + ", Middle=" + str(middleDetectionState) + ", Right=" + str(rightDetectionState))
        return (leftDetectionState, middleDetectionState, rightDetectionState)

    def __to_360(self, degree):
        if degree < 0:
            degree += 360
        elif degree > 360:
            degree -= 360
        return round(degree, 1)

if __name__ == "__main__":
    print ('Program started')
    sim.simxFinish(-1) # just in case, close all opened connections
    clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
    if clientID!=-1:
        print ('Connected to remote API server')

        robot = Robot(clientID)

        robot.start()

        # Now send some data to CoppeliaSim in a non-blocking fashion:
        sim.simxAddStatusbarMessage(clientID,'Hello CoppeliaSim!',sim.simx_opmode_oneshot)

        # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        sim.simxGetPingTime(clientID)

        # Now close the connection to CoppeliaSim:
        sim.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')
    print ('Program ended')

