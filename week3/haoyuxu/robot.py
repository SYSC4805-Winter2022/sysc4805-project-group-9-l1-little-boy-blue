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

import time
import math

class Robot:

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

    def extend_plow(self):
        sim.simxSetJointTargetPosition(self.clientID,self.PlowBaseJoint, 0.1,sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(self.clientID,self.PlowLeftJoint, -0.2,sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(self.clientID,self.PlowRightJoint, 0.2,sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(self.clientID,self.PlowLeftBarrierJoint, 0.1,sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(self.clientID,self.PlowRightBarrierJoint, 0.1,sim.simx_opmode_oneshot)

    def retract_plow(self):
        sim.simxSetJointTargetPosition(self.clientID,self.PlowBaseJoint, 0,sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(self.clientID,self.PlowLeftJoint, 0,sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(self.clientID,self.PlowRightJoint, 0,sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(self.clientID,self.PlowLeftBarrierJoint, 0,sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(self.clientID,self.PlowRightBarrierJoint, 0,sim.simx_opmode_oneshot)
    
    def move_straight(self, velocity):
        """
            the diameter of the wheel is 0.2m
            maximum 2 m/s = 1146.4968152866 degrees/s
            degree: to ensure the robot moves straight
        """
        returnCode = sim.simxSetJointTargetVelocity(self.clientID,self.LeftMotor,velocity,sim.simx_opmode_oneshot)
        returnCode = sim.simxSetJointTargetVelocity(self.clientID,self.LeftMotor0,velocity,sim.simx_opmode_oneshot)
        returnCode = sim.simxSetJointTargetVelocity(self.clientID,self.RightMotor,velocity,sim.simx_opmode_oneshot)
        returnCode = sim.simxSetJointTargetVelocity(self.clientID,self.RightMotor0,velocity,sim.simx_opmode_oneshot)

    def move_right(self, velocity):
        returnCode = sim.simxSetJointTargetVelocity(self.clientID,self.LeftMotor,velocity,sim.simx_opmode_oneshot)
        returnCode = sim.simxSetJointTargetVelocity(self.clientID,self.LeftMotor0,velocity,sim.simx_opmode_oneshot)
        returnCode = sim.simxSetJointTargetVelocity(self.clientID,self.RightMotor,-velocity,sim.simx_opmode_oneshot)
        returnCode = sim.simxSetJointTargetVelocity(self.clientID,self.RightMotor0,-velocity,sim.simx_opmode_oneshot)

    def move_left(self, velocity):
        returnCode = sim.simxSetJointTargetVelocity(self.clientID,self.LeftMotor,-velocity,sim.simx_opmode_oneshot)
        returnCode = sim.simxSetJointTargetVelocity(self.clientID,self.LeftMotor0,-velocity,sim.simx_opmode_oneshot)
        returnCode = sim.simxSetJointTargetVelocity(self.clientID,self.RightMotor,velocity,sim.simx_opmode_oneshot)
        returnCode = sim.simxSetJointTargetVelocity(self.clientID,self.RightMotor0,velocity,sim.simx_opmode_oneshot)

    def get_orientation(self):
        """
            
        """
        returnCode, orientation = sim.simxGetObjectOrientation(self.clientID,self.RobotBody,-1,sim.simx_opmode_blocking)
        alpha, beta, gamma = orientation
        # print(self.alphaBetaGammaToYawPitchRoll(alpha, beta, gamma))
        print(orientation)

    def alphaBetaGammaToYawPitchRoll(self, alpha, beta, gamma):
        A = math.cos(alpha)
        B = math.sin(alpha)
        C = math.cos(beta)
        D = math.sin(beta)
        E = math.cos(gamma)
        F = math.sin(gamma)
        AD = A*D
        BD = B*D
        m = [
            C*E,
            -C*F,
            D,
            BD*E+A*F,
            -BD*F+A*E,
            -B*C,
            AD*E+B*F,
            AD*F+B*E,
            A*C
        ]
        v = m[6]
        if v > 1:
            v = 1
        elif v < -1:
            v = -1
        pitchAngle = math.asin(-v)
        if abs(v) < 0.999999:
            rollAngle=math.atan2(m[7],m[8])
            yawAngle=math.atan2(m[3],m[0])
        else:
            rollAngle=math.atan2(-m[5],m[4])
            yawAngle=0
        return [yawAngle, pitchAngle, rollAngle]

if __name__ == "__main__":
    print ('Program started')
    sim.simxFinish(-1) # just in case, close all opened connections
    clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
    if clientID!=-1:
        print ('Connected to remote API server')

        robot = Robot(clientID)

        # robot.extend_plow()
        # robot.retract_plow()
        # robot.move_straight(2)
        # robot.move_left(2)
        robot.move_right(0.1)
        while 1:
            robot.get_orientation()
            time.sleep(0.1)

        # Now send some data to CoppeliaSim in a non-blocking fashion:
        sim.simxAddStatusbarMessage(clientID,'Hello CoppeliaSim!',sim.simx_opmode_oneshot)

        # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        sim.simxGetPingTime(clientID)

        # Now close the connection to CoppeliaSim:
        sim.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')
    print ('Program ended')

