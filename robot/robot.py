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


import inspect
import math
import enum

class Direction(enum.Enum):
    """
        World direction
    """
    North = 0
    East = 270
    South = 180
    West = 90

class State(enum.Enum):
    """
        Robot state
    """
    Beginning = 1
    Pathing = 2
    End = 3

class Robot:

    DEGREE_DIFFERENCE = 3 # degree tolerance
    BACKING_DISPLACEMENT = -0.02 # move back distance
    FORWARD_DISPLACEMENT = 0.7 # move forward distance
    MOVE_STRAIGHT_SPEED = 1.2 # moving speed
    AVOID_OBJECT_DISPLACEMENT = 1 # avoid object distance
    TUNRING_SPEED = 0.1 # turn speed

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
        returnCode, self.RightBackSensor = sim.simxGetObjectHandle(clientID, 'Right_back_sensor', sim.simx_opmode_blocking)
        returnCode, self.LeftBackSensor = sim.simxGetObjectHandle(clientID, 'Left_back_sensor', sim.simx_opmode_blocking)
        self.currDir = Direction.North

    def start(self):
        """
            Run the robot
        """
        self.init_sensors()
        self.extend_plow()
        while not self.is_plow_(extended=True):
            continue
        self.move_to(y=1)
        self.turn_to(Direction.East.value)
        self.currDir = Direction.East
        self.state = State.Beginning
        print(self.get_position())
        while True:  # Main loop code, can be time dependent
            # print(self.get_position())
            _, middleDetectionState, _, _, _, _ = self.get_object_detection_sensor_data()
            if middleDetectionState:
                print("Object detected")
                self.avoid_object()
            self.move_straight(self.MOVE_STRAIGHT_SPEED)
            middleReturnCode, middleDetectionState, auxPackets = self.get_line_following_sensor_data()

            if middleDetectionState >= 0 and middleReturnCode == 0:
                if auxPackets[0][11] < 0.5:
                    # print("mid_return\n", middleReturnCode)
                    # print("mid_state\n", middleDetectionState)
                    # print("mid_data\n", auxPackets[0][11])
                    # E -> N once
                    if self.currDir == Direction.East and self.state == State.Beginning:
                        print("E -> N")
                        # changing x
                        self.move_to(x=self.BACKING_DISPLACEMENT)
                        self.turn_to(Direction.North.value)
                        self.state = State.Pathing
                        self.currDir = Direction.North

                    # N -> W -> S
                    elif self.currDir == Direction.North and self.state == State.Pathing:
                        print("N -> W -> S")
                        # changing y
                        self.move_to(y=self.BACKING_DISPLACEMENT)
                        self.turn_to(Direction.West.value)
                        self.currDir = Direction.West
                        # changing x
                        self.move_to(x=self.FORWARD_DISPLACEMENT)
                        self.turn_to(Direction.South.value)
                        self.currDir = Direction.South

                    # S -> W -> N
                    elif self.currDir == Direction.South and self.state == State.Pathing:
                        print("S -> W -> N")
                        self.move_to(y=self.BACKING_DISPLACEMENT)
                        self.turn_to(Direction.West.value)
                        self.currDir = Direction.West
                        self.move_to(x=self.FORWARD_DISPLACEMENT)
                        self.turn_to(Direction.North.value)
                        self.currDir = Direction.North

                    # Detect end state

        # leftDetectionState, middleDetectionState , rightDetectionState = self.get_line_following_sensor_data()
        # while not (leftDetectionState or middleDetectionState or rightDetectionState):
        #    self.move_straight(1)
        # time.sleep(2)
        # while 1:
        #     print(self.determine_orientation())
        #     time.sleep(0.1)
        return

    def avoid_object(self):
        """
            Avoid an object
        """
        init_position = self.get_position()
        is_detected = True
        if self.currDir == Direction.North or self.currDir == Direction.South:
            self.turn_to(Direction.West.value)
        elif self.currDir == Direction.East or self.currDir == Direction.West:
            self.turn_to(Direction.South.value)
        while is_detected:
            self.move_straight(self.MOVE_STRAIGHT_SPEED)
            leftDetectedState, _, rightDetectedState, leftDetectedPoint, _, rightDetectedPoint = self.get_object_detection_sensor_data()
            if (round(rightDetectedPoint[2], 1) == 0 and not rightDetectedState) and (round(leftDetectedPoint[2], 1) == 0 and not leftDetectedState):
                is_detected = False
                avoided_position = self.get_position()
                print("Object avoided")
        # self.turn_to(self.currDir.value)
        # self.move_straight(self.MOVE_STRAIGHT_SPEED)
        # _x = init_position[0] - avoided_position[0]
        # _y = init_position[1] - avoided_position[1]
        # if -0.5 < _x < 0.5:
        #     _x = 0
        # if -0.5 < _y < 0.5:
        #     _y = 0
        # delta_position = (_x, _y)
        self.navigate_back(init_position, avoided_position)

    def navigate_back(self, initial_pos, avoided_pos):
        """
            navigate to a point
        """
        prev_dir = self.currDir
        print("Initial", initial_pos)
        print("Avoided", avoided_pos)
        print("curr_dir", prev_dir)

        if prev_dir == Direction.North:
            self.turn_to(Direction.North.value)
            self.move_to(y=self.AVOID_OBJECT_DISPLACEMENT)
            left_back_detection_state, _, right_back_detection_state, _ = self.get_laser_data()
            _, _, rightDetectionState, _, _, _ = self.get_object_detection_sensor_data()
            self.move_straight(self.MOVE_STRAIGHT_SPEED)
            while right_back_detection_state or rightDetectionState:
                left_back_detection_state, _, right_back_detection_state, _ = self.get_laser_data()
                _, _, rightDetectionState, _, _, _ = self.get_object_detection_sensor_data()
            print("out")
            self.turn_to(Direction.East.value)
            self.currDir = Direction.East
            self.move_to(x=initial_pos[0]-avoided_pos[0])
            # back to nav point
            self.turn_to(Direction.North.value)
            self.currDir = Direction.North
            return
        elif prev_dir == Direction.South:
            self.turn_to(Direction.South.value)
            self.move_to(y=self.AVOID_OBJECT_DISPLACEMENT)
            left_back_detection_state, _, right_back_detection_state, _ = self.get_laser_data()
            leftDetectedState, _, rightDetectionState, _, _, _ = self.get_object_detection_sensor_data()
            self.move_straight(self.MOVE_STRAIGHT_SPEED)
            while left_back_detection_state or leftDetectedState:
                left_back_detection_state, _, right_back_detection_state, _ = self.get_laser_data()
                leftDetectedState, _, rightDetectionState, _, _, _ = self.get_object_detection_sensor_data()
            print("out")
            self.turn_to(Direction.East.value)
            self.currDir = Direction.East
            self.move_to(x=initial_pos[0]-avoided_pos[0])
            # back to nav point
            self.turn_to(Direction.South.value)
            self.currDir = Direction.South
            return
        #print("Delta position: ", point)
        # if self.currDir == Direction.North or self.currDir == Direction.South:
        #     self.turn_to(Direction.East.value)
        # elif self.currDir == Direction.East or self.currDir == Direction.West:
        #     self.turn_to(Direction.North.value)
        # if point[0] != 0:
        #     self.move_to(x=point[0])
        # elif point[1] != 0:
        #     self.move_to(y=point[1])

    def move_to(self, x=None, y=None):
        """
            current x and y position +/- x and +/- y
            North: +x -> move forward
                   -x -> move backward
                   +y -> pass
                   -y -> pass
            South: +x -> move forward
                   -x -> move backward
                   +y -> pass
                   -y -> pass
            East:  +x -> pass
                   -x -> pass
                   +y -> move forward
                   -y -> move backward
            West:  +x -> pass
                   -x -> pass
                   +y -> move forward
                   -y -> move backward
        """
        self.stop()
        if x is None and y is None:
            return
        initial_x, initial_y = self.get_position()
        current_x, current_y = initial_x, initial_y
        if x is not None:
            if self.currDir == Direction.East:
                if x > 0:
                    self.move_straight(self.MOVE_STRAIGHT_SPEED)
                    expression = lambda: current_x <= (initial_x + x)
                else:
                    self.move_back(self.MOVE_STRAIGHT_SPEED)
                    expression = lambda: current_x >= (initial_x + x)
            else:
                if x > 0:
                    self.move_straight(self.MOVE_STRAIGHT_SPEED)
                    expression = lambda: current_x >= (initial_x - x)
                else:
                    self.move_back(self.MOVE_STRAIGHT_SPEED)
                    expression = lambda: current_x <= (initial_x - x)
        else:
            if self.currDir == Direction.North:
                if y > 0:
                    self.move_straight(self.MOVE_STRAIGHT_SPEED)
                    expression = lambda: current_y <= (initial_y + y)
                else:
                    self.move_back(self.MOVE_STRAIGHT_SPEED)
                    expression = lambda: current_y >= (initial_y + y)
            else:
                if y > 0:
                    self.move_straight(self.MOVE_STRAIGHT_SPEED)
                    expression = lambda: current_y >= (initial_y - y)
                else:
                    self.move_back(self.MOVE_STRAIGHT_SPEED)
                    expression = lambda: current_y <= (initial_y - y)
        while expression():
            current_x, current_y = self.get_position()
        self.stop()

    def turn_to(self, degree):
        """
            Turn to a absolute (world) degree
        """
        current_orientation = self.get_orientation()
        quadrant = current_orientation // 90
        if quadrant == 0 or quadrant == 1:
            if current_orientation <= degree <= current_orientation + 180:
                self.turn_left(degree - current_orientation)
            else:
                if current_orientation > degree:
                    _degree = current_orientation - degree
                else:
                    _degree = 360 - degree + current_orientation
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
            Turn left relative to current orientation
        """
        degree = lambda current_orientation, relative_orientation: current_orientation + relative_orientation
        self.__turn(relative_orientation, degree, self.move_left)

    def turn_right(self, relative_orientation):
        """
            Turn right relative to current orientation
        """
        degree = lambda current_orientation, relative_orientation: current_orientation - relative_orientation
        self.__turn(relative_orientation, degree, self.move_right)

    def __turn(self, relative_orientation, calc_func, direction_func):
        """
            Abstract turn function
        """
        current_orientation = self.get_orientation()
        self.stop()
        degree = calc_func(current_orientation, relative_orientation)
        min_degree = self.__to_360(degree - self.DEGREE_DIFFERENCE)
        max_degree = self.__to_360(degree + self.DEGREE_DIFFERENCE)
        direction_func(self.TUNRING_SPEED)
        if min_degree > max_degree:
            while not (self.get_orientation() >= min_degree or self.get_orientation() <= max_degree):
                continue
        else:
            while not (min_degree <= self.get_orientation() <= max_degree):
                continue
        self.stop()
        print(inspect.stack()[1][3], self.get_orientation())

    def stop(self):
        """
            Stop the robot
        """
        self.move_straight(0)

    def extend_plow(self):
        """
            Fully extend the plow
        """
        self.plow_down()
        self.plow_extend()
        self.extend_barrier()

    def retract_plow(self):
        """
            Fully retract the plow
        """
        self.plow_up()
        self.plow_retract()
        self.retract_barrier()
    
    def retract_barrier(self):
        """
            Retract the barrier
        """
        sim.simxSetJointTargetPosition(self.clientID,self.PlowLeftBarrierJoint, 0,sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(self.clientID,self.PlowRightBarrierJoint, 0,sim.simx_opmode_oneshot)

    def extend_barrier(self):
        """
            Extend the barrier
        """
        sim.simxSetJointTargetPosition(self.clientID,self.PlowLeftBarrierJoint, 0.09,sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(self.clientID,self.PlowRightBarrierJoint, 0.09, sim.simx_opmode_oneshot)

    def plow_up(self):
        """
            Raise the plow
        """
        sim.simxSetJointTargetPosition(self.clientID,self.PlowBaseJoint, 0,sim.simx_opmode_oneshot)

    def plow_down(self):
        """
            Lower the plow
        """
        sim.simxSetJointTargetPosition(self.clientID,self.PlowBaseJoint, 0.1,sim.simx_opmode_oneshot)

    def plow_extend(self):
        """
            Extend the plow
        """
        sim.simxSetJointTargetPosition(self.clientID,self.PlowLeftJoint, -0.2,sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(self.clientID,self.PlowRightJoint, 0.2,sim.simx_opmode_oneshot)

    def plow_retract(self):
        """
            Retract the plow
        """
        sim.simxSetJointTargetPosition(self.clientID,self.PlowLeftJoint, 0,sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(self.clientID,self.PlowRightJoint, 0,sim.simx_opmode_oneshot)

    def is_plow_(self, extended=False, retracted=False):
        """
            Check if the plow is in the desired state
        """
        returnCode, position = sim.simxGetJointPosition(self.clientID,self.PlowRightJoint,sim.simx_opmode_blocking)
        position = round(position, 1)
        if extended:
            return position >= 0.2
        if retracted:
            return position <= 0
        return False

    def move_back(self, velocity):
        """
            Move back
        """
        velocity *= 10
        returnCode = sim.simxSetJointTargetVelocity(self.clientID,self.LeftMotor,-velocity,sim.simx_opmode_blocking)
        returnCode = sim.simxSetJointTargetVelocity(self.clientID,self.LeftMotor0,-velocity,sim.simx_opmode_blocking)
        returnCode = sim.simxSetJointTargetVelocity(self.clientID,self.RightMotor,-velocity,sim.simx_opmode_blocking)
        returnCode = sim.simxSetJointTargetVelocity(self.clientID,self.RightMotor0,-velocity,sim.simx_opmode_blocking)

    def move_straight(self, velocity):
        """
            Move forward
        """
        velocity *= 10
        returnCode = sim.simxSetJointTargetVelocity(self.clientID,self.LeftMotor,velocity,sim.simx_opmode_oneshot)
        returnCode = sim.simxSetJointTargetVelocity(self.clientID,self.LeftMotor0,velocity,sim.simx_opmode_oneshot)
        returnCode = sim.simxSetJointTargetVelocity(self.clientID,self.RightMotor,velocity,sim.simx_opmode_oneshot)
        returnCode = sim.simxSetJointTargetVelocity(self.clientID,self.RightMotor0,velocity,sim.simx_opmode_oneshot)

    def move_right(self, velocity):
        """
            Turn right
        """
        velocity *= 10
        returnCode = sim.simxSetJointTargetVelocity(self.clientID,self.LeftMotor,velocity,sim.simx_opmode_oneshot)
        returnCode = sim.simxSetJointTargetVelocity(self.clientID,self.LeftMotor0,velocity,sim.simx_opmode_oneshot)
        returnCode = sim.simxSetJointTargetVelocity(self.clientID,self.RightMotor,-velocity,sim.simx_opmode_oneshot)
        returnCode = sim.simxSetJointTargetVelocity(self.clientID,self.RightMotor0,-velocity,sim.simx_opmode_oneshot)
    
    def move_left(self, velocity):
        """
            Turn left
        """
        velocity *= 10
        returnCode = sim.simxSetJointTargetVelocity(self.clientID,self.LeftMotor,-velocity,sim.simx_opmode_oneshot)
        returnCode = sim.simxSetJointTargetVelocity(self.clientID,self.LeftMotor0,-velocity,sim.simx_opmode_oneshot)
        returnCode = sim.simxSetJointTargetVelocity(self.clientID,self.RightMotor,velocity,sim.simx_opmode_oneshot)
        returnCode = sim.simxSetJointTargetVelocity(self.clientID,self.RightMotor0,velocity,sim.simx_opmode_oneshot)

    def get_orientation(self):
        """
            Get the orientation of the robot in absolute (world) degree
        """
        returnCode, euler = sim.simxGetObjectOrientation(self.clientID,self.RobotBody,-1,sim.simx_opmode_blocking)
        _, __, degree = math.degrees(euler[0]), math.degrees(euler[1]), math.degrees(euler[2])
        return self.__to_360(degree)

    def get_position(self):
        """
            Get the position of the robot in the world
        """
        returnCode, position = sim.simxGetObjectPosition(self.clientID,self.RobotBody,-1,sim.simx_opmode_blocking)
        return (round(position[0], 5), round(position[1], 5))

    def get_line_following_sensor_data(self):
        """
            Detect the black line
        """
        leftReturnCode, leftDetectionState, leftAuxPackets = sim.simxReadVisionSensor(self.clientID, self.LineDetectSensorLeft, sim.simx_opmode_blocking)
        rightReturnCode, rightDetectionState, rightAuxPackets= sim.simxReadVisionSensor(self.clientID, self.LineDetectSensorRight, sim.simx_opmode_blocking)
        middleReturnCode, middleDetectionState, auxPackets = sim.simxReadVisionSensor(self.clientID, self.LineDetectSensorMiddle, sim.simx_opmode_streaming)
        return middleReturnCode, middleDetectionState, auxPackets

    def init_sensors(self):
        """
            Initialize the sensors
        """
        _, _, _, _, _ = sim.simxReadProximitySensor(self.clientID, self.ObjDetectSensorLeft, sim.simx_opmode_streaming)
        _, _, _, _, _ = sim.simxReadProximitySensor(self.clientID, self.ObjDetectSensorMiddle, sim.simx_opmode_streaming)
        _, _, _, _, _ = sim.simxReadProximitySensor(self.clientID, self.ObjDetectSensorRight, sim.simx_opmode_streaming)
        _, _, _, _, _ = sim.simxReadProximitySensor(self.clientID, self.LeftBackSensor, sim.simx_opmode_streaming)
        _, _, _, _, _ = sim.simxReadProximitySensor(self.clientID, self.RightBackSensor, sim.simx_opmode_streaming)

    def get_object_detection_sensor_data(self):
        """
            Detect obstacles using ultrasonic sensors
        """
        leftReturnCode, leftDetectionState, leftDetectedPoint, leftDetectedObjectHandle, leftDetectedSurfaceNormalVector = sim.simxReadProximitySensor(self.clientID, self.ObjDetectSensorLeft, sim.simx_opmode_buffer)
        middleReturnCode, middleDetectionState, middleDetectedPoint, middleDetectedObjectHandle, middleDetectedSurfaceNormalVector = sim.simxReadProximitySensor(self.clientID, self.ObjDetectSensorMiddle, sim.simx_opmode_buffer)
        rightReturnCode, rightDetectionState, rightDetectedPoint, rightDetectedObjectHandle, rightDetectedSurfaceNormalVector = sim.simxReadProximitySensor(self.clientID, self.ObjDetectSensorRight, sim.simx_opmode_buffer)
        return leftDetectionState, middleDetectionState, rightDetectionState, leftDetectedPoint, middleDetectedPoint, rightDetectedPoint

    def get_laser_data(self):
        """
            Detect obstacles using laser sensors
        """
        _, right_back_detection_state, right_back_detected_point, _, _ = sim.simxReadProximitySensor(self.clientID, self.RightBackSensor, sim.simx_opmode_buffer)
        _, left_back_detection_state, left_back_detected_point, _, _ = sim.simxReadProximitySensor(self.clientID, self.LeftBackSensor, sim.simx_opmode_buffer)
        return left_back_detection_state, left_back_detected_point, right_back_detection_state, right_back_detected_point

    def __to_360(self, degree):
        """
            Convert degrees (-180 - 180) to (0 - 360)
        """
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

