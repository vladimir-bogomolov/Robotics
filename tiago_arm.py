"""bt_navigation controller."""
from controller import Robot, Supervisor
from os.path import exists
import numpy as np
import py_trees
from py_trees.composites import Sequence, Parallel, Selector
from mapping import Mapping
from navigation import Navigation
from planning import Planning
from set_position import SetPosition
from camera_helpers import camera_to_base

positions = {
    'safe': {
       'torso_lift_joint': 0.35,
       'arm_1_joint': 0.71,
       'arm_2_joint': 1.02,
       'arm_3_joint': -2.815,
       'arm_4_joint': 1.011,
       'arm_5_joint': 0,
       'arm_6_joint': 0,
       'arm_7_joint': 0,
       'gripper_left_finger_joint': 0,
       'gripper_right_finger_joint': 0,
       'head_1_joint': 0,
       'head_2_joint': 0
    },
    'open': {
       'torso_lift_joint': 0.35,
       'arm_1_joint': 1.5,
       'arm_2_joint': 0,
       'arm_3_joint': 0,
       'arm_4_joint': 0,
       'arm_5_joint': 0,
       'arm_6_joint': 0,
       'arm_7_joint': -1.5,
       'gripper_left_finger_joint': 0.04,
       'gripper_right_finger_joint': 0.04,
       'head_1_joint': 0,
       'head_2_joint': 0
    }
}

robot_encoders = {
       'torso_lift_joint': 'torso_lift_joint_sensor',
       'head_1_joint': 'head_1_joint_sensor',
       'head_2_joint': 'head_2_joint_sensor'
    }

# create the Robot instance.
robot = Supervisor()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

WP = [(0.92, -0.84), (0.51, -2.84), (-0.53, -3.29), (-1.71, -2.67), (-1.72, 0.3), (0, 0), (-0.99, 0.57), (-1.71, 0.05), (-1.64, -2.8), (-0.53, -3.29), (0.73, -2.15), (0.57, -0.15), (0, 0)]

class DoesMapExist(py_trees.behaviour.Behaviour):
    def update(self):
        file_exists = exists('map.npy')
        if (file_exists):
            print('Map exists')
            return py_trees.common.Status.SUCCESS
        else:
            print("No map found. Start mapping")
            return py_trees.common.Status.FAILURE
        
class FindObject(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(FindObject, self).__init__(name)
        self.robot = blackboard.read('robot')
        self.blackboard = blackboard
    def setup(self):
        # enable sensors
        self.timestep = int(self.robot.getBasicTimeStep())
        self.camera = self.robot.getDevice("camera")
        self.camera.enable(self.timestep)
        self.left_motor = self.robot.getDevice('wheel_left_joint')
        self.left_motor.setPosition(float('inf'))
        self.right_motor = self.robot.getDevice('wheel_right_joint')
        self.right_motor.setPosition(float('inf'))
        for _, sensor_name in robot_encoders.items():
            sensor = self.robot.getDevice(sensor_name)
            if sensor:
                sensor.enable(self.timestep)
    def update(self):
        self.left_motor.setVelocity(0.5)
        self.right_motor.setVelocity(-0.5)
        
        self.camera.recognitionEnable(self.timestep)
        self.objects = self.camera.getRecognitionObjects()
        if len(self.objects) > 0:
            self.left_motor.setVelocity(0)
            self.right_motor.setVelocity(0)
            for object in self.objects:
                print(list(object.getPosition()))
            current_object = self.objects[0]
            # TO DO: recalculate to base robot coordinates
            current_object_position = list(current_object.getPosition())
            self.blackboard.write('objects', current_object_position)
            sensor_values = {}
            for _, sensor_name in robot_encoders.items():
                sensor = self.robot.getDevice(sensor_name)
                if sensor:
                    current_value = sensor.getValue()
                    sensor_values[sensor_name] = current_value
            print(camera_to_base(current_object_position[0], current_object_position[1], current_object_position[2], sensor_values['torso_lift_joint_sensor'], sensor_values['head_1_joint_sensor'], sensor_values['head_2_joint_sensor']))
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

class Stop(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(Stop, self).__init__(name)
        self.robot = blackboard.read('robot')
        self.blackboard = blackboard
    def setup(self):
        # enable sensors
        self.timestep = int(self.robot.getBasicTimeStep())
        self.left_motor = self.robot.getDevice('wheel_left_joint')
        self.left_motor.setPosition(float('inf'))
        self.right_motor = self.robot.getDevice('wheel_right_joint')
        self.right_motor.setPosition(float('inf'))
    def update(self):
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
        return py_trees.common.Status.RUNNING
             
class Blackboard:
    def __init__(self):
        self.data = {}
    def write(self, key, value):
        self.data[key] = value
    def read(self, key):
        return self.data[key]

blackboard = Blackboard()
blackboard.write('robot', robot)
blackboard.write('waypoints', WP)

tree = Sequence("Main", children=[
            SetPosition("Put arm in target position", blackboard, positions['safe']),
            Selector("Does map exist?", children=[
                DoesMapExist("Test the map"),
                Parallel("Mapping", policy=py_trees.common.ParallelPolicy.SuccessOnOne(), children=[
                    Mapping("Map the environment", blackboard),
                    Navigation("Move around the table", blackboard)
                ])
            ], memory=True),
            Planning("Compute path to the counter", blackboard, (0.71, 0.41)),
            Navigation("Move to the counter", blackboard),
            FindObject("Find target object to grasp", blackboard),
            SetPosition("Put arm in target position", blackboard, positions['open']),
            Stop("Stop", blackboard)
       ], memory=True)
       
tree.setup_with_descendants()

# Main loop:
while robot.step(timestep) != -1:
    tree.tick_once()
