"""bt_navigation controller."""
from controller import Robot, Supervisor
from os.path import exists
import numpy as np
import py_trees
from py_trees.composites import Sequence, Parallel, Selector
from mapping import Mapping
from navigation import Navigation
from planning import Planning
from scan import FindObject
from picking import PickTheObject
from set_position import SetPosition
from robot_helpers import positions
from backwards import MoveBackward
from face_table import FaceTable

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

class Stop(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(Stop, self).__init__(name)
        self.robot = blackboard.read('robot')
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
blackboard.write('table', (-0.65, -1.43))

tree = Sequence("Main", children=[
            SetPosition("Put arm in target position", blackboard, positions['safe']),
            Selector("Does map exist?", children=[
                DoesMapExist("Test the map"),
                Parallel("Mapping", policy=py_trees.common.ParallelPolicy.SuccessOnOne(), children=[
                    Mapping("Map the environment", blackboard),
                    Navigation("Move around the table", blackboard)
                ])
            ], memory=True),
            Planning("Compute path to the counter", blackboard, (0.4, 0.04)),
            Navigation("Move to the counter", blackboard),
            SetPosition("Put arm in target position", blackboard, positions['open']),
            FindObject("Find target object to grasp", blackboard),
            PickTheObject("Pick the object up", blackboard),
            SetPosition("Put arm in target position", blackboard, positions['holding']),
            MoveBackward("Move to the counter", blackboard),
            Planning("Compute path to the counter", blackboard, (0, 0)),
            Navigation("Move to the counter", blackboard),
            FaceTable("Face the table", blackboard),
            Stop("Stop", blackboard)
       ], memory=True)
       
tree.setup_with_descendants()

# Main loop:
while robot.step(timestep) != -1:
    tree.tick_once()
