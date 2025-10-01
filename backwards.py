import py_trees
import numpy as np

class MoveBackward(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard, distance=0.7, speed=2.0):
        super(MoveBackward, self).__init__(name)
        self.robot = blackboard.read('robot')
        self.blackboard = blackboard
        self.target_distance = distance  # meters
        self.speed = speed               # magnitude of backward speed

    def setup(self):
        self.timestep = int(self.robot.getBasicTimeStep())

        # GPS
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)

        # Motors
        self.left_motor = self.robot.getDevice('wheel_left_joint')
        self.right_motor = self.robot.getDevice('wheel_right_joint')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))

    def initialise(self):
        # Record starting position
        x, z = self.gps.getValues()[0], self.gps.getValues()[2]
        self.start_pos = np.array([x, z])

        # Start moving backward
        self.left_motor.setVelocity(-self.speed)
        self.right_motor.setVelocity(-self.speed)

    def update(self):
        # Current position
        x, z = self.gps.getValues()[0], self.gps.getValues()[2]
        current_pos = np.array([x, z])

        # Distance traveled
        dist = np.linalg.norm(current_pos - self.start_pos)

        if dist >= self.target_distance:
            # Stop the robot
            self.left_motor.setVelocity(0)
            self.right_motor.setVelocity(0)
            print(f"Moved {self.target_distance} meters backward")
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
