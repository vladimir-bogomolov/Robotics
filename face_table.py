import py_trees
import numpy as np
from map_helpers import world2map

class FaceTable(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(FaceTable, self).__init__(name)
        self.robot = blackboard.read('robot')
        self.blackboard = blackboard
        
    def setup(self):
        self.timestep = int(self.robot.getBasicTimeStep())
        # Devices
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(self.timestep)
        self.left_motor = self.robot.getDevice('wheel_left_joint')
        self.left_motor.setPosition(float('inf'))
        self.right_motor = self.robot.getDevice('wheel_right_joint')
        self.right_motor.setPosition(float('inf'))
        self.display = self.robot.getDevice('display')
                
    def initialise(self):
        # Single waypoint
        self.target = self.blackboard.read('table')
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
        print('Facing target point ', self.target)
        
    def update(self):
        # Get robot position
        xw, yw = self.gps.getValues()[0], self.gps.getValues()[1]

        # Robot orientation
        robot_angle = np.arctan2(self.compass.getValues()[0], self.compass.getValues()[1])

        # Angle to target
        target_angle = np.arctan2(self.target[1] - yw, self.target[0] - xw)
        angle_error = target_angle - robot_angle

        # Normalize to [-pi, pi]
        if angle_error > np.pi:
            angle_error -= 2 * np.pi
        if angle_error < -np.pi:
            angle_error += 2 * np.pi

        # Rotation speed
        kp = 2.0
        left_speed = -kp * angle_error
        right_speed = kp * angle_error

        # Clamp speeds
        max_speed = 6.28
        left_speed = np.clip(left_speed, -max_speed, max_speed)
        right_speed = np.clip(right_speed, -max_speed, max_speed)

        # Apply velocities
        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)

        # Optional display
        xd, yd = world2map(xw, yw)
        self.display.setColor(0xFCBA03)
        self.display.drawPixel(xd, yd)

        # Finished if within 5 degrees
        if abs(angle_error) < np.deg2rad(5):
            print('Target faced')
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING
        
    def terminate(self, new_status):
        print("Finished facing point")
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
