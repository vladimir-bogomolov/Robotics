import py_trees
import numpy as np
from map_helpers import world2map, map2world

class Navigation(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(Navigation, self).__init__(name)
        self.robot = blackboard.read('robot')
        self.blackboard = blackboard
        
    def setup(self):
        self.timestep = int(self.robot.getBasicTimeStep())
        # get and setup gps, compass, motor devices
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(self.timestep)
        self.left_motor = self.robot.getDevice('wheel_left_joint')
        self.left_motor.setPosition(float('inf'))
        self.right_motor = self.robot.getDevice('wheel_right_joint')
        self.right_motor.setPosition(float('inf'))
        self.marker = self.robot.getFromDef('marker').getField('translation')
        self.display = self.robot.getDevice('display')
                
    def initialise(self):
        self.index = 0
        self.WP = self.blackboard.read('waypoints')
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
        self.printed = set()
        
    def update(self):
        if (not self.index in self.printed):
            if self.index < len(self.WP):
                print('Moving towards waypoint ', self.WP[self.index])
            self.printed.add(self.index)
        # Get GPS
        xw = self.gps.getValues()[0]
        yw = self.gps.getValues()[1]  
        # Read compass values
        robot_angle = np.arctan2(self.compass.getValues()[0],self.compass.getValues()[1])
        # Calculate the error
        distance_error = np.sqrt((xw - self.WP[self.index][0])**2 + (yw - self.WP[self.index][1])**2)
        angle_error = np.arctan2(self.WP[self.index][1] - yw, self.WP[self.index][0] - xw) - robot_angle
        # Adjust the angle error
        if angle_error > np.pi:
            angle_error = angle_error - 2* np.pi
        if angle_error < -np.pi:
            angle_error = angle_error + 2* np.pi
        # Set multipliers
        if (abs(angle_error/3.1415 * 180) > 10):
            # When turning - slow down
            p1 = 3/2
            p2 = 1/2
        else:
            # When going straight - speed up
            p1 = 1/2
            p2 = 9/2
        # Calculate speed
        left_speed = - angle_error * p1 + distance_error * p2
        right_speed = angle_error * p1 + distance_error * p2
        left_speed = max(min(left_speed,6.28),-6.28)
        right_speed = max(min(right_speed,6.28),-6.28)
        # Set marker to a way point
        self.marker.setSFVec3f([*self.WP[self.index], 0])
        # Set speed values for the motors
        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)
        (xd, yd) = world2map(xw, yw)
        self.display.setColor(0xFCBA03)
        self.display.drawPixel(xd, yd)
        
        # When in 0.4 m from the way point - switch to the next point
        if (distance_error < 0.4):
            print('Reached ', self.WP[self.index])
            if self.index + 1 < len(self.WP):
                self.index = self.index + 1
                return py_trees.common.Status.RUNNING
            else:
                # If no more way poins, we are done
                return py_trees.common.Status.SUCCESS
                
        return py_trees.common.Status.RUNNING
        
    def terminate(self, new_status):
        print("Reached the target")
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
        self.index = 0