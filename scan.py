import py_trees
from robot_helpers import find_object, robot_camera_position_encoders

class FindObject(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(FindObject, self).__init__(name)
        self.robot = blackboard.read('robot')
    def setup(self):
        self.timestep = int(self.robot.getBasicTimeStep())
        self.camera = self.robot.getDevice("camera")
        self.camera.enable(self.timestep)
        self.left_motor = self.robot.getDevice('wheel_left_joint')
        self.left_motor.setPosition(float('inf'))
        self.right_motor = self.robot.getDevice('wheel_right_joint')
        self.right_motor.setPosition(float('inf'))
        for _, sensor_name in robot_camera_position_encoders.items():
                sensor = self.robot.getDevice(sensor_name)
                if sensor:
                    sensor.enable(self.timestep)
    def update(self):
        (found, coordinates) = find_object(self.camera, self.timestep, self.robot)
        if found:
            delta = 0.01

            # Align y axis
            if (coordinates[1] < -delta or coordinates[1] > delta):
                # Aligning Y axis
                if (coordinates[1] < 0):
                    # Turn right
                    self.left_motor.setVelocity(1)
                    self.right_motor.setVelocity(0)
                else:
                    # Turn left
                    self.left_motor.setVelocity(0)
                    self.right_motor.setVelocity(1)
                return py_trees.common.Status.RUNNING
            else:
                # Alining X axis
                arm_length = 1.265
                if (coordinates[0] > arm_length):
                    self.left_motor.setVelocity(0.5)
                    self.right_motor.setVelocity(0.5)
                    return py_trees.common.Status.RUNNING
                else:
                    return py_trees.common.Status.SUCCESS
        else:
            self.left_motor.setVelocity(-1)
            self.right_motor.setVelocity(1)
            return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status):
        print("Ready to grab the object")
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
