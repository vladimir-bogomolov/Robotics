import py_trees
from robot_helpers import robot_touch_sensors

class PickTheObject(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(PickTheObject, self).__init__(name)
        self.robot = blackboard.read('robot')
    def setup(self):
        self.timestep = int(self.robot.getBasicTimeStep())
        # self.camera = self.robot.getDevice("camera")
        # self.camera.enable(self.timestep)
        # self.left_motor = self.robot.getDevice('wheel_left_joint')
        # self.left_motor.setPosition(float('inf'))
        # self.right_motor = self.robot.getDevice('wheel_right_joint')
        # self.right_motor.setPosition(float('inf'))
        for _, sensor_name in robot_touch_sensors.items():
                sensor = self.robot.getDevice(sensor_name)
                if sensor:
                    sensor.enable(self.timestep)
    def update(self):
        target_value = 0.04
        is_holding = False
        for joint_name, sensor_name in robot_touch_sensors.items():
            device = self.robot.getDevice(joint_name)
            if device:
                sensor = self.robot.getDevice(sensor_name)
                if sensor:
                    current_value = sensor.getValue()
                    print(joint_name, current_value)
                    if (current_value < 0.0438):
                        is_holding = True
                    device.setPosition(target_value)
        if is_holding:
            # for joint_name, sensor_name in robot_touch_sensors.items():
            #             device = self.robot.getDevice(joint_name)
            #             if device:
            #                 sensor = self.robot.getDevice(sensor_name)
            #                 if sensor:
            #                     current_value = sensor.getValue()
            #                     device.setPosition(current_value)
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status):
        print("Holding the object")
