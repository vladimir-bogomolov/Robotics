import py_trees
from robot_helpers import robot_touch_sensors

class PickTheObject(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(PickTheObject, self).__init__(name)
        self.robot = blackboard.read('robot')
    def setup(self):
        self.timestep = int(self.robot.getBasicTimeStep())
        for device, sensor_name in robot_touch_sensors.items():
                sensor = self.robot.getDevice(sensor_name)
                gripper = self.robot.getDevice(device)
                if gripper:
                    gripper.enableForceFeedback(self.timestep)
                if sensor:
                    sensor.enable(self.timestep)
    def update(self):
        target_value = 0.04
        is_holding = False
        for joint_name, sensor_name in robot_touch_sensors.items():
            device = self.robot.getDevice(joint_name)
            if device:
                if device.getForceFeedback() < -16:
                    print('Holding with Force ', device.getForceFeedback())
                    is_holding = True
                device.setPosition(target_value)
        if is_holding:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status):
        print("Holding the object")
