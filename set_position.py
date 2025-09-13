import py_trees

robot_encoders = {
       'torso_lift_joint': 'torso_lift_joint_sensor',
       'arm_1_joint': 'arm_1_joint_sensor',
       'arm_2_joint': 'arm_2_joint_sensor',
       'arm_3_joint': 'arm_3_joint_sensor',
       'arm_4_joint': 'arm_4_joint_sensor',
       'arm_5_joint': 'arm_5_joint_sensor',
       'arm_6_joint': 'arm_6_joint_sensor',
       'arm_7_joint': 'arm_7_joint_sensor',
       'gripper_left_finger_joint': 'gripper_left_sensor_finger_joint',
       'gripper_right_finger_joint': 'gripper_right_sensor_finger_joint',
       'head_1_joint': 'head_1_joint_sensor',
       'head_2_joint': 'head_2_joint_sensor'
    }

class SetPosition(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard, target):
        super(SetPosition, self).__init__(name)
        self.robot = blackboard.read('robot')
        self.blackboard = blackboard
        self.target = target
    def setup(self):
        # enable sensors
        self.timestep = int(self.robot.getBasicTimeStep())
        for _, sensor_name in robot_encoders.items():
            sensor = self.robot.getDevice(sensor_name)
            if sensor:
                sensor.enable(self.timestep)
        # stop wheels
        self.left_motor = self.robot.getDevice('wheel_left_joint')
        self.left_motor.setPosition(float('inf'))
        self.right_motor = self.robot.getDevice('wheel_right_joint')
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
    def update(self):
        isInPosition = True
        for joint_name, target_value in self.target.items():
            device = self.robot.getDevice(joint_name)
            if device:
                sensor = self.robot.getDevice(robot_encoders[joint_name])
                if sensor:
                    current_value = sensor.getValue()
                    isDeviceInPosition = True
                    if abs(abs(current_value) - abs(target_value)) > 0.02:
                        isDeviceInPosition = False
                isInPosition = isInPosition and isDeviceInPosition
                device.setPosition(target_value)
        if isInPosition:
            print('Arm is in the target position')
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING