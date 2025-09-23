import numpy as np

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
       'arm_1_joint': 1.71,
       'arm_2_joint': 0,
       'arm_3_joint': 0,
       'arm_4_joint': 0,
       'arm_5_joint': 0,
       'arm_6_joint': 0,
       'arm_7_joint': -1.5,
       'gripper_left_finger_joint': 0.044,
       'gripper_right_finger_joint': 0.044,
       'head_1_joint': 0,
       'head_2_joint': -0.3
    }
}

robot_camera_position_encoders = {
       'torso_lift_joint': 'torso_lift_joint_sensor',
       'head_1_joint': 'head_1_joint_sensor',
       'head_2_joint': 'head_2_joint_sensor'
    }

def R_y(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[ c, 0, s],
                     [ 0, 1, 0],
                     [-s, 0, c]])

def R_z(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[ c,-s, 0],
                     [ s, c, 0],
                     [ 0, 0, 1]])

def homog(R, t):
    T = np.eye(4)
    T[:3,:3] = R
    T[:3,3]  = t
    return T

def camera_to_base(x_cam, y_cam, z_cam,
                   q_torso, q_pan, q_tilt):
    """
    Convert coordinates from camera frame to base frame.
    Uses translations + pan/tilt rotations 
    """

    # --- constants from robot devices tramslations ---
    t_torso = np.array([0.0, 0.0, 0.0])   # torso base offset
    t_head1 = np.array([0.182, 0.0, 0.0]) # torso->head_1
    t_head2 = np.array([0.005, 0.0, 0.098]) # head_1->head_2
    t_cam   = np.array([0.186, 0.1254, -0.009]) # head_2->camera

    # --- homogeneous transforms ---
    # base -> torso
    T01 = homog(np.eye(3), t_torso + np.array([0,0,q_torso]))

    # torso -> head_1 (rotation about z, then translate)
    R_pan = R_z(q_pan)
    T12 = homog(R_pan, R_pan @ t_head1)

    # head_1 -> head_2 (rotation about y, then translate)
    R_tilt = R_y(q_tilt)
    T23 = homog(R_tilt, R_tilt @ t_head2)

    # head_2 -> camera (pure translation, no rotation)
    T34 = homog(np.eye(3), t_cam)

    # total
    T03 = T01 @ T12 @ T23 @ T34

    # transform point
    p_cam = np.array([x_cam, y_cam, z_cam, 1.0])
    p_base = T03 @ p_cam
    return p_base[:3]

def find_object(camera, timestep, robot):
    camera.recognitionEnable(timestep)
    objects = camera.getRecognitionObjects()
    if len(objects) > 0:
        current_object = objects[0]
        current_object_position = list(current_object.getPosition())
        sensor_values = {}
        for _, sensor_name in robot_camera_position_encoders.items():
            sensor = robot.getDevice(sensor_name)
            if sensor:
                current_value = sensor.getValue()
                sensor_values[sensor_name] = current_value
        found_obj_coordinates = camera_to_base(current_object_position[0], current_object_position[1], current_object_position[2], sensor_values['torso_lift_joint_sensor'], sensor_values['head_1_joint_sensor'], sensor_values['head_2_joint_sensor'])
        print("I see the object at", found_obj_coordinates)
        return (True, found_obj_coordinates)
    return (False, [])
