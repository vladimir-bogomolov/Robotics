import numpy as np

# Transform world coordinats to display (map) coordinats
def world2map(x, y):
    px = int((x + 2.25) * 40)
    py = int((y - 2) * (-50))
    px = max(0, min(199, px))
    py = max(0, min(299, py))
    return [px, py]

# Transform display (map) coordinats to world coordinats   
def map2world(px, py):
    px = max(0, min(199, px))
    py = max(0, min(299, py))
    # Reverse the transformations
    x = px / 40 - 2.25
    y = py / (-50) + 2
    return [x, y]

# --------------------------
# Rotation matrices
# --------------------------
def R_x(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[1, 0, 0],
                     [0, c, -s],
                     [0, s,  c]])

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
    """Build homogeneous 4x4 from rotation R (3x3) and translation t (3,)"""
    T = np.eye(4)
    T[:3,:3] = R
    T[:3,3]  = t
    return T

# --------------------------
# Build transforms
# --------------------------
def T_0_1(q_torso, base_to_torso_static_z=0.6):
    """Base -> torso lift"""
    t = np.array([0, 0, base_to_torso_static_z + q_torso])
    return homog(np.eye(3), t)

def T_1_2(q_pan, q_tilt, t_head):
    """Torso -> head (pan + tilt + offset)"""
    R = R_z(q_pan) @ R_y(q_tilt)
    t = R_z(q_pan) @ t_head
    return homog(R, t)

def T_2_3(t_cam, R_cam=np.eye(3)):
    """Head -> camera (fixed offset/rotation)"""
    return homog(R_cam, t_cam)

# --------------------------
# Main conversion for camera objects to base coordinates
# --------------------------
def camera_to_base(x_cam, y_cam, z_cam, q_torso, q_pan, q_tilt):
    """
    Convert coordinates from camera frame to base frame.
    
    Args:
        x_cam,y_cam,z_cam : coords in camera frame
        q_torso : torso lift joint value (m)
        q_pan   : head pan joint angle (rad)
        q_tilt  : head tilt joint angle (rad)

    Returns:
        np.array([x_base, y_base, z_base])
    """
    # base_to_torso_static_z : static base->torso offset (default 0.6m)
    base_to_torso_static_z=0.6
    # R_cam   : fixed rotation of camera wrt head (default identity)
    R_cam=np.eye(3)
    # t_head  : np.array([tx,ty,tz]) head mount offset (m)
    t_head = np.array([0.182, 0.0, 0.0])
    # t_cam   : np.array([tx,ty,tz]) camera offset from head (m)
    t_cam = np.array([0.186, 0.1254, -0.009])


    # Build chain
    T01 = T_0_1(q_torso, base_to_torso_static_z)
    T12 = T_1_2(q_pan, q_tilt, t_head)
    T23 = T_2_3(t_cam, R_cam)
    T03 = T01 @ T12 @ T23

    # Point in camera coords
    p_cam = np.array([x_cam, y_cam, z_cam, 1.0])

    # Transform to base coords
    p_base = T03 @ p_cam
    return p_base[:3]
