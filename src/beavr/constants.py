import os.path as path
import numpy as np
# VR detector 
# Arm movement
WRIST_HOME_STATE = {
    'translation': [0, 0, 0],
    'rotation_matrix': [
        1, 0, 0,
        0, 1, 0,
        0, 0, -1
    ]
}

# Joint Information
OCULUS_NUM_KEYPOINTS = 24
#VR_THUMB_BOUND_VERTICES = 8
VR_THUMB_BOUND_VERTICES = 4
GRIPPER_OPEN=0
GRIPPER_CLOSE=1

OCULUS_JOINTS = {
    'metacarpals': [2, 6, 9, 12, 15],
    'knuckles': [6, 9, 12, 16],
    'thumb': [2, 3, 4, 5, 19],
    'index': [6, 7, 8, 20],
    'middle': [9, 10, 11, 21],
    'ring': [12, 13, 14, 22],
    'pinky': [15, 16, 17, 18, 23]
}

OCULUS_VIEW_LIMITS = {
    'x_limits': [-0.04, 0.04],
    'y_limits': [-0.02, 0.25],
    'z_limits': [-0.04, 0.04]
}

VR_FREQ = 30
RECORDER_FREQ = 30

# XELA Sensor parameters
XELA_FPS = 100
XELA_NUM_SENSORS = 18 # 3 in thumb 4 in other 3 fingers 
XELA_PALM_NUM_SENSORS = 3
XELA_FINGERTIP_NUM_SENSORS = 4
XELA_FINGER_NUM_SENSORS = 11
XELA_PALM_NUM_TAXELS = 24
XELA_FINGERTIP_NUM_TAXELS= 30
XELA_FINGER_NUM_TAXELS= 16 
XELA_NUM_TAXELS = 16
# Robot parameters

# Allegro and Leap
ALLEGRO_JOINTS_PER_FINGER = LEAP_JOINTS_PER_FINGER = 4
ALLEGRO_JOINT_OFFSETS = LEAP_JOINT_OFFSETS = {
    'index': 0,
    'middle': 4,
    'ring': 8,
    'thumb': 12
}

# Realsense Camera parameters
NUM_CAMS = 4
CAM_FPS = 30
CAM_FPS_SIM = 60
WIDTH = 1280
HEIGHT = 720
PROCESSING_PRESET = 1 # High accuracy post-processing mode
VISUAL_RESCALE_FACTOR = 2
VIZ_PORT_OFFSET = 500
DEPTH_PORT_OFFSET = 1000

# Calibration file paths
CALIBRATION_FILES_PATH = 'calibration_files'
VR_THUMB_BOUNDS_PATH = path.join(CALIBRATION_FILES_PATH, 'vr_thumb_bounds.npy')
VR_DISPLAY_THUMB_BOUNDS_PATH = path.join(CALIBRATION_FILES_PATH, 'vr_thumb_plot_bounds.npy')
VR_2D_PLOT_SAVE_PATH = path.join(CALIBRATION_FILES_PATH, 'oculus_hand_2d_plot.jpg')
XELA_PLOT_SAVE_PATH = path.join(CALIBRATION_FILES_PATH, 'xela_plot.png')


# Data recording parameters - Images are recorded at CAM_FPS rate
IMAGE_RECORD_RESOLUTION = (1280, 720) 
IMAGE_RECORD_RESOLUTION_SIM = (480, 480)
DEPTH_RECORD_FPS = 30
ALLEGRO_SAMPLE_OFFSET = 10 # For sampling states
SAMPLE_WRITER_FPS = 5

# Deployment
DEPLOY_REACH_THRESHOLD = 0.35
DEPLOY_FREQ = 3 

#RESOLUTION SPECIFIC parameters

ARM_HIGH_RESOLUTION = 1 #  for arm teleoperation
ARM_LOW_RESOLUTION = 0


ARM_TELEOP_CONT = 1
ARM_TELEOP_STOP = 0


# Bimanual Robot Constants

XARM_SCALE_FACTOR= 1000

RIGHT_ARM_IP = "192.168.86.230" # For Right XArm
LEFT_ARM_IP = "192.168.86.216" # For Left XArm

BIMANUAL_LEFT_HOME = [206,0,475,3.142,0,0]
BIMANUAL_RIGHT_HOME = [206,0,475,3.142,0,0] 

ROBOT_HOME_POSE_AA = [206.0, -0.0, 475, 3.142, 0.0, 0.0]
ROBOT_HOME_JS = [0.0, -0.4363323129985824, -0.017453292519943295,
                 0.4537856055185257, 0.0, 0.8726646259971648, 0.0]

# -----------------------------------------------------------------------------
# LEAP hand solver scaling factors (used by LeapHandIKSolver)
# -----------------------------------------------------------------------------

LEAP_FINGER_SCALE_FACTOR = 1.8               # default scaling for non-thumb fingertips
LEAP_THUMB_SCALE_FACTOR = 1.7         # separate scaling for thumb positions
LEAP_HOME_JS = np.zeros(16)