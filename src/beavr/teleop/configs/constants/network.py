# -----------------------------------------------------------------------------
# Network Configuration Constants
# -----------------------------------------------------------------------------

# Host addresses
HOST_ADDRESS = "10.31.140.117"

# Robot IP addresses
LEFT_XARM_IP = "192.168.1.237"
RIGHT_XARM_IP = "192.168.1.197"
LEFT_ARM_IP = "192.168.86.216"  # For Left XArm
RIGHT_ARM_IP = "192.168.86.230"  # For Right XArm

# LEAP hand USB ports
LEFT_LEAP_USB_PORT = '/dev/ttyUSB1'
RIGHT_LEAP_USB_PORT = '/dev/ttyUSB0'

# Robot IP addresses (alternative naming)
XARM_LEFT_IP = "192.168.1.237"
XARM_RIGHT_IP = "192.168.1.197"

# Network configuration
TELEOP_HANDSHAKE_PORT = 8150

# Deployment constants
DEPLOY_REACH_THRESHOLD = 0.35
DEPLOY_FREQ = 3

# Teleop constants
ARM_TELEOP_CONT = 1
ARM_TELEOP_STOP = 0

# Resolution specific parameters
ARM_HIGH_RESOLUTION = 1  # for arm teleoperation
ARM_LOW_RESOLUTION = 0

LEFT_HAND_PORT = 8110
RIGHT_HAND_PORT = 8087