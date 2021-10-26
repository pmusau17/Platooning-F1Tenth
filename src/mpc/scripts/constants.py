
# Frame ID shouldn't change - it is defined within the F1/10 ROS simulator
FRAME_ID = "/map"

# Drive topics (closely related to ROS technological stack)
LASER_SCAN_TOPIC = "racecar/scan"
ODOMETRY_TOPIC = "racecar/odom"
DRIVE_TOPIC = "vesc/ackermann_cmd_mux/input/teleop"

# Front and rear wheel distances, wheelbase = lr + lf
LR = 0.17145
LF = 0.15875
WHEELBASE_LENGTH = 0.3302

# Car size
CAR_WIDTH = 0.2032

# Lidar angle increment and minimum angle (max angle is max index * increment + min angle, which gives pi)
LIDAR_MINIMUM_ANGLE = -3.1415927410125732
LIDAR_ANGLE_INCREMENT = 0.005823155865073204

# Max lidar index (max index times lidar angle increment should give 2 * pi)
LIDAR_MAX_INDEX = 1080

# Declare how close should the vehicle be to the lap line to consider as passing through it (this is distance squared)
LAP_FINISH_DISTANCE = 0.01

# Declare how far should the vehicle be after crossing the line to start checking for lap finish (also distance squared)
LAP_CHECK_DISTANCE = 4

# Define parameters here for easier configurability
HORIZON_LENGTH = 6
TIME_STEP = 0.027

# FTG scan angle limits
RIGHT_DIVERGENCE_INDEX = 350
LEFT_DIVERGENCE_INDEX = LIDAR_MAX_INDEX - RIGHT_DIVERGENCE_INDEX

# Any points with this range will be avoided at all cost
FTG_IGNORE_RANGE = 0

# Need to store value for computing most likely position (for better MPC predictions)
POSITION_PREDICTION_TIME = 0.020

# All points around the car within this radius are marked as unsafe
SAFETY_RADIUS = 2
