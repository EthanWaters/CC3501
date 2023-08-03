from pyniryo import *
from pynput import keyboard

DIST_SMOOTHING = 0.1

robot = NiryoRobot("10.10.10.10")
print("hellow")
# robot = NiryoRobot("169.254.200.200")
# robot = NiryoRobot("169.254.101.169")
robot.calibrate_auto()
robot.update_tool()
robot.release_with_tool()
print(robot.get_joints())

# robot.joints = [0.05, -0.1, 0.0, 0.0, 0.0, 0.0]
# robot.joints = [0.07, -0.3, 0.0, 0.0, 0.0, 0.0]
# robot.joints = [0.2, -0.4, 0.0, 0.0, 0.0, 0.0]

positions = [[0.05, -0.1, 0.0, 0.0, 0.0, 0.0],[0.07, -0.3, 0.0, 0.0, 0.0, 0.0], [0.2, -0.4, 0.0, 0.0, 0.0, 0.0]]
list_type = ["joint","joint","joint"]

robot.execute_trajectory_from_poses_and_joints(positions, list_type, DIST_SMOOTHING)
robot.go_to_sleep()
robot.close_connection()