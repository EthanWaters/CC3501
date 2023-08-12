"""Main code to send to the robot arm containing data collected from embedded circuit on the EduExo Arm
(Curtis & Kai - Group 9) """
from pyniryo import *

robot_ip_address = "192.168.0.176"  # IP address of Ned

shift_pose_value = 0.02

global starting_pose
starting_pose = PoseObject(x=0.174, y=-0.01, z=0.394, roll=2.35, pitch=1.45, yaw=2.23)

from pynput import keyboard


def on_press(key):
    global current_key
    try:
        if key == keyboard.Key.shift:
            print('shift key!')
            current_key = ''
        else:
            current_key = key.char
        # print('current key pressed', current_key)
    except AttributeError:
        print('special key {0} pressed'.format(
            key))
        current_key = ''


def on_release(key):
    global current_key
    current_key = ''
    if key == keyboard.Key.esc:
        return False
    if key == keyboard.Key.ctrl:
        return False
    elif key == keyboard.Key.shift:
        current_key = ''
    else:
        current_key = ''


def niryo_shift(niryo_robot, pose_shift):
    global starting_pose
    starting_pose = starting_pose.copy_with_offsets(x_offset=pose_shift[0], y_offset=pose_shift[1],z_offset=pose_shift[2],
                                                    roll_offset=pose_shift[3],pitch_offset=pose_shift[4], yaw_offset=pose_shift[5])
    #moving
    niryo_robot.move_pose(starting_pose)


if __name__ == '__main__':
    # Connect to robot
    robot = NiryoRobot(robot_ip_address)
    # Changing tool
    robot.update_tool()
    # Calibrate robot if robot needs calibration
    robot.calibrate_auto()
    # starting pose
    (PoseObject(x=0.174, y=-0.01, z=0.394, roll=2.35, pitch=1.45, yaw=2.23))
    robot.set_jog_control(True)

    # Create key    
    current_key = ''
    print("press CTRL or ESC  to finish the program")

    # starts keyboard listener
    listener = keyboard.Listener(
        on_press=on_press,
        on_release=on_release)
    listener.start()

    while listener.is_alive():

        while current_key != '':
            #print'key pressed : ', current_key

            if current_key == 'i':  # i : z axis (+)
                try:
                    robot.move_joints(0.0, 0.0, shift_pose_value, 0.5, 0.0, 0.0)
                    joints_read = robot.get_joints()
                    print(joints_read)
                except:
                    print("not a pose")

            # - SHIFT POSE
            #if current_key == 'i':  # i : z axis (+)
                #try:
                    #robot.jog_joints(0.0, 0.0, shift_pose_value, 0.0, 0.0, 0.0)
                    #joints_read = robot.get_joints()
                    #print(joints_read)
                #except:
                    #print("not a pose")


            elif current_key == 'k':  # k : z axis (-)
                try:
                    robot.jog_joints(0.0, 0.0, -shift_pose_value, -0.5, 0.0, 0.0)
                    joints_read = robot.get_joints()
                    print(joints_read)
                except:
                    print("not a pose")

            elif current_key == 'l':  # l : y axis (+)
                try:
                    robot.jog_joints(0.0, shift_pose_value, 0.0, 0.0, 0.0, 0.0)
                except:
                    print("not a pose")

            elif current_key == 'j':  # j : y axis (-)
                try:
                    robot.jog_joints(0.0, -shift_pose_value, 0.0, 0.0, 0.0, 0.0)
                except:
                    print("not a pose")

            elif current_key == 'p':  # p : x axis (+)
                try:
                    robot.jog_joints(shift_pose_value, 0.0, 0.0, 0.0, 0.0, 0.0)
                except:
                    print("not a pose")

            elif current_key == 'm':  # m : x axis (-)
                try:
                    robot.jog_joints(-shift_pose_value, 0.0, 0.0, 0.0, 0.0, 0.0)
                except:
                    print("not a pose")
            elif current_key == 'p':  # p : x axis (+)
                try:
                    robot.jog_joints(shift_pose_value, 0.0, 0.0, 0.0, 0.0, 0.0)
                except:
                    print("not a pose")

            elif current_key == 'm':  # m : x axis (-)
                try:
                    robot.jog_joints(-shift_pose_value, 0.0, 0.0, 0.0, 0.0, 0.0)
                except:
                    print("not a pose")
            elif current_key == 'p':  # p : x axis (+)
                try:
                    robot.jog_joints(shift_pose_value, 0.0, 0.0, 0.0, 0.0, 0.0)
                except:
                    print("not a pose")

            elif current_key == 'm':  # m : x axis (-)
                try:
                    robot.jog_joints(-shift_pose_value, 0.0, 0.0, 0.0, 0.0, 0.0)
                except:
                    print("not a pose")

    if not listener.running:
        print('keyboard listener died')

    # Ending
    robot.go_to_sleep()
    # Releasing connection
    robot.close_connection()
