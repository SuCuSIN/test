from gello.robots.ur import URRobotimport
import numpy as np

ROBOT_IP = "192.168.0.119"
def main():
    robot = URRobot(robot_ip=ROBOT_IP)

    joints = robot.get_joint_state()

    print("Current UR joint state:")
    print(np.round(joints, 4))

    print("\nDegree:")
    print(np.round(np.degrees(joints), 2))
if __name__ == "__main__":
    main()
