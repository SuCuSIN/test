import csv
import glob
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import tyro

from gello.env import RobotEnv
from gello.robots.robot import PrintRobot
from gello.utils.launch_utils import instantiate_from_dict
from gello.zmq_core.robot_node import ZMQClientRobot
from gello.agents.sts3215_ur5e_agent import STS3215UR5eAgent


def print_color(*args, color=None, attrs=(), **kwargs):
    import termcolor

    if len(args) > 0:
        args = tuple(termcolor.colored(arg, color=color, attrs=attrs) for arg in args)
    print(*args, **kwargs)


@dataclass
class Args:
    agent: str = "none"
    robot_port: int = 6001
    wrist_camera_port: int = 5000
    base_camera_port: int = 5001
    hostname: str = "127.0.0.1"
    robot_type: str = None  # only needed for quest agent or spacemouse agent
    hz: int = 60
    start_joints: Optional[Tuple[float, ...]] = None

    gello_port: Optional[str] = None
    mock: bool = False
    use_save_interface: bool = False
    data_dir: str = "~/bc_data"
    bimanual: bool = False
    verbose: bool = False

    def __post_init__(self):
        if self.start_joints is not None:
            self.start_joints = np.array(self.start_joints)


def main(args):
    log_rows = []
    start_time = time.time()
    if args.mock:
        if args.agent == "sts3215_ur5e":
            robot_client = PrintRobot(7, dont_print=True)
        else:
            robot_client = PrintRobot(8, dont_print=True)

        camera_clients = {}
    else:
        camera_clients = {
            # you can optionally add camera nodes here for imitation learning purposes
            # "wrist": ZMQClientCamera(port=args.wrist_camera_port, host=args.hostname),
            # "base": ZMQClientCamera(port=args.base_camera_port, host=args.hostname),
        }
        robot_client = ZMQClientRobot(port=args.robot_port, host=args.hostname)
    env = RobotEnv(robot_client, control_rate_hz=args.hz, camera_dict=camera_clients)

    agent_cfg = {}
    if args.bimanual:
        if args.agent == "gello":
            # dynamixel control box port map (to distinguish left and right gello)
            right = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7WBG6A-if00-port0"
            left = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7WBEIA-if00-port0"
            agent_cfg = {
                "_target_": "gello.agents.agent.BimanualAgent",
                "agent_left": {
                    "_target_": "gello.agents.gello_agent.GelloAgent",
                    "port": left,
                },
                "agent_right": {
                    "_target_": "gello.agents.gello_agent.GelloAgent",
                    "port": right,
                },
            }
        elif args.agent == "quest":
            agent_cfg = {
                "_target_": "gello.agents.agent.BimanualAgent",
                "agent_left": {
                    "_target_": "gello.agents.quest_agent.SingleArmQuestAgent",
                    "robot_type": args.robot_type,
                    "which_hand": "l",
                },
                "agent_right": {
                    "_target_": "gello.agents.quest_agent.SingleArmQuestAgent",
                    "robot_type": args.robot_type,
                    "which_hand": "r",
                },
            }
        elif args.agent == "spacemouse":
            left_path = "/dev/hidraw0"
            right_path = "/dev/hidraw1"
            agent_cfg = {
                "_target_": "gello.agents.agent.BimanualAgent",
                "agent_left": {
                    "_target_": "gello.agents.spacemouse_agent.SpacemouseAgent",
                    "robot_type": args.robot_type,
                    "device_path": left_path,
                    "verbose": args.verbose,
                },
                "agent_right": {
                    "_target_": "gello.agents.spacemouse_agent.SpacemouseAgent",
                    "robot_type": args.robot_type,
                    "device_path": right_path,
                    "verbose": args.verbose,
                    "invert_button": True,
                },
            }
        else:
            raise ValueError(f"Invalid agent name for bimanual: {args.agent}")

        # System setup specific. This reset configuration works well on our setup. If you are mounting the robot
        # differently, you need a separate reset joint configuration.
        reset_joints_left = np.deg2rad([0, -90, -90, -90, 90, 0, 0])
        reset_joints_right = np.deg2rad([0, -90, 90, -90, -90, 0, 0])
        reset_joints = np.concatenate([reset_joints_left, reset_joints_right])
        curr_joints = env.get_obs()["joint_positions"]
        max_delta = (np.abs(curr_joints - reset_joints)).max()
        steps = min(int(max_delta / 0.01), 100)

        for jnt in np.linspace(curr_joints, reset_joints, steps):
            env.step(jnt)
    else:
        if args.agent == "gello":
            gello_port = args.gello_port
            if gello_port is None:
                usb_ports = glob.glob("/dev/serial/by-id/*")
                print(f"Found {len(usb_ports)} ports")
                if len(usb_ports) > 0:
                    gello_port = usb_ports[0]
                    print(f"using port {gello_port}")
                else:
                    raise ValueError(
                        "No gello port found, please specify one or plug in gello"
                    )
            agent_cfg = {
                "_target_": "gello.agents.gello_agent.GelloAgent",
                "port": gello_port,
                "start_joints": args.start_joints,
            }
            if args.start_joints is None:
                reset_joints = np.deg2rad(
                    [0, -90, 90, -90, -90, 0, 0]
                )  # Change this to your own reset joints
            else:
                reset_joints = np.array(args.start_joints)

            curr_joints = env.get_obs()["joint_positions"]
            if reset_joints.shape == curr_joints.shape:
                max_delta = (np.abs(curr_joints - reset_joints)).max()
                steps = min(int(max_delta / 0.01), 100)

                for jnt in np.linspace(curr_joints, reset_joints, steps):
                    env.step(jnt)
                    time.sleep(0.001)
        elif args.agent == "quest":
            agent_cfg = {
                "_target_": "gello.agents.quest_agent.SingleArmQuestAgent",
                "robot_type": args.robot_type,
                "which_hand": "l",
            }
        elif args.agent == "spacemouse":
            agent_cfg = {
                "_target_": "gello.agents.spacemouse_agent.SpacemouseAgent",
                "robot_type": args.robot_type,
                "verbose": args.verbose,
            }
        elif args.agent == "sts3215_ur5e":
            agent_cfg = {
                "_target_": "gello.agents.sts3215_ur5e_agent.STS3215UR5eAgent",
                "port": "COM3",
                "baudrate": 1000000,
                "offset_file": "servo_offsets.json",
            }
        elif args.agent == "dummy" or args.agent == "none":
            agent_cfg = {
                "_target_": "gello.agents.agent.DummyAgent",
                "num_dofs": robot_client.num_dofs(),
            }
        elif args.agent == "policy":
            raise NotImplementedError("add your imitation policy here if there is one")
        else:
            raise ValueError("Invalid agent name")

    agent = instantiate_from_dict(agent_cfg)
    # # going to start position
    # print("Going to start position")
    
    # # --- Session alignment: make current controller pose equal to current UR5 pose ---
    # obs_align = env.get_obs()
    # current_align_joints = np.array(obs_align["joint_positions"], dtype=float)
    # raw_align_command = np.array(agent.act(obs_align), dtype=float)

    # session_bias = current_align_joints - raw_align_command

    # print("Session alignment applied.")
    # print("Current UR5 joints :", current_align_joints)
    # print("Raw controller cmd :", raw_align_command)
    # print("Session bias       :", session_bias)

    # print("DEBUG 1: before env.get_obs()")
    # obs0 = env.get_obs() 
    # print("DEBUG 2: after env.get_obs()")

    # print("DEBUG 3: before agent.act()")
    # start_pos = agent.act(obs0)
    # print("DEBUG 4: after agent.act()")
    
    # obs = env.get_obs()
    # joints = obs["joint_positions"]

    # abs_deltas = np.abs(start_pos - joints)
    # id_max_joint_delta = np.argmax(abs_deltas)

    # max_joint_delta = 0.5

    # if abs_deltas[id_max_joint_delta] > max_joint_delta:
    #     id_mask = abs_deltas > max_joint_delta
    #     print("Warning: large initial joint difference detected, but continuing for simulation test.")
    #     ids = np.arange(len(id_mask))[id_mask]

    #     for i, delta, joint, current_j in zip(
    #         ids,
    #         abs_deltas[id_mask],
    #         start_pos[id_mask],
    #         joints[id_mask],
    #     ):
    #         print(
    #             f"joint[{i}]: \t delta: {delta:4.3f} , leader: \t{joint:4.3f} , follower: \t{current_j:4.3f}"
    #         )

    # Do not return during simulation testing
    # return

    # print(f"Start pos: {len(start_pos)}", f"Joints: {len(joints)}")
    # assert len(start_pos) == len(
    #     joints
    # ), f"agent output dim = {len(start_pos)}, but env dim = {len(joints)}"

    # Safer motion settings

    print("Starting relative teleoperation mode")

    # --------------------------------------------------
    # 1. Align current controller pose with current UR5 pose
    # --------------------------------------------------
    obs0 = env.get_obs()
    follower_home_full = np.array(obs0["joint_positions"], dtype=float)

    follower_home = follower_home_full[:6]
    robot_gripper_home = follower_home_full[6]

    leader_home, gripper_home = agent.get_action_with_gripper()
    leader_home = np.array(leader_home, dtype=float)[:6]

    n = 6

    print("Follower arm home:", follower_home)
    print("Robot gripper home:", robot_gripper_home)
    print("Leader arm home:", leader_home)
    print("Leader gripper home:", gripper_home)
    print("DOF:", n)

    # --------------------------------------------------
    # 2. Motion smoothing settings
    # --------------------------------------------------
    max_delta = 0.12      # rad per loop, about 0.86 deg
    alpha = 0.75           # low-pass filter strength
    deadband = 0.003       # ignore tiny controller noise

    smoothed_target = follower_home.copy()

    # --------------------------------------------------
    # 3. Control loop
    # --------------------------------------------------
    try:
        while True:
            obs = env.get_obs()

            current_joints = np.array(obs["joint_positions"], dtype=float)[:n]
            leader_now, gripper_now = agent.get_action_with_gripper()
            leader_now = np.array(leader_now, dtype=float)[:n]

            # Controller movement after connection
            leader_delta = leader_now - leader_home


            # Ignore tiny controller noise
            leader_delta[np.abs(leader_delta) < deadband] = 0.0

            # UR5 target = UR5 start pose + controller movement
            target_joints = follower_home + leader_delta

            # Smooth target
            smoothed_target = (1 - alpha) * smoothed_target + alpha * target_joints

            # Limit movement per loop
            delta = smoothed_target - current_joints
            delta = np.clip(delta, -max_delta, max_delta)

            sent_arm_joints = current_joints + delta

            # Gripper control
            gripper_gain = 4.0

            if gripper_home is None or gripper_now is None:
                target_gripper = np.array(obs["joint_positions"], dtype=float)[6]
            else:
                gripper_delta = (gripper_now - gripper_home) * gripper_gain
                target_gripper = robot_gripper_home + gripper_delta
                target_gripper = np.clip(target_gripper, 0.0, 1.0)

            full_sent_joints = np.array(obs["joint_positions"], dtype=float).copy()
            full_sent_joints[:6] = sent_arm_joints

            full_sent_joints[6] = 1.0

            env.step(full_sent_joints)

            actual_obs = env.get_obs()
            actual_joints = np.array(actual_obs["joint_positions"], dtype=float)[:n]

            t = time.time() - start_time
            log_rows.append(
                [t]
                + list(leader_now)
                + list(target_joints)
                + list(full_sent_joints)
                + list(actual_joints)
            )

            time.sleep(1.0 / args.hz)

    except KeyboardInterrupt:
        print("Stopped by user.")

    finally:
        with open("ur5_compare_log.csv", "w", newline="") as f:
            writer = csv.writer(f)

            header = ["time"]
            header += [f"leader_{i}" for i in range(1, n + 1)]
            header += [f"target_{i}" for i in range(1, n + 1)]
            header += [f"sent_{i}" for i in range(1, 8)]
            header += [f"actual_{i}" for i in range(1, n + 1)]

            writer.writerow(header)
            writer.writerows(log_rows)

        print("Saved log to ur5_compare_log.csv")
    # Warm-up: slowly blend from current UR5 position toward controller command
    # for _ in range(60):
    #     obs = env.get_obs()
    #     raw_command_joints = np.array(agent.act(obs), dtype=float)
    #     command_joints = raw_command_joints + session_bias
    #     current_joints = np.array(obs["joint_positions"], dtype=float)

    #     if smoothed_command is None:
    #         smoothed_command = current_joints.copy()

    #     smoothed_command = (1 - alpha) * smoothed_command + alpha * command_joints

    #     delta = smoothed_command - current_joints
    #     delta = np.clip(delta, -max_delta, max_delta)

    #     sent_joints = current_joints + delta
    #     env.step(sent_joints)

    #     actual_obs = env.get_obs()
    #     actual_joints = actual_obs["joint_positions"]

    #     t = time.time() - start_time
    #     log_rows.append(
    #         [t]
    #         + list(command_joints)
    #         + list(sent_joints)
    #         + list(actual_joints)
    #     )

    #     time.sleep(1.0 / args.hz)

    #     actual_obs = env.get_obs()
    #     actual_joints = actual_obs["joint_positions"]

    #     t = time.time() - start_time
    #     log_rows.append(
    #         [t]
    #         + list(command_joints)
    #         + list(sent_joints)
    #         + list(actual_joints)
    #     )

    #     obs = env.get_obs()
    #     joints = np.array(obs["joint_positions"], dtype=float)
    #     action = np.array(agent.act(obs), dtype=float)

    #     diff = action - joints
    #     if np.abs(diff).max() > 0.5:
    #         print("Warning: action is still far from current joints, but continuing with smoothing.")
    #         joint_index = np.where(np.abs(diff) > 0.5)[0]
    #         for j in joint_index:
    #             print(
    #                 f"Joint [{j}], leader: {action[j]:.3f}, follower: {joints[j]:.3f}, diff: {diff[j]:.3f}"
    #             )

    from gello.utils.control_utils import SaveInterface, run_control_loop

    save_interface = None
    if args.use_save_interface:
        save_interface = SaveInterface(
            data_dir=args.data_dir, agent_name=args.agent, expand_user=True
        )

    #run_control_loop(env, agent, save_interface, use_colors=True)
    # try:
    #     while True:
    #         obs = env.get_obs()
    #         raw_command_joints = np.array(agent.act(obs), dtype=float)
    #         command_joints = raw_command_joints + session_bias
    #         current_joints = np.array(obs["joint_positions"], dtype=float)

    #         smoothed_command = (1 - alpha) * smoothed_command + alpha * command_joints

    #         delta = smoothed_command - current_joints

    #         deadband = 0.01
    #         delta[np.abs(delta) < deadband] = 0.0

    #         delta = np.clip(delta, -max_delta, max_delta)

    #         sent_joints = current_joints + delta
    #         env.step(sent_joints)

    #         actual_obs = env.get_obs()
    #         actual_joints = actual_obs["joint_positions"]

    #         t = time.time() - start_time

    #         log_rows.append(
    #             [t]
    #             + list(command_joints)
    #             + list(sent_joints)
    #             + list(actual_joints)
    #         )

    #         time.sleep(1.0 / args.hz)

    #         actual_obs = env.get_obs()
    #         actual_joints = actual_obs["joint_positions"]

    #         t = time.time() - start_time

    #         log_rows.append(
    #             [t]
    #             + list(command_joints)
    #             + list(sent_joints)
    #             + list(actual_joints)
    #         )

    # except KeyboardInterrupt:
    #     print("Stopped by user.")

    # finally:
    #     with open("ur5_compare_log.csv", "w", newline="") as f:
    #         writer = csv.writer(f)

    #         header = ["time"]
    #         header += [f"cmd_{i}" for i in range(1, 8)]
    #         header += [f"sent_{i}" for i in range(1, 8)]
    #         header += [f"actual_{i}" for i in range(1, 8)]

    #         writer.writerow(header)
    #         writer.writerows(log_rows)

    #     print("Saved log to ur5_compare_log.csv")

if __name__ == "__main__":
    main(tyro.cli(Args))
