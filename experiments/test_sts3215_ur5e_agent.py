import time

from gello.agents.sts3215_ur5e_agent import STS3215UR5eAgent


def main():
    agent = STS3215UR5eAgent(
        port="COM3",
        baudrate=1000000,
        offset_file="servo_offsets.json",
    )

    print("STS3215 UR5e GELLO agent test started.")
    print("Press Ctrl + C to stop.\n")

    try:
        while True:
            joints, gripper = agent.get_action_with_gripper()

            print("UR5e joints rad:")
            print([round(v, 4) for v in joints])

            print("Gripper rad:")
            print(round(gripper, 4) if gripper is not None else None)

            print("-" * 50)
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        agent.close()


if __name__ == "__main__":
    main()