import time
from ur5e_agent import STS3215UR5eAgent


def main():
    agent = STS3215UR5eAgent()

    print("Testing clean UR5e joint output with gripper...")
    print("Press Ctrl + C to stop.\n")

    try:
        while True:
            joints, gripper = agent.get_action_with_gripper()

            joints_rounded = [round(value, 4) for value in joints]
            gripper_rounded = round(gripper, 4) if gripper is not None else None

            print("UR5e joints:")
            print(joints_rounded)

            print("Gripper:")
            print(gripper_rounded)

            print("-" * 50)

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        agent.close()


if __name__ == "__main__":
    main()