import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("ur5_compare_log.csv")

for i in range(1, 8):
    plt.figure(figsize=(10, 5))
    plt.plot(df["time"], df[f"cmd_{i}"], label="Controller raw")
    plt.plot(df["time"], df[f"sent_{i}"], label="Sent to UR5")
    plt.plot(df["time"], df[f"actual_{i}"], label="UR5 actual")

    plt.xlabel("Time (s)")
    plt.ylabel("Joint angle (rad)")
    plt.title(f"Joint {i}: Controller vs UR5")
    plt.legend()
    plt.grid(True)

plt.show()