import pandas as pd
import matplotlib.pyplot as plt

CSV_FILE = "servo_positions.csv"

data = pd.read_csv(CSV_FILE)

plt.figure(figsize=(12, 6))

for column in data.columns:
    if column != "time":
        plt.plot(data["time"], data[column], label=column)

plt.xlabel("Time (s)")
plt.ylabel("Servo Position")
plt.title("STS3215 Servo Position vs Time")
plt.legend()
plt.grid(True)
plt.tight_layout()

plt.show()