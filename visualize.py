import pandas as pd
import matplotlib.pyplot as plt

data = pd.read_csv("motion.csv")

plt.plot(data["true_x"], data["true_y"], label="True Path", linewidth=2)   #plotting original data
plt.scatter(data["meas_x"], data["meas_y"], label="Measurements", color="gray", s=10)  #ploting noisy measurement 
plt.plot(data["est_x"], data["est_y"], label="Kalman Estimate", linestyle='--')  #ploting kalman pediction

plt.xlabel("x")
plt.ylabel("y")
plt.legend()
plt.title("2D Kalman Filter Simulation")
plt.grid(True)
plt.show()
