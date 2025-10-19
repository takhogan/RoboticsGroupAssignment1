import pandas as pd
import matplotlib.pyplot as plt
from numpy import zeros_like, array
from numpy.linalg import norm

df = pd.read_csv("pid_log.csv")
df['error'] = norm(df[['error_x', 'error_y']], axis=1)
zero_line = zeros_like(df["error"].to_numpy())
plt.plot(df["time"], -df["error"], label="Distance Error")
plt.plot(df["time"], zero_line, label="Goal")
plt.xlabel("Time [s]")
plt.ylabel("Error [m]")
plt.legend()
plt.show()