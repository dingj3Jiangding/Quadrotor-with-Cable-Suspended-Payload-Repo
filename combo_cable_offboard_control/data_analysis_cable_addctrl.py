import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from tkinter import filedialog
import tkinter as tk

# Hide tkinter root window
root = tk.Tk()
root.withdraw()

# Open file dialog
file_path = filedialog.askopenfilename(
    title="Select your CSV log",
    filetypes=(("CSV files", "*.csv"), ("All files", "*.*"))
)

if not file_path:
    print("No file selected.")
    exit()

print(f"📂 Selected file: {file_path}")

# Load CSV with header
data = pd.read_csv(file_path)

# Extract variables
t = data["t"].values
xL = data[["xL_1", "xL_2", "xL_3"]].values
xd = data[["xd_1", "xd_2", "xd_3"]].values
F = data[["F_1", "F_2", "F_3"]].values
M = data[["M_1", "M_2", "M_3"]].values

# --- Plot Position Tracking (Load vs Desired) ---
fig1, axs = plt.subplots(3, 1, figsize=(8, 8))
labels = ['x', 'y', 'z']
for i in range(3):
    axs[i].plot(t, xL[:, i], label='xL_' + labels[i], color='r')
    axs[i].plot(t, xd[:, i], label='xd_' + labels[i], color='b')
    axs[i].set_ylabel('Position [m]')
    axs[i].legend()
axs[2].set_xlabel('Time [s]')
fig1.suptitle('Load Position vs Desired Trajectory')
plt.tight_layout()
fig1.savefig(file_path + "_pos.png", dpi=300)

# --- Plot Force and Moment ---
fig2, axs2 = plt.subplots(3, 1, figsize=(8, 8))
for i in range(3):
    axs2[i].plot(t, F[:, i], label=f"F_{labels[i]}")
    axs2[i].plot(t, M[:, i], label=f"M_{labels[i]}")
    axs2[i].legend()
    axs2[i].set_ylabel('Force / Moment')
axs2[2].set_xlabel("Time [s]")
fig2.suptitle("Thrust Force and Moment Command")
plt.tight_layout()
fig2.savefig(file_path + "_FM.png", dpi=300)

# --- Plot Position Error e_x ---
e_x = data[["e_x_1", "e_x_2", "e_x_3"]].values
fig3, axs3 = plt.subplots(3, 1, figsize=(8, 8))
for i in range(3):
    axs3[i].plot(t, e_x[:, i], label=f"e_x_{labels[i]}")
    axs3[i].set_ylabel('Position Error [m]')
    axs3[i].legend()
axs3[2].set_xlabel("Time [s]")
fig3.suptitle("Position Error $e_x = x_L - x_d$")
plt.tight_layout()
fig3.savefig(file_path + "_e_x.png", dpi=300)

# --- Plot Velocity Error e_v ---
e_v = data[["e_v_1", "e_v_2", "e_v_3"]].values
fig4, axs4 = plt.subplots(3, 1, figsize=(8, 8))
for i in range(3):
    axs4[i].plot(t, e_v[:, i], label=f"e_v_{labels[i]}")
    axs4[i].set_ylabel('Velocity Error [m/s]')
    axs4[i].legend()
axs4[2].set_xlabel("Time [s]")
fig4.suptitle("Velocity Error $e_v = v_L - v_d$")
plt.tight_layout()
fig4.savefig(file_path + "_e_v.png", dpi=300)

f = data["f"].values
# --- Plot Scalar Thrust f ---
fig5, ax5 = plt.subplots(figsize=(8, 4))
ax5.plot(t, f, label="f (scalar thrust)", color='purple')
ax5.set_ylabel("Thrust [N]")
ax5.set_xlabel("Time [s]")
ax5.set_title("Scalar Thrust $f$")
ax5.legend()
plt.tight_layout()
fig5.savefig(file_path + "_f.png", dpi=300)


plt.show()
