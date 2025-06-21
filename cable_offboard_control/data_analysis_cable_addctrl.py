import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from tkinter import filedialog, Tk
from pathlib import Path

plt.rcParams["figure.dpi"] = 120  # 分辨率

# ── 1. 选择 CSV ──────────────────────────────────────────
root = Tk(); root.withdraw()
file_path = filedialog.askopenfilename(
    title="Select log CSV",
    filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
)
if not file_path:
    print("❗ No file selected."); exit()
file_path = Path(file_path)
print(f"📂 Selected: {file_path}")

# ── 2. 读取数据 ──────────────────────────────────────────
data = pd.read_csv(file_path)
t = data["t"].to_numpy()

def cols(prefix):
    return data[[f"{prefix}{i}" for i in (1, 2, 3)]].to_numpy()

xL, vL   = cols("xL_"),  cols("vL_")
xd, vd   = cols("xd_"),  cols("vd_")
ad       = cols("ad_") if {"ad_1","ad_2","ad_3"}.issubset(data) else None
q, dq    = cols("q_"),   cols("dq_")
q_d      = cols("q_d_")
Fn       = cols("Fn_")
F_att    = cols("F_att_")
M        = cols("M_")
Ω_cmd    = cols("Omega_cmd_")
e_x, e_v = cols("e_x_"), cols("e_v_")
int_e_x  = cols("int_e_x_")
f_scalar = data["f_scalar"].to_numpy()

labels = ["x", "y", "z"]

# ── 3. 位置跟踪 ──────────────────────────────────────────
fig, axs = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
for i in range(3):
    axs[i].plot(t, xL[:, i], "r-",  label=f"xL_{labels[i]}")
    axs[i].plot(t, xd[:, i], "b--", label=f"xd_{labels[i]}")
    axs[i].set_ylabel("pos [m]"); axs[i].legend()
axs[-1].set_xlabel("time [s]")
fig.suptitle("Load Position vs Desired"); fig.tight_layout()
fig.savefig(file_path.with_suffix("_pos.png"))

# ── 4. 速度跟踪 ──────────────────────────────────────────
fig, axs = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
for i in range(3):
    axs[i].plot(t, vL[:, i], "r-",  label=f"vL_{labels[i]}")
    axs[i].plot(t, vd[:, i], "b--", label=f"vd_{labels[i]}")
    axs[i].set_ylabel("vel [m/s]"); axs[i].legend()
axs[-1].set_xlabel("time [s]")
fig.suptitle("Load Velocity vs Desired"); fig.tight_layout()
fig.savefig(file_path.with_suffix("_vel.png"))

# ── 5. 误差 e_x & e_v ──────────────────────────────────
for err, name in [(e_x, "e_x"), (e_v, "e_v")]:
    fig, axs = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
    for i in range(3):
        axs[i].plot(t, err[:, i], label=f"{name}_{labels[i]}")
        axs[i].set_ylabel(name); axs[i].legend()
    axs[-1].set_xlabel("time [s]")
    fig.suptitle(f"{name} tracking error"); fig.tight_layout()
    fig.savefig(file_path.with_suffix(f"_{name}.png"))

# ── 6. 力向量比较 ────────────────────────────────────────
fig, axs = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
for i in range(3):
    axs[i].plot(t, Fn[:, i], "g-",  label=f"Fn_{labels[i]}")
    axs[i].plot(t, F_att[:, i], "m--", label=f"F_att_{labels[i]}")
    axs[i].set_ylabel("N"); axs[i].legend()
axs[-1].set_xlabel("time [s]")
fig.suptitle("Position-layer vs Attitude-layer Force"); fig.tight_layout()
fig.savefig(file_path.with_suffix("_forces.png"))

# ── 7. 标量推力 + 力矩 ───────────────────────────────────
fig, axs = plt.subplots(4, 1, figsize=(8, 8), sharex=True)
axs[0].plot(t, f_scalar, "purple", label="f_scalar"); axs[0].legend()
for i in range(3):
    axs[i+1].plot(t, M[:, i], label=f"M_{labels[i]}"); axs[i+1].legend()
axs[0].set_ylabel("N"); [ax.set_ylabel("Nm") for ax in axs[1:]]
axs[-1].set_xlabel("time [s]")
fig.suptitle("Scalar Thrust + Body Moments"); fig.tight_layout()
fig.savefig(file_path.with_suffix("_f_M.png"))

# ── 8. 期望角速度 Ω_cmd ─────────────────────────────────
fig, axs = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
for i in range(3):
    axs[i].plot(t, Ω_cmd[:, i], label=f"Ω_cmd_{labels[i]}"); axs[i].legend()
    axs[i].set_ylabel("rad/s")
axs[-1].set_xlabel("time [s]")
fig.suptitle("Desired Body Angular Velocity Ω_cmd"); fig.tight_layout()
fig.savefig(file_path.with_suffix("_Omega_cmd.png"))

plt.show()
