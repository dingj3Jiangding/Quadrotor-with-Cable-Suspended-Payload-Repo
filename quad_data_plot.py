import pandas as pd
import matplotlib.pyplot as plt

# ===== ① 读 CSV =====
csv_file = "/home/ubuntu2204/quadrotor_with_cable_suspended_payload_repo/control_code/data-2025-06-21-8.csv"     
data = pd.read_csv(csv_file)

fig_pos, axes_pos = plt.subplots(3, 1, figsize=(8, 10), sharex=True)
axes_labels = ['X', 'Y', 'Z']
for i, ax in enumerate(axes_pos, start=1):
    ax.plot(data['t'], data[f'x_{i}'],          label=f'x{i}')
    ax.plot(data['t'], data[f'x_des_{i}'], '--', label=f'x_des{i}')
    ax.set_ylabel(f'{axes_labels[i-1]} [m]')
    ax.grid(True)
    ax.legend(loc="upper right")
axes_pos[-1].set_xlabel('time [s]')
fig_pos.suptitle('Position Tracking')

fig_vel, axes_vel = plt.subplots(3, 1, figsize=(8, 10), sharex=True)
for i, ax in enumerate(axes_vel, start=1):
    ax.plot(data['t'], data[f'v_{i}'],          label=f'v{i}')
    ax.plot(data['t'], data[f'v_des_{i}'], '--', label=f'v_des{i}')
    ax.set_ylabel(f'v{axes_labels[i-1]} [m/s]')
    ax.grid(True)
    ax.legend(loc="upper right")
axes_vel[-1].set_xlabel('time [s]')
fig_vel.suptitle('Velocity Tracking')

# ==== 3. 角速度 ====
fig_w, axes_w = plt.subplots(3, 1, figsize=(8, 10), sharex=True)
for i, ax in enumerate(axes_w, start=1):
    ax.plot(data['t'], data[f'w_{i}'],          label=f'w{i}')
    ax.plot(data['t'], data[f'w_des_{i}'], '--', label=f'w_des{i}')
    ax.set_ylabel(f'ω{axes_labels[i-1]} [rad/s]')
    ax.grid(True)
    ax.legend(loc="upper right")
axes_w[-1].set_xlabel('time [s]')
fig_w.suptitle('Angular-Rate Tracking')

fig_f, ax_f = plt.subplots(figsize=(8, 4))
ax_f.plot(data['t'], data['cmd_f'], label='cmd_f')
ax_f.set_xlabel('time [s]')
ax_f.set_ylabel('thrust command  [N]')  
ax_f.set_title('Commanded Thrust')
ax_f.grid(True)
ax_f.legend()

plt.tight_layout()
plt.show()
