import numpy as np
import os
import csv
from datetime import datetime
from .load_position_controller import load_position_controller
from .load_attitude_controller import load_attitude_controller
from .quadrotor_attitude_controller import quadrotor_attitude_controller
from .vec_2_skew import vec_2_skew
from .skew_2_vec import skew_2_vec
from .rot_2_eul import rot_2_eul


class LoadController:
    def __init__(self, params, logging=True, log_dir=None):
        # 控制参数
        self.mQ = params['mQ']
        self.mL = params['mL']
        self.l = params['l']
        self.g = params['g']
        self.JQ = params['JQ']

        # 增益
        self.kx = params['kx']
        self.kv = params['kv']
        self.kI = params['kI']
        self.kq = params['kq']
        self.kw = params['kw']
        self.kR = params['kR']
        self.kOmega = params['kOmega']

        # 初始状态
        self.q_d_m1 = np.zeros(3)
        self.q_d_m2 = np.zeros(3)
        self.R_d_m1 = np.eye(3)
        self.Omega_d_m1 = np.zeros(3)

        # 日志设置
        self.logging = logging
        if self.logging:
            if log_dir is None:
                log_dir = os.getcwd()
            date_str = datetime.now().strftime("%Y-%m-%d")
            existing_files = [f for f in os.listdir(log_dir) if f.startswith(f"load_data-{date_str}") and f.endswith(".csv")]
            file_number = len(existing_files) + 1
            filename = f"load_data-{date_str}-{file_number}.csv"
            self.log_path = os.path.join(log_dir, filename)
            self.log_file = open(self.log_path, mode='w', newline='')
            self.writer = csv.writer(self.log_file)
            self.writer.writerow([
                't',
                'xL_1', 'xL_2', 'xL_3',
                'xd_1', 'xd_2', 'xd_3',
                'q_1', 'q_2', 'q_3',
                'q_d_1', 'q_d_2', 'q_d_3',
                'dq_1', 'dq_2', 'dq_3',
                'f',
                'F_1', 'F_2', 'F_3',
                'M_1', 'M_2', 'M_3',
                'taut_flag'
            ])

    def update(self, dt, state, ref, t=None):
        # 解包状态
        xQ = state['xQ']
        vQ = state['vQ']
        R = state['R']
        Omega = state['Omega']
        xL = state['xL']
        vL = state['vL']
        w = state['w']

        # 判断缆绳是否拉紧
        delta = xQ - xL
        delta_norm = np.linalg.norm(delta)
        taut = np.abs(delta_norm - self.l) < 0.01

        if taut:
            q = delta / self.l
            dq = (vL - vQ) / self.l
        else:
            q = (xL - xQ) / self.l
            dq = np.cross(vQ - vL, q) / self.l

        # 解包参考轨迹
        xd = ref['x']
        vd = ref['x_dot']
        ad = ref['x_ddot']
        b1 = ref['b1'] if 'b1' in ref else np.array([0, 1, 0])

        # 控制器调用
        q_d, Fn = load_position_controller(xL, xd, vL, vd, ad, np.zeros(3),
                                           self.mQ, self.mL, self.g, self.l, q, dq, self.kx, self.kv, self.kI)

        F, f = load_attitude_controller(self.mQ, self.l, R, Fn, q, dq, q_d, self.q_d_m1, self.q_d_m2, dt, self.kq, self.kw)

        M, R_d, Omega_d = quadrotor_attitude_controller(self.JQ, F, b1, R, self.R_d_m1, dt, Omega, self.Omega_d_m1, self.kR, self.kOmega)

        # 更新历史
        self.q_d_m2 = self.q_d_m1.copy()
        self.q_d_m1 = q_d.copy()
        self.R_d_m1 = R_d.copy()
        self.Omega_d_m1 = Omega_d.copy()

        # 日志记录
        if self.logging and t is not None:
            self.writer.writerow(
                [t] + list(xL) + list(xd) + list(q) + list(q_d) + list(dq) + [f] + list(F) + list(M) + [int(taut)]
            )

        return {
            'f': f,
            'M': M,
            'q': q,
            'dq': dq,
            'q_d': q_d,
            'F': F,
            'taut': taut
        }

    def close(self):
        if self.logging:
            self.log_file.close()
