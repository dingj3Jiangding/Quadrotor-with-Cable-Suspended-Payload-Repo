import numpy as np
import os
import csv
from datetime import datetime
# 假设您的控制器函数和工具函数在同一个目录或已正确安装
from .load_position_controller import load_position_controller
from .load_attitude_controller import load_attitude_controller
from .quadrotor_attitude_controller import quadrotor_attitude_controller
from .vec_2_skew import vec_2_skew
from .skew_2_vec import skew_2_vec
from .rot_2_eul import rot_2_eul


class LoadController:
    def __init__(self, params, logging=True, log_dir=None):
        
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

        self.q_d_m1 = np.zeros(3)
        self.q_d_m2 = np.zeros(3)
        self.R_d_m1 = np.eye(3)
        self.Omega_d_m1 = np.zeros(3)
        
        self.integral_xL_error = np.zeros(3)

        self.la_counter = 0
        self.lp_counter = 0
        self.qa_counter = 0

        self.dq_d_prev = np.zeros(3)

        # 日志设置
        self.logging = logging
        if self.logging:
            if log_dir is None:
               
                log_dir = os.getcwd()
                
            date_str = datetime.now().strftime("%Y-%m-%d")

            # log_files_dir = os.path.join(log_dir, "log_files") 
            log_files_dir = log_dir 
            os.makedirs(log_files_dir, exist_ok=True)

            existing_files = [f for f in os.listdir(log_files_dir) if f.startswith(f"load_data-{date_str}") and f.endswith(".csv")]
            file_number = len(existing_files) + 1
            filename = f"load_data-{date_str}-{file_number}.csv"
            self.log_path = os.path.join(log_files_dir, filename) 
            
            self.log_file = open(self.log_path, mode='w', newline='')
            self.writer = csv.writer(self.log_file)
            self.writer.writerow([
                # 't',
                # 'xQ_1', 'xQ_2', 'xQ_3',
                # 'xL_1', 'xL_2', 'xL_3',
                # 'xd_1', 'xd_2', 'xd_3',
                # 'q_1', 'q_2', 'q_3',
                # 'q_d_1', 'q_d_2', 'q_d_3',
                # 'dq_1', 'dq_2', 'dq_3',
                # 'f',
                # 'F_1', 'F_2', 'F_3',
                # 'taut_flag',
                # 'integral_xL_error_1', 'integral_xL_error_2', 'integral_xL_error_3',
                # 'e_x_1', 'e_x_2', 'e_x_3',                 # ✅ 新增误差项
                # 'e_v_1', 'e_v_2', 'e_v_3',
                # 'Omega_cmd_1', 'Omega_cmd_2', 'Omega_cmd_3'
                't',
                # ★ 传感状态 (xQ/vQ 已含 xL/vL)
                'xQ_1','xQ_2','xQ_3',
                'vQ_1','vQ_2','vQ_3',            # ← 新增
                'xL_1','xL_2','xL_3',
                'vL_1','vL_2','vL_3',            # ← 新增
                # ★ 参考轨迹
                'xd_1','xd_2','xd_3',
                'vd_1','vd_2','vd_3',            # ← 新增
                'ad_1','ad_2','ad_3',            # ← 新增
                # ★ load-position I/O
                'q_1','q_2','q_3',
                'dq_1','dq_2','dq_3',
                'q_d_1','q_d_2','q_d_3',
                'Fn_1','Fn_2','Fn_3',            # ← 新增 (位置层输出)
                # ★ load-attitude I/O
                'f_scalar',
                'F_att_1','F_att_2','F_att_3',   # ← 已有但改成更清晰名字
                # ★ quad-attitude I/O
                # 'M_1','M_2','M_3',               # ← 新增 (姿态层输出)
                'Omega_cmd_1','Omega_cmd_2','Omega_cmd_3',
                # 'R_d_11','R_d_12','R_d_13',      # ← 如想记录 R_d，可按需保留/删
                # 'R_d_21','R_d_22','R_d_23',
                # 'R_d_31','R_d_32','R_d_33',
                # ★ 误差与标志
                'e_x_1','e_x_2','e_x_3',
                'e_v_1','e_v_2','e_v_3',
                'int_e_x_1','int_e_x_2','int_e_x_3',
                # 'taut_flag'
            ])
#            print(f"Logging data to: {self.log_path}")


    def update(self, dt, state, ref, t=None):
        # 解包状态
        xQ = state['xQ']
        vQ = state['vQ']
        R = state['R']
        Omega = state['Omega']
        xL = state['xL']
        vL = state['vL']
        # w = state['w'] # 'w' (负载角速度) 在您的MATLAB代码中是w_0，并且在taut_dynamics中计算得到
                       # 在Python中，如果不是直接从状态中传入，需要确保其来源或计算方式

        # 判断缆绳是否拉紧
        delta = xL - xQ
        delta_norm = np.linalg.norm(delta)

        # taut_threshold_lower = self.l - 0.01 
        # taut_threshold_upper = self.l + 0.01

        # is_taut = (delta_norm >= taut_threshold_lower) and (delta_norm < taut_threshold_upper)

        q = delta / delta_norm 
        dq = -(vQ - vL) / delta_norm 

#        print(f"[{t:.3f}s] dq = {dq}, ||dq||^2 = {np.dot(dq, dq):.6f}")
#        print(f"vQ = {vQ}, vL = {vL}, vQ - vL = {vQ - vL}")
#        print(f"is_taut = {is_taut}, delta_norm = {delta_norm}")


        # 解包参考轨迹
        xd = ref['x']
        vd = ref['x_dot']
        ad = ref['x_ddot'] # 期望负载加速度
        b1 = ref['b1'] if 'b1' in ref else np.array([0.0, 1.0, 0.0]) # 期望偏航方向

        # --- 计算并累积积分项 ---
        e_x_current = xL - xd
        e_v_current = vL - vd
        
        # 简单的累积积分（前向欧拉）
        self.integral_xL_error += e_x_current * dt

        q_d, Fn_vector = load_position_controller(xL, xd, vL, vd, ad, self.integral_xL_error,
                                           self.mQ, self.mL, self.g, self.l, q, dq, 
                                           self.kx, self.kv, self.kI, self.lp_counter)
#        print(self.lp_counter)

        # 确保 Fn_vector 是 load_attitude_controller 所期望的格式 (应该是3x1的numpy array)
        F_attitude_ctrl_input, f_scalar_thrust_component = load_attitude_controller(
                                                                self.mQ, self.l, R, Fn_vector, 
                                                                q, dq, q_d, 
                                                                self.q_d_m1, self.q_d_m2, dt, 
                                                                self.kq, self.kw, self.la_counter, self.dq_d_prev)
#        print(self.la_counter)

        M_torque, R_d_desired, Omega_d_desired = quadrotor_attitude_controller(
                                                                self.JQ, F_attitude_ctrl_input, b1, R, 
                                                                self.R_d_m1, dt, Omega, 
                                                                self.Omega_d_m1, self.kR, self.kOmega, self.qa_counter)
#       print(self.qa_counter)

        # 更新历史状态，用于下一周期的微分计算 (q_d, R_d, Omega_d)
        self.q_d_m2 = self.q_d_m1.copy()
        self.q_d_m1 = q_d.copy() # q_d 来自位置控制器
        self.R_d_m1 = R_d_desired.copy() # R_d_desired 来自姿态控制器
        self.Omega_d_m1 = Omega_d_desired.copy() # Omega_d_desired 来自姿态控制器
        # self.q_prev = q.copy() # 如果 nontaut 状态的 dq 使用 (q - q_prev)/dt

        # 日志记录
        if self.logging and t is not None:
            log_data = (
                # [t] + list(xQ) + list(xL) + list(xd) + 
                # list(q) + list(q_d) + list(dq) + 
                # [f_scalar_thrust_component] + list(F_attitude_ctrl_input) + 
                # [int(is_taut)] + list(self.integral_xL_error) + 
                # list(e_x_current) + list(e_v_current) + list(Omega_d_desired)
                [t] +
                list(xQ) + list(vQ) +
                list(xL) + list(vL) +
                list(xd) + list(vd) + list(ad) +
                list(q) + list(dq) + list(q_d) +
                list(Fn_vector) +
                [f_scalar_thrust_component] + list(F_attitude_ctrl_input) +
                list(Omega_d_desired) +
                # list(R_d.flatten()) +          
                list(e_x_current) + list(e_v_current) +
                list(self.integral_xL_error) 
                # [int(is_taut)]
            )
            self.writer.writerow(log_data)


        return {
            'f': f_scalar_thrust_component, # 来自 load_attitude_controller
            'M': M_torque,                  # 来自 quadrotor_attitude_controller
            'q': q,
            'dq': dq,
            'q_d': q_d,                     # 来自 load_position_controller
            'F': F_attitude_ctrl_input,     # 来自 load_attitude_controller
            # 'taut': is_taut,                 # 使用修正后的 is_taut
            'Omega_cmd': Omega_d_desired
        }

    def close(self):
        if self.logging:
            if hasattr(self, 'log_file') and not self.log_file.closed:
                self.log_file.close()
#                print(f"Log file closed: {self.log_path}")
