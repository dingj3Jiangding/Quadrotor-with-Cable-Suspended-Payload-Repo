import numpy as np
import pandas as pd

class PolyTrajPhase1(object):
    def __init__(self, h0=2.0, vy0=0.5, vx0=0.5, target_x=5.0, target_y=5.0):
        self.h0 = h0
        self.vy0 = vy0
        self.vx0 = vx0
        self.target_x = target_x
        self.target_y = target_y
        self.g = 9.8

        # Load CSVs and subsample
        time_arr = pd.read_csv("Traj/quad_load/time_arr.csv", header=None).values.squeeze()
        xd = pd.read_csv("Traj/quad_load/xL.csv", header=None).values.T
        vd = pd.read_csv("Traj/quad_load/vL.csv", header=None).values.T
        ad = pd.read_csv("Traj/quad_load/aL.csv", header=None).values.T

        total_points = time_arr.shape[0]

        self.t_ref = time_arr
        self.x_ref = xd
        self.v_ref = vd
        self.a_ref = ad
        self.t_final = self.t_ref[-1]

    def update(self, t):
        t = np.clip(t, self.t_ref[0], self.t_ref[-1] - 1e-5)
        
        idx = np.searchsorted(self.t_ref, t, side = 'right') - 1
        idx = np.clip(idx, 0, len(self.t_ref) - 1)

        x = self.x_ref[idx]
        v = self.v_ref[idx]
        a = self.a_ref[idx]

        yaw = 0
        yaw_dot = 0
        yaw_ddot = 0

        flat_output = {
            'x': x, 
            'x_dot': v, 
            'x_ddot': a, 
            'yaw': yaw, 
            'yaw_dot': yaw_dot, 
            'yaw_ddot': yaw_ddot
        }
        return flat_output
