import numpy as np
import pandas as pd
import minsnap_trajectories as ms

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
        xQ = pd.read_csv("Traj/quad_load/xQ.csv", header=None).values
        vQ = pd.read_csv("Traj/quad_load/vQ.csv", header=None).values
        aQ = pd.read_csv("Traj/quad_load/aQ.csv", header=None).values

        total_points = time_arr.shape[0]
        sample_indices = np.linspace(0, total_points - 1, 300, dtype=int)

        phase1_t_arr = time_arr[sample_indices]
        xQ_sub = xQ[:, sample_indices]
        vQ_sub = vQ[:, sample_indices]
        aQ_sub = aQ[:, sample_indices]

        # Construct intermediate waypoints only
        intermediate_waypoints = []
        for i in range(len(phase1_t_arr)):
            wp = ms.Waypoint(
                time=phase1_t_arr[i],
                position=xQ_sub[:, i],
                velocity=vQ_sub[:, i],
                acceleration=aQ_sub[:, i]
            )
            intermediate_waypoints.append(wp)

        t_intermediate, refs_intermediate, polys_intermediate = ms.generate_trajectory(
            intermediate_waypoints,
            degree=7,
            idx_minimized_orders=(3, 4),
            num_continuous_orders=4,
            algorithm="closed-form"
        )

        self.t_ref = t_intermediate
        self.t_final = self.t_ref[-1]
        self.polys = (self.t_ref, refs_intermediate, polys_intermediate)

    def update(self, t):
        t = np.clip(t, self.t_ref[0], self.t_ref[-1] - 1e-5)
        pvajs = ms.compute_trajectory_derivatives(self.polys, [t], 7)

        x = pvajs[0, :, :].reshape(3,)
        x_dot = pvajs[1, :, :].reshape(3,)
        x_ddot = pvajs[2, :, :].reshape(3,)
        x_dddot = pvajs[3, :, :].reshape(3,)
        x_ddddot = pvajs[4, :, :].reshape(3,)

        yaw = 0
        yaw_dot = 0
        yaw_ddot = 0

        flat_output = {
            'x': x, 'x_dot': x_dot, 'x_ddot': x_ddot, 'x_dddot': x_dddot, 'x_ddddot': x_ddddot,
            'yaw': yaw, 'yaw_dot': yaw_dot, 'yaw_ddot': yaw_ddot
        }
        return flat_output
