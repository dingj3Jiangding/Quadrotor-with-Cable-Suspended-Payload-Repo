import numpy as np
import pandas as pd
import minsnap_trajectories as ms

class PolyTrajPhase1_2(object):
    """
    A composite trajectory: move to (5,5,5) → interpolated → main phase 2 polynomial.
    """
    def __init__(self, h0=2.0, vy0=0.5, vx0=0.5, target_x=5.0, target_y=5.0):
        self.h0 = h0
        self.vy0 = vy0
        self.vx0 = vx0
        self.target_x = target_x
        self.target_y = target_y
        self.g = 9.8

        # Total flight time (for final segment)
        # self.flighttime = 6 * np.sqrt(target_x**2 + target_y**2) / np.sqrt(vy0**2 + vx0**2)

        # Load CSVs and subsample to 500 points
        time_arr = pd.read_csv("Traj/quad_load/time_arr.csv", header=None).values.squeeze()
        xQ = pd.read_csv("Traj/quad_load/xQ.csv", header=None).values
        vQ = pd.read_csv("Traj/quad_load/vQ.csv", header=None).values
        aQ = pd.read_csv("Traj/quad_load/aQ.csv", header=None).values
        
        print(xQ.shape)

        total_points = time_arr.shape[0]
        sample_indices = np.linspace(0, total_points - 1, 1000, dtype=int)

         # Step 1: Add a segment to move to (5,5,3)
        t_start = 0.0
        self.t_transition = 4.0  # duration to reach (5,5,3)
        
        phase1_t_arr = time_arr[sample_indices]
        xQ_sub = xQ[:, sample_indices]
        vQ_sub = vQ[:, sample_indices]
        aQ_sub = aQ[:, sample_indices]

       

        # Composite times
        self.phase_2 = 3.6
        self.t_final = phase1_t_arr[-1] + self.t_transition + self.phase_2

        pre_waypoints = [
            ms.Waypoint(time=t_start, position=np.array([0.0, 0.0, 8.0])),
            ms.Waypoint(
                time=self.t_transition,
                position=xQ_sub[:, 0],
                velocity=vQ_sub[:, 0],
                acceleration=aQ_sub[:, 0]
            )
        ]

        t_pre, refs_pre, polys_pre = ms.generate_trajectory(
            pre_waypoints,
            degree=7,
            idx_minimized_orders=(3, 4),
            num_continuous_orders=4,
            algorithm="closed-form"
        )

        # Step 2: Intermediate trajectory segment
        intermediate_waypoints = []
        for i in range(len(phase1_t_arr)):
            wp = ms.Waypoint(
                time=phase1_t_arr[i] + self.t_transition,  # shift time to start after pre segment
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

        # Step 3: Hardcoded main trajectory segment
        coeffs_x = np.array([10.0046124690190, -0.976664985294321, -0.0990553881399172, -0.0431355570354469, -0.499839540557793, 0.371145295974661, -0.0902435626373021, 0.00737387384988523])
        coeffs_y = np.array([10.0029383111661, -0.987668172833212, -0.100007596352331, -0.0434056857541643, -0.493739076439916, 0.367431522570830, -0.0894264272242574, 0.00731114575045867])
        coeffs_z = np.array([10.3051065093192, -0.983231511726613, -9.66209436457588e-07, 0.00506042872436662, 1.39418574715068, -0.913301036901987, 0.209081602290375, -0.0164690271950580])
        
        hardcoded_seg = np.zeros((1, 8, 3))
        hardcoded_seg[0, :, 0] = coeffs_x
        hardcoded_seg[0, :, 1] = coeffs_y
        hardcoded_seg[0, :, 2] = coeffs_z

        # Combine all segments into one poly array
        poly_combined = np.concatenate((polys_pre, polys_intermediate, hardcoded_seg), axis=0)

        t_hardcoded_end = self.t_transition + phase1_t_arr[-1] + self.phase_2

        t_int = phase1_t_arr + self.t_transition

        self.t_ref = np.concatenate((t_pre, t_int[1:], np.array([t_hardcoded_end])), axis=0)

        # # Update t_ref for all segments
        # self.t_ref = np.array([
        #     t_start,
        #     self.t_transition,
        #     self.t_transition + phase1_t_arr[1],
        #     self.t_transition + phase1_t_arr[2],
        #     self.t_transition + phase1_t_arr[-1],
        #     self.t_transition + phase1_t_arr[-1] + self.phase_2
        # ])

        # refs just needs to be passed along with the poly segments
        self.polys = (self.t_ref, refs_pre + refs_intermediate, poly_combined)

    def update(self, t):
        """
        Given the present time, return the desired flat output and derivatives.
        """
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
