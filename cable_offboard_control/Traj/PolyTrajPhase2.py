import numpy as np
import minsnap_trajectories as ms

class PolyTrajPhase2(object):
    """
    A composite trajectory: takeoff → transition → main phase 2 polynomial.
    """
    def __init__(self, h0=10.3052, vy0=0.5, vx0=0.5, target_x=10.0014, target_y=10.0014):
        self.h0 = h0
        self.vy0 = vy0
        self.vx0 = vx0
        self.target_x = target_x
        self.target_y = target_y
        self.g = 9.8

        # Total flight time (for final segment)
        # self.flighttime = 6 * np.sqrt(target_x**2 + target_y**2) / np.sqrt(vy0**2 + vx0**2)
        # self.flighttime = 1.0

        # Composite times
        self.t_transition = 4.0
        self.t_main = 3.6
        self.t_trans_hold = 0.5
        self.hold = 5
        self.t_final = self.t_transition + self.t_main

        # Hardcoded target state at start of main trajectory
        x0 = np.array([10.0025, 10.0001, 10.3044])
        v0 = np.array([-0.2105, -0.2130, -0.2937])
        a0 = np.array([-0.1263, -0.1264, 1.0019])

        # Waypoints: Takeoff, Transition
        waypoints = [
            ms.Waypoint(time=0.0, position=np.array([15.0, 15.0, 10.0])),
            ms.Waypoint(time=self.t_transition, position=x0, velocity = v0, acceleration = a0),
        ]

        # Generate initial segments (2 of them)0014
        t_ref, refs, poly_segs = ms.generate_trajectory(
            waypoints,
            degree=7,
            idx_minimized_orders=(3, 4),
            num_continuous_orders=4,
            algorithm="closed-form"
        )

        # Hardcoded segment (third)
        # coeffs_x = np.array([10.0025, -0.2102, -0.0692, -0.0309, -0.8646, 0.5947, -0.1395, 0.0112])
        # coeffs_y = np.array([10.0001, -0.2141, -0.0701, -0.0310, -0.8615, 0.5928, -0.1391, 0.0111])
        # coeffs_z = np.array([10.3044, -0.2101,  0.0009,  0.0036,  1.0638, -0.7065, 0.1631, -0.0129])

        coeffs_x = np.array([10.0046124690190, -0.976664985294321, -0.0990553881399172, -0.0431355570354469, -0.499839540557793, 0.371145295974661, -0.0902435626373021, 0.00737387384988523])
        coeffs_y = np.array([10.0029383111661, -0.987668172833212, -0.100007596352331, -0.0434056857541643, -0.493739076439916, 0.367431522570830, -0.0894264272242574, 0.00731114575045867])
        coeffs_z = np.array([10.3051065093192, -0.983231511726613, -9.66209436457588e-07, 0.00506042872436662, 1.39418574715068, -0.913301036901987, 0.209081602290375, -0.0164690271950580])

        hardcoded_seg = np.zeros((1, 8, 3))
        hardcoded_seg[0, :, 0] = coeffs_x
        hardcoded_seg[0, :, 1] = coeffs_y
        hardcoded_seg[0, :, 2] = coeffs_z

        # x_end = hardcoded_seg[0, 0, :]
        # hold_waypoints = [
        #     ms.Waypoint(time=0.0, position=x_end),
        #     ms.Waypoint(time=self.t_trans_hold, position=x_end, velocity=np.zeros(3), acceleration=np.zeros(3))
        # ]

        # _, refs_hold, poly_hold = ms.generate_trajectory(
        #     hold_waypoints,
        #     degree=7,
        #     idx_minimized_orders=(3, 4),
        #     num_continuous_orders=4,
        #     algorithm="closed-form"
        # )

        # Combine all segments into one poly array
        poly_combined = np.concatenate((poly_segs, hardcoded_seg), axis=0)

        # New t_ref for 3 segments
        self.t_ref = np.array([
            0.0,
            self.t_transition,
            self.t_transition + self.t_main,
            # self.t_final
        ])

        self.polys = (self.t_ref, refs , poly_combined)

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
