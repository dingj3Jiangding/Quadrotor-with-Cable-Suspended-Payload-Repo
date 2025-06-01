import numpy as np
import minsnap_trajectories as ms

class HoldTrajectory(object):
    """
    A simple trajectory that holds position (1, 1, 2) for 10 seconds.
    """
    def __init__(self, hold_position=np.array([1.0, 1.0, 2.0]), duration=10.0):
        self.duration = duration
        self.t_final = duration
        self.t_ref = np.array([0.0, duration])
        
        # Create two identical waypoints at t=0 and t=duration
        hold_wp_1 = ms.Waypoint(
            time=0.0,
            position=hold_position,
            velocity=np.zeros(3),
            acceleration=np.zeros(3)
        )
        hold_wp_2 = ms.Waypoint(
            time=duration,
            position=hold_position,
            velocity=np.zeros(3),
            acceleration=np.zeros(3)
        )

        t_ref, refs, polys = ms.generate_trajectory(
            [hold_wp_1, hold_wp_2],
            degree=7,
            idx_minimized_orders=(3, 4),
            num_continuous_orders=4,
            algorithm="closed-form"
        )

        self.polys = (t_ref, refs, polys)
        self.t_ref = t_ref

    def update(self, t):
        """
        Return flat output at time t.
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
