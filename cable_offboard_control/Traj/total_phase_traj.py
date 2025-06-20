# my_trajs.py  —— 专管轨迹
import numpy as np


class TrajectoryBase:

    def __init__(self, coeff_x, coeff_y, coeff_z, t_final):
        self.cx = np.asarray(coeff_x, dtype=float)
        self.cy = np.asarray(coeff_y, dtype=float)
        self.cz = np.asarray(coeff_z, dtype=float)

        self.cx_d1, self.cx_d2 = np.polyder(self.cx), np.polyder(np.polyder(self.cx))
        self.cy_d1, self.cy_d2 = np.polyder(self.cy), np.polyder(np.polyder(self.cy))
        self.cz_d1, self.cz_d2 = np.polyder(self.cz), np.polyder(np.polyder(self.cz))
        self.t_final = float(t_final)

    def update(self, t):
        t = float(np.clip(t, 0.0, self.t_final))
        x  = np.polyval(self.cx,  t)
        y  = np.polyval(self.cy,  t)
        z  = np.polyval(self.cz,  t)
        vx = np.polyval(self.cx_d1, t)
        vy = np.polyval(self.cy_d1, t)
        vz = np.polyval(self.cz_d1, t)
        ax = np.polyval(self.cx_d2, t)
        ay = np.polyval(self.cy_d2, t)
        az = np.polyval(self.cz_d2, t)
        return {
            "x":      np.array([x, y, z], dtype=float),
            "x_dot":  np.array([vx, vy, vz], dtype=float),
            "x_ddot": np.array([ax, ay, az], dtype=float),
            "yaw": 0., 
            "yaw_dot": 0., 
            "yaw_ddot": 0.
        }

# higher oder comes first!
class PolyTrajPhase1(TrajectoryBase):
    def __init__(self, t_final=3.69):
        coeff_file="Traj/quad_load/phase1_coeff.csv"
        coeffs = np.loadtxt(coeff_file, delimiter=',')
        coeff_x = coeffs[0]
        coeff_y = coeffs[1]
        coeff_z = coeffs[2]
        super().__init__(coeff_x, coeff_y, coeff_z, t_final)


class PolyTrajPhase2(TrajectoryBase):

    def __init__(self, t_final=4.5):
        coeff_file="Traj/quad_load/phase2_coeff.csv"
        coeffs = np.loadtxt(coeff_file, delimiter=',')
        coeff_x = coeffs[0]
        coeff_y = coeffs[1]
        coeff_z = coeffs[2] 
        super().__init__(coeff_x, coeff_y, coeff_z, t_final)
