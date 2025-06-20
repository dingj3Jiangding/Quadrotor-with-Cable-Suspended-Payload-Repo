import numpy as np
import sympy as sp
from .skew_2_vec import skew_2_vec
from .vec_2_skew import vec_2_skew

def quadrotor_attitude_controller(JQ, F, b1, R, R_d_m1, dt, Omega, Omega_d_m1, kR, kOmega,qa_counter):
    
    b3_d = F / np.linalg.norm(F)
    b1_d = np.cross(b1, b3_d) / np.linalg.norm(np.cross(b1, b3_d))
    b2_d = np.cross(b3_d, b1_d) / np.linalg.norm(np.cross(b3_d, b1_d))

    R_d = np.column_stack((b1_d, b2_d, b3_d))

    # dR_d = np.zeros((3,3))
    
    skew_Omega_d = R_d.T @ R_d_m1
    Omega_d = skew_2_vec(sp.linalg.logm(skew_Omega_d)/dt)

    # dOmega_d = np.array([0.0,0.0,0.0])
    e_R = skew_2_vec(0.5* (R_d.T @ R - R.T @ R_d))
    e_Omega = Omega - R.T @ R_d @ Omega_d

    # M = -kR * e_R - kOmega * e_Omega + np.cross(Omega, JQ @ Omega) - JQ @ (vec_2_skew(Omega) @ R.T @ R_d @ Omega_d - R.T @ R_d @ dOmega_d)
    Omega_cmd = -kR * e_R - kOmega* e_Omega + R.T @ R_d @ Omega_d
    # return M.reshape(3,), R_d, Omega_d
    M = 0
    qa_counter += 1
    return M, R_d, Omega_cmd