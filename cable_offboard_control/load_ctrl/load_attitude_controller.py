import numpy as np
from .vec_2_skew import vec_2_skew

def load_attitude_controller(mQ, l, R, Fn, q, d_q, q_d, q_d_m1, q_d_m2, dt, kq, kw, la_counter, dq_d_prev):
    # align the cable direction q with the desired cable direction q_d

    dq_d_alpha = 0.5
    dq_d_raw = (q_d - q_d_m1) / dt
    dq_d = dq_d_alpha * dq_d_prev + (1 - dq_d_alpha) * dq_d_raw
   #  dq_d = np.array([0.0, 0.0, 0.0])
#   d2q_d  = (q_d - 2*q_d_m1 + q_d_m2) /dt**2
    d2q_d = np.array([0.0, 0.0, 0.0])

    e_q = vec_2_skew(q) @ (vec_2_skew(q) @ q_d)       # Correction Direction
    e_dq = d_q - np.cross(np.cross(q_d, dq_d), q)   # Correction Direction between actual angular v and targeted angular v

    Fpd = -kq*e_q - kw*e_dq

    Fff = mQ * l * np.dot(q, np.cross(q_d, dq_d)) * np.cross(q, d_q) + mQ * l * np.cross(np.cross(q_d, d2q_d), q)
    
    F = Fn - Fpd - Fff

    e3 = np.array([0.0,0.0,1.0])

    f = np.dot(F, R @ e3)

    Fn = -np.dot(q_d, q) * q
    
    F = Fn - Fpd - Fff

    la_counter +=1
    dq_d_prev = dq_d

    return F, f
