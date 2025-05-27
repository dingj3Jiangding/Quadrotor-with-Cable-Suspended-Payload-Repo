import numpy as np

def load_position_controller(xL, xd, vL, vd, d2_xd, integral_xL, mQ, mL, g, l, q, d_q, kx, kv, kI):
    
    e_x = xL - xd           # load position error
    e_v = vL - vd           # load vel error
    e_I = integral_xL       # for now kI = [0, 0, 0] -- not working

    e3 = np.array([0, 0, 1])
    
    A = (
        - kx.dot(e_x)
        - kv.dot(e_v)
        - kI.dot(integral_xL)
        + (mQ + mL) * (d2_xd + g * e3)
        + mQ * l * np.dot(d_q, d_q) * q
    )

    Fn = np.dot(A, q) * q

    q_d = - A / np.linalg.norm(A)

    return q_d, Fn.flatten()