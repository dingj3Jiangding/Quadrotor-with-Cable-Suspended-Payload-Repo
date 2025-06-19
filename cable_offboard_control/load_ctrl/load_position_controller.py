import numpy as np

def load_position_controller(xL, xd, vL, vd, d2_xd, integral_xL, mQ, mL, g, l, q, d_q, kx, kv, kI, lp_counter):
    
    e_x = xL - xd           # load position error
    e_v = vL - vd           # load vel error
    e_I = integral_xL      

    e3 = np.array([0, 0, 1])
    
    A = (
        - kx.dot(e_x)
        - kv.dot(e_v)
        - kI.dot(e_I)
        + (mQ + mL) * (d2_xd + g * e3)
        + mQ * l * np.dot(d_q, d_q) * q
    )

    # print("A components debug:", {
    #     "e_x": - e_x,
    #     "e_v": - e_v,
    #     "px": - kx @ e_x,
    #     "pv": - kv @ e_v,
    #     "pi": - kI @ integral_xL,
    #     "traj": (mQ + mL) * (d2_xd + g * np.array([0, 0, 1])),
    #     "swing": mQ * l * np.dot(d_q, d_q) * q
    # })

    Fn = np.dot(A, q) * q

    q_d = - A / np.linalg.norm(A)
    lp_counter +=1

    return q_d, Fn