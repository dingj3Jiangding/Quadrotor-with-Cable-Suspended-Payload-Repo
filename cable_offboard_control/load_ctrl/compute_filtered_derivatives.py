def compute_filtered_derivatives(q_d, q_d_m1, q_d_m2, dt):
    dq_d = (q_d - q_d_m1) / dt
    ddq_d = (q_d - 2 * q_d_m1 + q_d_m2) / (dt ** 2)
    return dq_d, ddq_d
