import numpy as np
def vec_2_skew(w):
    w_skew = np.array([     [0,     -w[2],  w[1]],
                            [w[2],  0,      -w[0]],
                            [-w[1], w[0],   0]])
    return w_skew