import numpy as np
def skew_2_vec(skew_matrix):
    if skew_matrix.shape != (3, 3):
        raise ValueError("Input must be a 3x3 matrix.")
    return np.array([skew_matrix[2, 1], skew_matrix[0, 2], skew_matrix[1, 0]])