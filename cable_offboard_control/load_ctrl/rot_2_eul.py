import numpy as np
import math
import warnings

def rot_2_eul(R):
    R = R.T

    det_R = R[0,0]*(R[1,1]*R[2,2]-R[1,2]*R[2,1])-R[0,1]*(R[1,0]*R[2,2]-R[1,2]*R[2,0])+R[0,2]*(R[1,0]*R[2,1]-R[1,1]*R[2,0])
    if ( abs(det_R-1) > 1e-4):
        warnings.warn(f"The matrix provided is not a Rotation Matrix, det(R) difference from 1 is {abs(det_R-1):.6e}", RuntimeWarning)
    
    val_for_asin = R[1,2]
    if val_for_asin > 1.0 and np.isclose(val_for_asin, 1.0):
        val_for_asin = 1.0
    elif val_for_asin < -1.0 and np.isclose(val_for_asin, -1.0):
        val_for_asin = -1.0
    
    if not (-1.0 <= val_for_asin <= 1.0):
        warnings.warn(
            f"Value for asin ({R[1,2]:.8f}) is significantly outside [-1,1] even after initial correction. Clipping forcefully.",
            RuntimeWarning
        )
        val_for_asin = np.clip(val_for_asin, -1.0, 1.0)

    phi = math.asin(val_for_asin)
    theta = math.atan2(-R[0,2], R[2,2])
    psi = math.atan2(-R[1,0], R[1,1])
    
    return phi, theta, psi