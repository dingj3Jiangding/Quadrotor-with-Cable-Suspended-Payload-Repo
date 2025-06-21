import numpy as np
from load_position_controller import load_position_controller
from load_attitude_controller import load_attitude_controller
from quadrotor_attitude_controller import quadrotor_attitude_controller

# Inject utility functions
def vec_2_skew(v):
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])

def skew_2_vec(M):
    return np.array([M[2, 1], M[0, 2], M[1, 0]])

# Patch if needed
import load_attitude_controller as lac
import quadrotor_attitude_controller as qac
lac.vec_2_skew = vec_2_skew
qac.vec_2_skew = vec_2_skew
qac.skew_2_vec = skew_2_vec

class LoadController:
    def __init__(self, params):
        self.mQ = params['mQ']
        self.mL = params['mL']
        self.l = params['l']
        self.g = params['g']
        self.JQ = params['JQ']
        self.kx = params['kx']
        self.kv = params['kv']
        self.kI = params['kI']
        self.kq = params['kq']
        self.kw = params['kw']
        self.kR = params['kR']
        self.kOmega = params['kOmega']
        self.q_d_m1 = np.zeros(3)
        self.q_d_m2 = np.zeros(3)
        self.R_d_m1 = np.eye(3)
        self.Omega_d_m1 = np.zeros(3)

    def update(self, dt, state, ref):
        xQ = state['xQ']
        vQ = state['vQ']
        R = state['R']
        Omega = state['Omega']
        xL = state['xL']
        vL = state['vL']
        w = state['w']
        q = (xL - xQ) / self.l
        dq = (vL - vQ) / self.l
        xd = ref['x']
        vd = ref['xd']
        ad = ref['xdd']
        b1 = ref.get('b1', np.array([0, 1, 0]))

        q_d, Fn = load_position_controller(xL, xd, vL, vd, ad, np.zeros(3),
                                           self.mQ, self.mL, self.g, self.l, q, dq, self.kx, self.kv, self.kI)
        F, f = load_attitude_controller(self.mQ, self.l, R, Fn, q, dq, q_d,
                                        self.q_d_m1, self.q_d_m2, dt, self.kq, self.kw)
        M, R_d, Omega_d = quadrotor_attitude_controller(self.JQ, F, b1, R,
                                                        self.R_d_m1, dt, Omega, self.Omega_d_m1, self.kR, self.kOmega)

        self.q_d_m2 = self.q_d_m1.copy()
        self.q_d_m1 = q_d.copy()
        self.R_d_m1 = R_d.copy()
        self.Omega_d_m1 = Omega_d.copy()

        return {'f': f, 'M': M, 'q': q, 'dq': dq, 'q_d': q_d, 'F': F}

# === Test the controller ===
params = {
    'mQ': 0.665,
    'mL': 0.050,
    'l': 0.3,
    'g': 9.80665,
    'JQ': np.diag([0.005, 0.007, 0.006]),
    'kx': np.diag([8, 8, 6]),
    'kv': np.diag([4, 4, 3]),
    'kI': np.diag([0, 0, 0]),
    'kq': 2,
    'kw': 0.5,
    'kR': 10,
    'kOmega': 0.5
}

controller = LoadController(params)

state = {
    'xQ': np.array([0.0, 0.0, 0.3]),
    'vQ': np.zeros(3),
    'R': np.eye(3),
    'Omega': np.zeros(3),
    'xL': np.array([0.0, 0.0, 0.0]),
    'vL': np.zeros(3),
    'w': np.zeros(3)
}

ref = {
    'x': np.array([1.0, 0.0, 0.0]),
    'xd': np.zeros(3),
    'xdd': np.zeros(3),
    'b1': np.array([1.0, 0.0, 0.0])
}

dt = 0.001
output = controller.update(dt, state, ref)

# Print results
print("Thrust f:", output['f'])
print("Moment M:", output['M'])
print("Cable direction q:", output['q'])
print("Desired cable direction q_d:", output['q_d'])
print("Force vector F:", output['F'])