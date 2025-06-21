"""
Physical parameters for a quadrotor. Values parameterize the inertia, motor dynamics, 
rotor aerodynamics, parasitic drag, and rotor placement. 
Additional sources:
    https://bitcraze.io/2015/02/measuring-propeller-rpm-part-3
    https://wiki.bitcraze.io/misc:investigations:thrust
    https://commons.erau.edu/cgi/viewcontent.cgi?article=2057&context=publication
Notes:
    Parameters of quadrotor in SITL are found in: https://discuss.ardupilot.org/t/customizing-sitl-multicopter-simulations-tailoring-realism-to-your-drone/105549
    /ardupilot/libraries/SITL/SIM_Frame.h
    /ardupilot/libraries/SITL/SIM_Motor.cpp
"""
import numpy as np

quad_params = {

    # Inertial properties
    'mass': 0.904,          # kg [+]
    'Ixx':  0.002354405654,     # kg*m^2 [+]
    'Iyy':  0.002629495468,     # kg*m^2 [+]
    'Izz':  0.003186782158,     # kg*m^2
    'Ixy':  0.0,        # kg*m^2
    'Iyz':  0.0,        # kg*m^2 
    'Ixz':  0.0,        # kg*m^2

    # Geometric properties, all vectors are relative to the center of mass.
    'num_rotors': 4,                        # for looping over each actuator
    'rotor_pos': {  
                    'r1': np.array([ 0.175/2, 0.131/2, 0]),    # Location of Rotor 1, meters [+]
                    'r2': np.array([-0.175/2,-0.131/2, 0]),    # Location of Rotor 2, meters [+]
                    'r3': np.array([-0.175/2, 0.131/2, 0]),    # Location of Rotor 3, meters [+]
                    'r4': np.array([ 0.175/2,-0.131/2, 0]),    # Location of Rotor 4, meters [+]
                 },

    'rotor_directions': np.array([1,1,-1,-1]),  # This dictates the direction of the torque for each motor. correct one.

    'rI': np.array([0,0,0]), # location of the IMU sensor, meters (NOT CHANGED)

    # Frame aerodynamic properties
    'c_Dx': 0.5e-2,  # parasitic drag in body x axis, N/(m/s)**2 (NOT CHANGED)
    'c_Dy': 0.5e-2,  # parasitic drag in body y axis, N/(m/s)**2 (NOT CHANGED)
    'c_Dz': 1e-2,  # parasitic drag in body z axis, N/(m/s)**2 (NOT CHANGED)

    # Rotor properties
    # See "System Identification of the Crazyflie 2.0 Nano Quadrocopter", Forster 2015.
    # k = k_m/k_eta = 1 / 40.35783336 = 0.024778337109423062 Nm/(rad/s)**2, according to "F60 and SITL: Motor thrust and torque"
    # keta = 0.0000007949116377, according to "F60 and SITL: Motor thrust and torque"
    # https://docs.google.com/spreadsheets/d/1iyoT1PP5nc-4BKYd8rkdHfsSkwKesJTSTqj36eKYE-k/edit#gid=0
    # the official data of T-Motor F60 pro V (KV=2550) gives larger keta
    # keta = 0.000001718368244
    # (kv) keta = 0.000001337986558 (MORE RELIABLE)
    # (average) keta = (0.0000007949116377 + 0.000001337986558) / 2 = 1.06644909785e-06
    'k_eta': 0.0000007949116377,           # thrust coefficient N/(rad/s)**2 (OUR TESTED DATA)
    'k_m':   1.969658853113417e-08,        # yaw moment coefficient Nm/(rad/s)**2 (OUR TESTED DATA)
    # 'k_eta': 1.06644909785e-06,           # thrust coefficient N/(rad/s)**2 (AVERAGE OF OFFICIAL AND OURS, WITH KV)
    # 'k_m':   2.6424835256567402e-08,        # yaw moment coefficient Nm/(rad/s)**2 (AVERAGE OF OFFICIAL AND OURS, WITH KV)
    'k_d':   10.2506e-07,       # rotor drag coefficient N/(rad*m/s**2) = kg/rad (NOT CHANGED)
    'k_z':   7.553e-07,         # induced inflow coefficient N/(rad*m/s**2) = kg/rad (NOT CHANGED)
    'k_flap': 0.0,              # Flapping moment coefficient Nm/(rad*m/s**2) = kg*m/rad (NOT CHANGED)

    # Motor properties
    'tau_m': 0.005,             # motor response time, seconds (NOT CHANGED)
    'rotor_speed_min': 0,       # rad/s (NOT CHANGED)
    'rotor_speed_max': 2760,    # rad/s (NOT CHANGED)
    # 'rotor_speed_max': 3100,    # rad/s (experimentally determined)
    'motor_noise_std': 100,     # rad/s (NOT CHANGED)

    # added by Sheng for reduced geometric controller
    # geometric controller parameters
    'kpx': np.array([7.0,6.0,8.0]), # P gains for translational control
    'kdx': np.array([0.6,0.6,2.5]), # D gains for translational control
    # 'kp_att': np.array([1,1,6]), # P gains for rotational control
    'kp_att': np.array([3.0,3.0,0.5]), # P gains for rotational control
    'kd_att': np.array([0.025,0.01,0.1]), # D gains for rotational control

    # sampling time
    'dt': 0.01, # 100 Hz

    # maximum thrust (used for geomtric control only)
    # 'thrust_max': 19.69, # quad_params['mass']*9.8/0.45, where 0.45 is the ratio of throttle needed to get the SITL drone to hover in QGC
    'thrust_max': 30.51, # experimentally determined, corresponding with thrust ratio of 0.29 on George
}
