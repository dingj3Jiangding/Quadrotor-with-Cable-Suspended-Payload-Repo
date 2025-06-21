import numpy as np
from scipy.spatial.transform import Rotation
from george_params import quad_params

# for logging controller params
import os
import csv
from datetime import datetime

class QuadGeoCtrl:
    """
        implementing the geometric control with output to total thrust and angular velocity (command)
    """
    def __init__(self, quad_params, logging = True, L1thrust = False, offset = [0,0,0]):
        """
        Parameters:
            quad_params, dictionary, keys specified in rotorpy/vehicles
            logging, binary,  determining whether to log controller data
            L1thrust: binary, determining whether to turn on L1 on the thrust channel
            offset: 3D vector, specifying the offset of the CoG for tracking purpose
        """

        # Quadrotor physical parameters.
        # only mass is needed
        self.mass            = quad_params['mass'] # kg

        self.g = 9.81 # m/s^2

        # gains for translational control
        self.k = {
            'x': quad_params['kpx'].reshape(3,1),
            'v': quad_params['kdx'].reshape(3,1),
        }
        # gains for rotational control
        self.kp_att = quad_params['kp_att']
        self.kd_att = quad_params['kd_att']

        # clipping the maximum angular velocity command
        self.angVelClipping = 3

        # save the offset
        self.offset = np.array(offset).reshape((3,1)) # turn the list to nparray

        # thrust channel L1
        self.L1thrust = L1thrust
        if L1thrust:
            # initialize previous step v measurement and v prediction from previous step
            self.v_prev = np.array([0,0,0])
            self.v_hat_prev = np.array([0,0,0])

            # initialize prev R
            self.R_prev = np.eye(3) 

            # save prev thrust command
            self.f_cmd_prev = 0
            self.f_ad_prev = 0

            # gain matrix of predictor (treated as a scalar)
            self.As_v = -5.0 
            # exponential coefficients for As
            self.exp_As_v_dt = np.exp(self.As_v * quad_params['dt'])

            # mass inverse
            self.massInverse = 1.0 / quad_params['mass']

            # initialize prev sigma
            self.sigma_m_hat_prev = 0.0
            self.sigma_um_hat_prev = np.zeros((2,))

            # lpf
            cutoffFrequency = 30
            self.lpf_coefficientThrust1 = np.exp(-cutoffFrequency * quad_params['dt']) 
            self.lpf_coefficientThrust2 = 1.0 - self.lpf_coefficientThrust1

            # prev lpf state
            self.lpf_prev = 0

        
        # for logging and debugging
        self.logging = logging
        if logging:
            # # Get current date and time
            # current_time = datetime.now()
            # # Format the date and time as a string
            # formatted_time = current_time.strftime("%Y-%m-%d_%H-%M-%S")
            # # Define the file name with the formatted date and time
            # file_name = f"myfile_{formatted_time}.csv"

            # Get the current date
            current_date = datetime.now().strftime("%Y-%m-%d")

            # Directory where the files are to be saved
            directory = os.getcwd()

            # List all files that start with "data-{current_date}"
            existing_files = [f for f in os.listdir(directory) if f.startswith(f"data-{current_date}") and f.endswith(".csv")]

            # Count the files that match the pattern
            file_number = len(existing_files) + 1  # +1 because the count starts from 1

            # Define the file name with the formatted date and the count
            if L1thrust:
                file_name = f"data-L1-{current_date}-{file_number}.csv" # use a different indicator in the file name
            else:
                file_name = f"data-{current_date}-{file_number}.csv"
            self.file_name = file_name # to display it when closing

            self.csv_file =  file_name
            with open(self.csv_file, mode='w', newline='') as file:
                writer = csv.writer(file)
                if L1thrust:
                    header = ['t','x_1', 'x_2', 'x_3', 
                            'x_des_1', 'x_des_2', 'x_des_3', 
                            'R_11', 'R_12', 'R_13', 'R_21', 'R_22', 'R_23', 'R_31', 'R_32', 'R_33', 
                            'R_des_11', 'R_des_12', 'R_des_13', 'R_des_21', 'R_des_22', 'R_des_23', 'R_des_31', 'R_des_32', 'R_des_33',
                            'v_1','v_2','v_3',
                            'v_des_1','v_des_2','v_des_3',
                            'w_1','w_2','w_3',
                            'w_des_1','w_des_2','w_des_3',
                            'cmd_f',
                            'cmd_w_1','cmd_w_2','cmd_w_3',
                            'v1','v2','v3', 'v_hat1','v_hat2','v_hat3', 'sigma_m_hat', 'f_ad'] # last row is dedicated to L1
                else:
                    # for general purpose logging
                    header = ['t','x_1', 'x_2', 'x_3', 
                            'x_des_1', 'x_des_2', 'x_des_3', 
                            'R_11', 'R_12', 'R_13', 'R_21', 'R_22', 'R_23', 'R_31', 'R_32', 'R_33', 
                            'R_des_11', 'R_des_12', 'R_des_13', 'R_des_21', 'R_des_22', 'R_des_23', 'R_des_31', 'R_des_32', 'R_des_33',
                            'v_1','v_2','v_3',
                            'v_des_1','v_des_2','v_des_3',
                            'w_1','w_2','w_3',
                            'w_des_1','w_des_2','w_des_3',
                            'cmd_f',
                            'cmd_w_1','cmd_w_2','cmd_w_3']
                writer.writerow(header)

    def update(self, time, state, flat_output):
        """
        This function receives the current time, true state, and desired flat
        outputs. It returns the command inputs.
        NOTE: the computation within this function are wrt to ENU

        Inputs:
            state, a dict describing the present state with keys
                x, position, m
                v, linear velocity, m/s
                q, quaternion [i,j,k,w]
                w, angular velocity, rad/s
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s

        Outputs:
            control_input, a dict describing the present computed control inputs with keys
                cmd_motor_speeds, rad/s [not used]
                cmd_motor_thrusts, N [not used]
                cmd_thrust, N 
                cmd_moment, N*m [not used]
                cmd_q, quaternion [i,j,k,w] [not used]
                cmd_w, angular rates in the body frame, rad/s
                cmd_v, velocity in the world frame, m/s [not used]
        """
        cmd_thrust = 0
        # here set it to be zero if not provided
        
        def wedge(x):
            """Return wedged vector."""
            wedge_x = np.array([[0,-x[2][0], x[1][0]], [x[2][0], 0, -x[0][0]], [-x[1][0], x[0][0], 0]])
            return wedge_x
        
        def deriv_unit_vector(q, q_dot, q_ddot):
            """derivative of a unit vector"""
            nq = np.linalg.norm(q)
            u = q / nq
            u_dot = q_dot / nq - q * np.dot(np.ravel(q), np.ravel(q_dot)) / nq**3
            u_ddot = q_ddot / nq - q_dot / (nq**3) * (2 * np.dot(np.ravel(q), np.ravel(q_dot))) \
            - q / nq**3 * (np.dot(np.ravel(q_dot), np.ravel(q_dot)) + np.dot(np.ravel(q), np.ravel(q_ddot))) \
            + 3 * q / nq**5 * np.dot(np.ravel(q), np.ravel(q_dot))**2
            return u, u_dot, u_ddot

        def vee(S):
            """Return vector corresponding to given skew symmetric matrix."""
            s = np.array([[-S[1,2]], [S[0,2]], [-S[0,1]]])
            return s

        def geometric_controller(self, time, state, flat_output):
            # split the state
            state = np.stack(state)
            x = state[0:3].reshape(3,1)
            v = state[3:6].reshape(3,1)
            # R = Rotation.from_quat(state[6:10],scalar_first=True).as_matrix() # important: scipy rotation has quaterion's last element as the scalar term by default
            R = Rotation.from_quat(np.concatenate([state[7:10],[state[6]]])).as_matrix() # complying with scipy 1.10.1 syntax of from_quat
            W = state[10:13].reshape(3,1)
            
            # converting flat_output to desired
            desired = dict()
            desired.update({'x': flat_output['x'].reshape(3,1)})
            desired.update({'v': flat_output['x_dot'].reshape(3,1)})
            desired.update({'x_2dot': flat_output['x_ddot'].reshape(3,1)})
            desired.update({'x_3dot': flat_output['x_dddot'].reshape(3,1)})
            desired.update({'x_4dot': flat_output['x_ddddot'].reshape(3,1)})

            desired.update({'yaw': flat_output['yaw']})
            b1 = np.array([np.cos(flat_output['yaw']), np.sin(flat_output['yaw']), 0])
            b1_dot = np.array([flat_output['yaw_dot'] * -np.sin(flat_output['yaw']), 
                               flat_output['yaw_dot'] * np.cos(flat_output['yaw']), 
                               0])
            b1_2dot = np.array([flat_output['yaw_ddot'] * -np.sin(flat_output['yaw']) + flat_output['yaw_dot']**2 * -np.cos(flat_output['yaw']), 
                               flat_output['yaw_ddot'] * np.cos(flat_output['yaw']) + flat_output['yaw_dot']**2 * -np.sin(flat_output['yaw']), 
                               0])
            
            desired.update({'b1': b1.reshape(3,1)})
            desired.update({'b1_dot': b1_dot.reshape(3,1)})
            desired.update({'b1_2dot': b1_2dot.reshape(3,1)})

            # consider the offset
            # (if offset = [0,0,0], then this is the original geometric control / L1)
            x = x + R @ self.offset # shift the position from center to center+offset (offset presented in body frame)
            R_dot = R @ wedge(W)
            v = v + R_dot @ self.offset # shift the velocity from center to center+offset

            # # update variable controller gains here.
            # if 0 <= time < 2:
            #     self.k = {
            #         'x': np.array([5.6885, 6.1519, 4.0334]).reshape(3,1),
            #         'v': np.array([1.1041, 2.2419, 1.5606]).reshape(3,1),
            #         }
            #     self.kp_att = np.array([4.3377, 3.5629, 3.9994])
            #     self.kd_att = np.array([0.0100, 0.0100, 1.0013])
            # elif 2 <= time < 4:
            #     self.k = {
            #         'x': np.array([5.2820, 6.0225, 4.0282]).reshape(3,1),
            #         'v': np.array([0.4680, 2.0458, 1.5421]).reshape(3,1),
            #     }
            #     self.kp_att = np.array([4.2446, 2.3689, 3.9998])
            #     self.kd_att = np.array([0.0100, 0.0100, 0.9995])
            # elif 4 <= time < 6:
            #     self.k = {
            #         'x': np.array([5.4495, 6.3337, 4.0052]).reshape(3,1),
            #         'v': np.array([0.1000, 1.6477, 1.5223]).reshape(3,1),
            #     }
            #     self.kp_att = np.array([4.2387, 2.3436, 4.0004])
            #     self.kd_att = np.array([0.0100, 0.0100, 0.9993])
            # elif 6 <= time < 8:
            #     self.k = {
            #         'x': np.array([4.9880, 5.7870, 4.0197]).reshape(3,1),
            #         'v': np.array([1.6317, 0.5124, 1.5383]).reshape(3,1),
            #     }
            #     self.kp_att = np.array([3.3981, 2.6930, 3.9998])
            #     self.kd_att = np.array([0.0100, 0.0100, 1.0002])
            # elif 8 <= time <= 10:
            #     self.k = {
            #         'x': np.array([5.4781, 6.5214, 4.7704]).reshape(3,1),
            #         'v': np.array([3.5612, 2.2581, 4.3480]).reshape(3,1),
            #     }
            #     self.kp_att = np.array([4.5736, 9.0989, 3.9129])
            #     self.kd_att = np.array([1.3608, 1.8628, 0.9874])

            # translational control
            cmd_f, R_des, w_des, _, _ = position_control(time, x, v, R, W, desired, self.k, self.mass)

            # rotational control
            S_err = 0.5 * (R_des.T @ R - R.T @ R_des) 
            att_err = vee(S_err)
            # Angular velocity error
            w_err = W - R.T @ R_des @ w_des 

            cmd_w = -self.kp_att*att_err.reshape(3,) - self.kd_att*w_err.reshape(3,) +  w_des.reshape(3,)

            # cliiping the angular velocity command
            cmd_w = np.clip(cmd_w, -self.angVelClipping, self.angVelClipping)

            # print(f"ang. vel. cmd = {cmd_w}")
            # print(f"thrust cmd= {cmd_f}")

            if self.logging:
                row_data = np.concatenate((np.array([time]), # time
                                           state[0:3],       # position
                                           flat_output['x'], # desired position
                                           R.flatten(),      # rotation matrix
                                           R_des.flatten(),  # desired rotation matrix
                                           state[3:6],      # velocity
                                           flat_output['x_dot'], # desired velocity   
                                           state[10:13],     # angluar velocity
                                           w_des.reshape(3,), # desired angular velocity
                                           cmd_f,            # thrust command
                                           cmd_w             # angular velocity command
                ))            
            return cmd_f, cmd_w, row_data


        # position control
        def position_control(time, x, v, R, W, desired, k, m):
            e3 = np.array([[0],[0],[1]])
            # e3 = np.array([0,0,1])
            g = 9.8

            error_x = x - desired['x']
            error_v = v - desired['v']
            error = {
                'x': error_x,
                'v': error_v,
                'W': 0,
                'R': 0
            }
            # ENU
            # A = -k['x'] * error['x'] - k['v'] * error['v'] - m * g * e3 + m * desired['x_2dot']
            A = -k['x'] * error['x'] - k['v'] * error['v'] + m * g * e3 + m * desired['x_2dot']
            
            b3 = R @ e3
            # ENU
            # f = -np.dot(np.ravel(A), b3)
            f = np.dot(np.ravel(A), b3)
            
            # ENU
            # ev_dot = g * e3 - f / m * b3 - desired['x_2dot']
            ev_dot = -g * e3 + f / m * b3 - desired['x_2dot']
            
            A_dot = -k['x'] * error['v'] - k['v'] * ev_dot + m * desired['x_3dot']

            b3_dot = R @ wedge(W) @ e3
            # ENU
            # f_dot = -np.dot(np.ravel(A_dot), b3) - np.dot(np.ravel(A), b3_dot)
            f_dot = np.dot(np.ravel(A_dot), b3) + np.dot(np.ravel(A), b3_dot)
            # ENU
            # ev_2dot = -f_dot / m * b3 - f / m * b3_dot - desired['x_3dot']
            ev_2dot = f_dot / m * b3 + f / m * b3_dot - desired['x_3dot']
            # ENU
            # A_ddot = - k['x'] * ev_dot - k['v'] * ev_2dot + m * desired['x_4dot']
            A_ddot = - k['x'] * ev_dot - k['v'] * ev_2dot + m * desired['x_4dot']
            
            # ENU
            # b3c, b3c_dot, b3c_ddot = deriv_unit_vector(-A, -A_dot, -A_ddot)
            b3c, b3c_dot, b3c_ddot = deriv_unit_vector(A, A_dot, A_ddot)

            A2 = -wedge(desired['b1']) @ b3c

            A2_dot = -wedge(desired['b1_dot']) @ b3c - wedge(desired['b1']) @ b3c_dot

            A2_ddot = -wedge(desired['b1_2dot']) @ b3c - 2 * wedge(desired['b1_dot']) @ b3c_dot - wedge(desired['b1']) @ b3c_ddot

            b2c, b2c_dot, b2c_ddot = deriv_unit_vector(A2, A2_dot, A2_ddot)

            b1c = wedge(b2c) @ b3c
            b1c_dot = wedge(b2c_dot) @ b3c + wedge(b2c) @ b3c_dot
            b1c_ddot = wedge(b2c_ddot) @ b3c + 2 * wedge(b2c_dot) @ b3c_dot + wedge(b2c) @ b3c_ddot

            Rc = np.hstack((b1c, b2c))
            Rc = np.hstack((Rc, b3c))
            Rc_dot = np.hstack((b1c_dot, b2c_dot))
            Rc_dot = np.hstack((Rc_dot, b3c_dot))

            Rc_ddot = np.hstack((b1c_ddot, b2c_ddot))
            Rc_ddot = np.hstack((Rc_ddot, b3c_ddot))

            Wc = vee(np.transpose(Rc) @ Rc_dot)
            Wc_dot = vee(np.transpose(Rc) @ Rc_ddot - wedge(Wc) @ wedge(Wc))
            
            return f, Rc, Wc, Wc_dot, error
        
        # # attitude control [not used]
        # def attitude_control(R, W, Rd, Wd, Wddot, k, J):
        #     eR = 1 / 2 * vee(np.transpose(Rd) @ R - np.transpose(R) @ Rd)
        #     eW = W - np.transpose(R) @ Rd @ Wd
        #     M = - k['R'] * eR - k['W'] * eW + wedge(W) @ (J @ W) - J @ (wedge(W) @ np.transpose(R) @ Rd @ Wd - np.transpose(R) @ Rd @ Wddot)
        #     return M, eR, eW

        f_cmd, w_cmd, data_for_logging = geometric_controller(self, time, state, flat_output)

        # turn L1 on if true (only to thrust channel)
        if self.L1thrust:
            # inputs needed: state, f_cmd,

            # considering the offset 
            # (if offset = [0,0,0], then this is the original geometric control / L1)
            W = np.array(state[10:13])
            R = Rotation.from_quat(state[6:10],scalar_first=True).as_matrix() # important: scipy rotation has quaterion's last element as the scalar term by default
            R_dot = R @ wedge(W.reshape(3,1))
            v = np.array(state[3:6]) + R_dot @ self.offset.reshape(3,)
            
            # update velocity error
            vpred_error_prev = self.v_hat_prev - self.v_prev

            # state predictor
            v_hat = self.v_hat_prev + (-np.array([0,0,1]) * 9.8 + self.R_prev[:, 2] * (self.f_cmd_prev + self.f_ad_prev + self.sigma_m_hat_prev) * self.massInverse + (self.R_prev[:,0:2] @ self.sigma_um_hat_prev.reshape((2, 1)) * self.massInverse).reshape((3,)) + self.As_v * vpred_error_prev) * quad_params['dt'] 

            # update the state prediction storage
            self.v_hat_prev = v_hat

            # adaptation law
            vpred_error = v_hat - v
            PhiInvmu_v = vpred_error / (self.exp_As_v_dt - 1) * self.As_v * self.exp_As_v_dt

            sigma_m_hat = -R[:,2].reshape((1,3)) @ PhiInvmu_v.reshape((3,1)) * quad_params['mass']
            sigma_um_hat = -R[:,0:2].T @ PhiInvmu_v.reshape((3,1)) * quad_params['mass']
            
            # store the sigma_hat's
            self.sigma_m_hat_prev = sigma_m_hat[0,0]
            self.sigma_um_hat_prev = sigma_um_hat.reshape((2,))

            # lpf
            lpf_out = self.lpf_coefficientThrust1 * self.lpf_prev + self.lpf_coefficientThrust2 * sigma_m_hat[0,0]
            
            # store the lpf-ed thrust
            self.lpf_prev = lpf_out

            # negate the lpf result to get L1 thrust
            f_ad = -lpf_out

            # store the values
            self.v_prev = v
            self.R_prev = R
            self.f_cmd_prev = f_cmd
            self.f_ad_prev = f_ad

            print(f"f geo = {f_cmd}")
            print(f"f L1 = {f_ad}")
        
            # generate the final thrust command
            f_cmd = f_cmd + f_ad

        # logging 
        if self.logging:
            if self.L1thrust:
                # L1 has its own logging format
                row_data = np.concatenate((data_for_logging, v, v_hat, np.array(sigma_m_hat[0,0]).reshape(1,), f_ad.reshape(1,) )) # row vectors of the values to be logged
                
            else:
                # logging for general purpose
                row_data = data_for_logging # row vectors of the values to be logged
                
            # write to the file
            with open(self.csv_file, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(row_data)

            
        # reduce it to minimum level of complexity
        control_input = { #'cmd_motor_speeds':cmd_motor_speeds,
                        #  'cmd_motor_thrusts':cmd_rotor_thrusts,
                         'cmd_thrust':f_cmd,
                        #  'cmd_moment':cmd_moment,
                        #  'cmd_q':cmd_q,
                         'cmd_w':w_cmd,
                        #  'cmd_v':cmd_v
                        }
        return control_input

    def __del__(self):
        # Close the file when the object is deleted or at the end of the program
        self.csv_file.close()
        # Your file_name is now formatted as "data-YYYY-MM-DD-x.csv"
        print("The file will be saved as:", self.file_name)
        

    
