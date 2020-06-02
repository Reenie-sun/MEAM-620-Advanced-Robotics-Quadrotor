import numpy as np
from scipy.spatial.transform import Rotation

class SE3Control(object):
    """

    """
    def __init__(self, quad_params):
        """
        This is the constructor for the SE3Control object. You may instead
        initialize any parameters, control gain values, or private state here.

        For grading purposes the controller is always initialized with one input
        argument: the quadrotor's physical parameters. If you add any additional
        input arguments for testing purposes, you must provide good default
        values!

        Parameters:
            quad_params, dict with keys specified by crazyflie_params.py

        """

        # Quadrotor physical parameters.
        self.mass            = quad_params['mass'] # kg
        self.Ixx             = quad_params['Ixx']  # kg*m^2
        self.Iyy             = quad_params['Iyy']  # kg*m^2
        self.Izz             = quad_params['Izz']  # kg*m^2
        self.arm_length      = quad_params['arm_length'] # meters
        self.rotor_speed_min = quad_params['rotor_speed_min'] # rad/s
        self.rotor_speed_max = quad_params['rotor_speed_max'] # rad/s
        self.k_thrust        = quad_params['k_thrust'] # N/(rad/s)**2
        self.k_drag          = quad_params['k_drag']   # Nm/(rad/s)**2

        # You may define any additional constants you like including control gains.
        self.inertia = np.diag(np.array([self.Ixx, self.Iyy, self.Izz])) # kg*m^2
        self.g = 9.81 # m/s^2

        # STUDENT CODE HERE

    def update(self, t, state, flat_output):
        """
        This function receives the current time, true state, and desired flat
        outputs. It returns the command inputs.

        Inputs:
            t, present time in seconds
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
                cmd_motor_speeds, rad/s
                cmd_thrust, N (for debugging and laboratory; not used by simulator)
                cmd_moment, N*m (for debugging; not used by simulator)
                cmd_q, quaternion [i,j,k,w] (for laboratory; not used by simulator)
        """
        cmd_motor_speeds = np.zeros((4,))           # contains four speeds of each propeller
        cmd_thrust = 0
        cmd_moment = np.zeros((3,))
        cmd_q = np.zeros((4,))

        # STUDENT CODE HERE
        Kd = np.diag([6, 6, 5.5])
        Kp = np.diag([10, 10, 9])
        KR = np.diag([500, 500, 2.5])
        Kw = np.diag([60, 60, 2.7])
        r_ddot_des = list(flat_output['x_ddot'] - np.dot(Kd, (state['v'] - flat_output['x_dot'])) - np.dot(Kp, (state['x'] - flat_output['x'])))

        # F_des 3*3     it should be 3*1
        F_des =  np.array([[self.mass * r_ddot_des[0]], [self.mass * r_ddot_des[1]], [self.mass * r_ddot_des[2]]]) + [[0], [0], [self.mass * self.g]]
        #print(F_des)

        # then calculate the u1, first the R
        rotation = Rotation.from_quat(state['q'])
        R_matrix = rotation.as_matrix()
        # u1 1*1
        u1 = np.dot(np.transpose(np.dot(R_matrix,[[0], [0], [1]])), F_des)

        # then is the u2 part
        b_3_des = F_des / np.linalg.norm(F_des)
        psai_T = flat_output['yaw']         # yaw is a single number
        a_psai = np.array([[np.cos(psai_T)], [np.sin(psai_T)], [0]])    # 3*1 vector
        b_2_des = np.transpose(np.cross(np.transpose(b_3_des), np.transpose(a_psai)) / np.linalg.norm(np.cross(np.transpose(b_3_des), np.transpose(a_psai))))
        R_des = np.ones([3,3])
        R_des[:, 0] = np.cross(np.transpose(b_2_des), np.transpose(b_3_des))
        R_des[0:3, [1]] = b_2_des
        R_des[:, [2]] = b_3_des
        #R_des[:,0] = np.cross(np.transpose(b_2_des), np.transpose(b_3_des)
        #R_des = np.concatenate((np.cross(b_2_des, b_3_des), b_2_des, b_3_des), axis= 1)

        e_R_skew = (np.dot(np.transpose(R_des), R_matrix) - np.dot(np.transpose(R_matrix), R_des))/2
        # then map the e_R into R3 from a skew matrix
        e_R = np.array([[e_R_skew[2,1]], [e_R_skew[0,2]], [e_R_skew[1, 0]]])
        #print(e_R.shape)

        # then calculate the u2
        temp = list(state['w'])
        #print(temp)
        angular_velo = np.ones([3, 1])
        angular_velo[0] = temp[0]
        angular_velo[1] = temp[1]
        angular_velo[2] = temp[2]
        #print(angular_velo.shape)
        #u2 = np.dot(self.inertia, (-np.dot(KR, e_R) - np.dot(Kw, np.array(state['w']))))
        u2 = np.dot(self.inertia, (-np.dot(KR, e_R) - np.dot(Kw, angular_velo)))

        # finally calculate the four F
        l = self.arm_length
        gamma = self.k_drag / self.k_thrust
        coeff_matrix = np.array([[1, 1, 1, 1], [0, l, 0, -l], [-l, 0, l, 0], [gamma, -gamma, gamma, -gamma]])
        #print(u2.shape)     #u2 should be 3*1
        #u_combined = [[u1], [u2]]

        u_combined = np.ones([4,1])
        u_combined[0] = u1
        u_combined[1:, :] = u2
        Force = np.dot(np.linalg.inv(coeff_matrix), u_combined)     # 4*1  matrix
        print(Force)
        #w_propeller = np.sqrt(Force / self.k_thrust)
        #print(w_propeller)
        # get the propeller speed and set the upper & lower limit
        motor_speed = np.zeros(4)
        for i in range(4):
            motor_speed[i] = np.sqrt(np.abs(Force[i]) / self.k_thrust)
            if motor_speed[i] > self.rotor_speed_max:
                motor_speed[i] = self.rotor_speed_max
            elif motor_speed[i] < self.rotor_speed_min:
                motor_speed[i] = self.rotor_speed_min
        cmd_motor_speeds[0] = motor_speed[0]
        cmd_motor_speeds[1] = motor_speed[1]
        cmd_motor_speeds[2] = motor_speed[2]
        cmd_motor_speeds[3] = motor_speed[3]
        print(cmd_motor_speeds)
        #print(self.k_thrust)

        cmd_thrust = u1
        cmd_moment = u2
        r = Rotation.from_matrix(R_des)
        cmd_q = r.as_quat()

        control_input = {'cmd_motor_speeds':cmd_motor_speeds,
                         'cmd_thrust':cmd_thrust,
                         'cmd_moment':cmd_moment,
                         'cmd_q':cmd_q}
        return control_input
