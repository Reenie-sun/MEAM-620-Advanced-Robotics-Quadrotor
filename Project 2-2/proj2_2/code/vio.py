#%% Imports

import numpy as np
from numpy.linalg import inv
from numpy.linalg import norm
from scipy.spatial.transform import Rotation


#%% Functions

def nominal_state_update(nominal_state, w_m, a_m, dt):
    """
    function to perform the nominal state update

    :param nominal_state: State tuple (p, v, q, a_b, w_b, g)
                        all elements are 3x1 vectors except for q which is a Rotation object
    :param w_m: 3x1 vector - measured angular velocity in radians per second
    :param a_m: 3x1 vector - measured linear acceleration in meters per second squared
    :param dt: duration of time interval since last update in seconds
    :return: new tuple containing the updated state
    """
    # Unpack nominal_state tuple
    p, v, q, a_b, w_b, g = nominal_state

    # Reference: https://www.ri.cmu.edu/wp-content/uploads/2017/04/eskf.pdf
    # http://www.iri.upc.edu/people/jsola/JoanSola/objectes/notes/kinematics.pdf   / (101)
    R = Rotation.as_matrix(q)   # convert the current rotation object into matrix
    Q = Rotation.as_quat(q)     # into quaternion

    # update p
    new_p = p + dt * v + (1/2) * pow(dt, 2) * (np.dot(R, (a_m - a_b)) + g)

    # update v
    new_v = v + dt* (np.dot(R, (a_m - a_b)) + g)

    # update q
    # as_quat [x, y, z, w]
    w_mb = dt * (w_m - w_b)
    # w_mb_quat = np.vstack((w_mb/norm(w_mb), norm(w_mb)))
    w_mb_Rotation = Rotation.from_rotvec(w_mb.reshape((3,)))              # replace it with the from_rotvec : Rotation object
    w_mb_quat = Rotation.as_quat(w_mb_Rotation)

    Q_x, Q_y, Q_z, Q_w = Q
    temp_x, temp_y, temp_z, temp_w = w_mb_quat
    Q_new = [Q_w*temp_x + Q_x*temp_w + Q_y*temp_z - Q_z*temp_y, \
             Q_w*temp_y - Q_x*temp_z + Q_y*temp_w + Q_z*temp_x, \
             Q_w*temp_z + Q_x*temp_y - Q_y*temp_x + Q_z*temp_w, \
             Q_w*temp_w - Q_x*temp_x - Q_y*temp_y - Q_z*temp_z]      # quaternion product
    new_q = Rotation.from_quat(Q_new)

    # update a, w, g
    a_b = a_b
    w_b = w_b
    g = g

    # delta_p =

    new_nominal_state = (new_p, new_v, new_q, a_b, w_b, g)
    # return new_p, new_v, new_q, a_b, w_b, g
    return new_nominal_state

def get_skew(vec):
    '''
    This function transform the vector into skew-symmetric matrix
    vec: 3*1/ 1*3 np.array
    '''
    x, y, z = vec
    skew_mat = [[0, -z, y], [z, 0, -x], [-y, x, 0]]
    return skew_mat

def error_covariance_update(nominal_state, error_state_covariance, w_m, a_m, dt,
                            accelerometer_noise_density, gyroscope_noise_density,
                            accelerometer_random_walk, gyroscope_random_walk):
    """
    Function to update the error state covariance matrix

    :param nominal_state: State tuple (p, v, q, a_b, w_b, g)
                        all elements are 3x1 vectors except for q which is a Rotation object
    :param error_state_covariance: 18x18 initial error state covariance matrix
    :param w_m: 3x1 vector - measured angular velocity in radians per second
    :param a_m: 3x1 vector - measured linear acceleration in meters per second squared
    :param dt: duration of time interval since last update in seconds
    :param accelerometer_noise_density: standard deviation of accelerometer noise
    :param gyroscope_noise_density: standard deviation of gyro noise
    :param accelerometer_random_walk: accelerometer random walk rate
    :param gyroscope_random_walk: gyro random walk rate
    :return:
    """

    # Unpack nominal_state tuple
    p, v, q, a_b, w_b, g = nominal_state

    R = Rotation.as_matrix(q)  # convert the current rotation object into matrix
    Q = Rotation.as_quat(q)  # into quaternion

    # compute the matrix elements in Fx
    I_mat = np.eye(3)
    I_dt = dt * np.eye(3)
    zero_mat = np.zeros((3, 3))
    R_dt = dt * R

    a_skew = get_skew(a_m-a_b)      # the skew matrix of am-ab

    w_mbt = dt*(w_m - w_b)      # rotation vector with radius magnitude
    w_mbt_Rotation = Rotation.from_rotvec(w_mbt.reshape((3,)))
    R_wmbt = Rotation.as_matrix(w_mbt_Rotation)

    Fx_1 = np.hstack((I_mat, I_dt, zero_mat, zero_mat, zero_mat, zero_mat))
    Fx_2 = np.hstack((zero_mat, I_mat, -np.dot(R, a_skew)*dt, -R_dt, zero_mat, I_dt))
    Fx_3 = np.hstack((zero_mat, zero_mat, np.transpose(R_wmbt), zero_mat, -I_dt, zero_mat))
    Fx_4 = np.hstack((zero_mat, zero_mat, zero_mat, I_mat, zero_mat, zero_mat))
    Fx_5 = np.hstack((zero_mat, zero_mat, zero_mat, zero_mat, I_mat, zero_mat))
    Fx_6 = np.hstack((zero_mat, zero_mat, zero_mat, zero_mat, zero_mat, I_mat))

    Fx_mat = np.vstack((Fx_1, Fx_2, Fx_3, Fx_4, Fx_5, Fx_6))
    Fi_mat = np.vstack((np.hstack((zero_mat, zero_mat, zero_mat, zero_mat)), \
                    np.hstack((zero_mat, zero_mat, zero_mat, zero_mat)), \
                    np.hstack((zero_mat, zero_mat, zero_mat, zero_mat)), \
                    np.hstack((zero_mat, zero_mat, zero_mat, zero_mat)), \
                    np.hstack((zero_mat, zero_mat, zero_mat, zero_mat)), \
                    np.hstack((zero_mat, zero_mat, zero_mat, zero_mat)) ))

    Vi = pow(accelerometer_noise_density, 2) * pow(dt, 2) * I_mat
    Thi = pow(gyroscope_noise_density, 2) * pow(dt, 2) * I_mat
    Ai = pow(accelerometer_random_walk, 2) * dt * I_mat
    Omegai = pow(gyroscope_random_walk, 2) * dt *I_mat
    Qi_mat = np.vstack((np.hstack((Vi, zero_mat, zero_mat, zero_mat)), \
                        np.hstack((zero_mat, Thi, zero_mat, zero_mat)), \
                        np.hstack((zero_mat, zero_mat, Ai, zero_mat)), \
                        np.hstack((zero_mat, zero_mat, zero_mat, Omegai)) ))
    P = error_state_covariance
    new_P = np.dot(np.dot(Fx_mat, P), np.transpose(Fx_mat)) + np.dot(np.dot(Fi_mat, Qi_mat), np.transpose(Fi_mat))

    return new_P
    # return an 18x18 covariance matrix


def measurement_update_step(nominal_state, error_state_covariance, uv, Pw, error_threshold, Q):
    """
    Function to update the nominal state and the error state covariance matrix based on a single
    observed image measurement uv, which is a projection of Pw.

    :param nominal_state: State tuple (p, v, q, a_b, w_b, g)
                        all elements are 3x1 vectors except for q which is a Rotation object
    :param error_state_covariance: 18x18 initial error state covariance matrix
    :param uv: 2x1 vector of image measurements
    :param Pw: 3x1 vector world coordinate
    :param error_threshold: inlier threshold
    :param Q: 2x2 image covariance matrix
    :return: new_state_tuple, new error state covariance matrix
    """

    # Unpack nominal_state tuple
    p, v, q, a_b, w_b, g = nominal_state

    R = Rotation.as_matrix(q)    # Rotation
    Q_euler = Rotation.as_euler(q, 'xyz')    # (3, )

    Pc = np.dot(R.T, (Pw - p))      # [Xc, Yc, Zc].T
    Xc, Yc, Zc = Pc.flatten()
    innovation = uv - 1/Zc * np.array([[Xc], [Yc]])    # 2*1
    Qt = Q
    Sigmat_prem = error_state_covariance

    # check the outlier
    if np.linalg.norm(innovation) >= error_threshold:  # no updates are performed
        return (p, v, q, a_b, w_b, g), error_state_covariance, innovation
    else:
        # preparation calculation
        Pc0 = np.dot(R.T, (Pw - p))     # p0 is the same thing with p
        zt_Pc = np.array([[1/Zc, 0, -Xc/Zc**2], [0, 1/Zc, -Yc/Zc**2]])
        # Pc_theta = get_skew(Pc0)

        zt_theta = np.dot(zt_Pc, get_skew(Pc0))      # 2*3
        zt_p = np.dot(zt_Pc, -R.T)

        Ht = np.zeros((2, 18))
        Ht[0:2, 0:3] = zt_p
        Ht[0:2, 6:9] = zt_theta
        # Kt = np.dot(np.dot(Sigmat_prem, np.transpose(Ht)), (np.dot(np.dot(Ht, Sigmat_prem), np.transpose(Ht)) + Qt))  # 18*2
        Kt = error_state_covariance @ Ht.T @ np.linalg.inv(np.dot(np.dot(Ht, error_state_covariance), Ht.T) + Q)
        I_18mat = np.identity(18)
        Sigmat_new = np.dot(np.dot((I_18mat - np.dot(Kt, Ht)), Sigmat_prem), np.transpose(I_18mat - np.dot(Kt, Ht))) + np.dot(np.dot(Kt, Qt), np.transpose(Kt))
        delta_x = np.dot(Kt, innovation)    # 18*2 - 2*1 = 18*1   6*3dims vector
        p = p + delta_x[0:3,:].reshape((3, 1))
        v = v + delta_x[3:6,:].reshape((3, 1))
        Q_euler = Q_euler + delta_x[6:9,:].reshape((3, ))
        q_new = Rotation.from_euler('xyz', Q_euler)
        a_b = a_b + delta_x[9:12,:].reshape((3, 1))
        w_b = w_b + delta_x[12:15,:].reshape((3, 1))
        g = g + delta_x[15:18,:].reshape((3, 1))

        new_nominal_state = (p, v, q_new, a_b, w_b, g)
        new_error_state_covariance = Sigmat_new

        return new_nominal_state, new_error_state_covariance, innovation
        # return (p, v, q, a_b, w_b, g), error_state_covariance, innovation
