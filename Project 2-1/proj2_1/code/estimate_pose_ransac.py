# Imports

import numpy as np
from scipy.spatial.transform import Rotation


# %%

def estimate_pose(uvd1, uvd2, pose_iterations, ransac_iterations, ransac_threshold):
    """
    Estimate Pose by repeatedly calling ransac

    :param uvd1:
    :param uvd2:
    :param pose_iterations:
    :param ransac_iterations:
    :param ransac_threshold:
    :return: Rotation, R; Translation, T; inliers, array of n booleans
    """

    R = Rotation.identity()

    for i in range(0, pose_iterations):
        w, t, inliers = ransac_pose(uvd1, uvd2, R, ransac_iterations, ransac_threshold)
        R = Rotation.from_rotvec(w.ravel()) * R

    return R, t, inliers


def ransac_pose(uvd1, uvd2, R, ransac_iterations, ransac_threshold):
    # find total number of correspondences
    n = uvd1.shape[1]

    # initialize inliers all false
    best_inliers = np.zeros(n, dtype=bool)

    for i in range(0, ransac_iterations):
        # Select 3  correspondences
        selection = np.random.choice(n, 3, replace=False)

        # Solve for w and  t
        w, t = solve_w_t(uvd1[:, selection], uvd2[:, selection], R)

        # find inliers
        inliers = find_inliers(w, t, uvd1, uvd2, R, ransac_threshold)

        # Update best inliers
        if inliers.sum() > best_inliers.sum():
            best_inliers = inliers.copy()

    # Solve for w and t using best inliers
    w, t = solve_w_t(uvd1[:, best_inliers], uvd2[:, best_inliers], R)

    return w, t, find_inliers(w, t, uvd1, uvd2, R, ransac_threshold)



def find_inliers(w, t, uvd1, uvd2, R0, threshold):
    """

    find_inliers core routine used to detect which correspondences are inliers

    :param w: ndarray with 3 entries angular velocity vector in radians/sec
    :param t: ndarray with 3 entries, translation vector
    :param uvd1: 3xn ndarray : normailzed stereo results from frame 1
    :param uvd2:  3xn ndarray : normailzed stereo results from frame 2
    :param R0: Rotation type - base rotation estimate
    :param threshold: Threshold to use
    :return: ndarray with n boolean entries : Only True for correspondences that pass the test
    """
    n = uvd1.shape[1]
    inliers = np.zeros(n, dtype='bool')
    # TODO Your code here replace the dummy return value with a value you compute
    Rot0 = Rotation.as_matrix(R0)
    for i in range(n):
        temp_uvd1 = uvd1[:, i]
        temp_uvd2 = uvd2[:, i]
        mat1 = [[1, 0, -temp_uvd1[0]], [0, 1, -temp_uvd1[1]]]

        [w1, w2, w3] = w
        # w_skew = [[0, -w3, w2], [w3, 0, -w1], [-w2, w1, 0]]
        # inter_mat = np.dot(np.eye(3)+w_skew, Rot0)
        w_skew_I = [[1, -w3, w2], [w3, 1, -w1], [-w2, w1, 1]]
        inter_mat = np.dot(w_skew_I, Rot0)

        mat2 = np.dot(inter_mat, [temp_uvd2[0], temp_uvd2[1], 1])
        mat3 = np.dot(temp_uvd2[2], t)
        sigma = np.dot(mat1, (mat2+mat3))
        print(sigma)
        sigma_norm = np.linalg.norm(sigma)

        if sigma_norm < threshold:
            inliers[i] = 1
        else:
            inliers[i] = 0
    return inliers

def solve_w_t(uvd1, uvd2, R0):
    """
    solve_w_t core routine used to compute best fit w and t given a set of stereo correspondences

    :param uvd1: 3xn ndarray : normailzed stereo results from frame 1
    :param uvd2: 3xn ndarray : normailzed stereo results from frame 1
    :param R0: Rotation type - base rotation estimate   Rotation object
    :return: w, t : 3x1 ndarray estimate for rotation vector, 3x1 ndarray estimate for translation
    """

    # TODO Your code here replace the dummy return value with a value you compute

    n = uvd1.shape[1]                       # number of eqns always 3
    print('n = ', str(n))
    # print('current n number:', str(n))
    for i in range(n):
        # Rot0 = R0.as_matrix()
        Rot0 = Rotation.as_matrix(R0)
        temp_uvd1 = uvd1[:, i]
        temp_uvd2 = uvd2[:, i]
        temp_y = np.dot(Rot0, [[temp_uvd2[0]], [temp_uvd2[1]], [1]])
        [y1, y2, y3] = temp_y
        temp_b = -np.dot([[1, 0, -temp_uvd1[0]], [0, 1, -temp_uvd1[1]]], temp_y)#.astype(float)

        temp_mat1 = [[1, 0, -temp_uvd1[0]], [0, 1, -temp_uvd1[1]]]
        temp_mat2 = [[0, y3, -y2, temp_uvd2[2], 0, 0], [-y3, 0, y1, 0, temp_uvd2[2], 0], [y2, -y1, 0, 0, 0, temp_uvd2[2]]]
        temp_A = np.dot(temp_mat1, temp_mat2)
        if i == 0:
            A = temp_A
            b = temp_b
        else:
            A = np.vstack((A, temp_A))
            b = np.vstack((b, temp_b))

    # print(A)
    x = np.linalg.lstsq(A, b)[0]    # only get the solution 6*1
    # x = np.dot(np.linalg.inv(A), b)
    w = np.asarray(x[0:3]).reshape((3,1))
    t = np.asarray(x[3:6]).reshape((3,1))
    print('THIS IS THE LSQ ESTIMATE', str(w))
    return w, t

# def solve_w_t(uvd1, uvd2, R0):
#     """
#     solve_w_t core routine used to compute best fit w and t given a set of stereo correspondences
#
#     :param uvd1: 3xn ndarray : normailzed stereo results from frame 1
#     :param uvd2: 3xn ndarray : normailzed stereo results from frame 1
#     :param R0: Rotation type - base rotation estimate
#     :return: w, t : 3x1 ndarray estimate for rotation vector, 3x1 ndarray estimate for translation
#     """
#
#     # TODO Your code here replace the dummy return value with a value you compute
#     w = t = np.zeros((3,1))
#
#     A = np.zeros((2 * uvd1.shape[1], 6))
#     b = np.zeros((2, 1))
#     # print(uvd1.shape[1])
#
#     for i in range(uvd1.shape[1]):
#         # print('i:', i)
#         # print(uvd1.shape[1])
#         # print(uvd2.shape[1])
#
#         R0_m = Rotation.as_matrix(R0)
#
#         y = R0_m @ np.array([uvd2[0, i], uvd2[1, i], 1]).reshape(3, 1)
#         btmp = - np.array([[1, 0, - uvd1[0, i]],
#                         [0, 1, - uvd1[1, i]]]) @ y
#         y1, y2, y3 = np.ndarray.flatten(y)
#         A1 = np.array([[1, 0, - uvd1[0, i]],
#                        [0, 1, - uvd1[1, i]]])
#         A2 = np.array([[0, y3, -y2, uvd2[2, i], 0, 0],
#                        [-y3, 0, y1, 0, uvd2[2, i], 0],
#                        [y2, -y1, 0, 0, 0, uvd2[2, i]]])
#         Atmp = A1 @ A2
#
#         if i == 0:
#             A = Atmp
#             b = btmp
#         else:
#             A = np.vstack((A, Atmp))
#             b = np.vstack((b, btmp))
#
#     x, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
#
#     w = x[0:3, :]
#     t = x[3:, :]
#         # print(w)
#         # print(t)
#         # exit()
#
#     return w, t
