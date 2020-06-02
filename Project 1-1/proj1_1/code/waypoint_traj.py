import numpy as np
import math

class WaypointTraj(object):
    """

    """
    def __init__(self, points):
        """
        This is the constructor for the Trajectory object. A fresh trajectory
        object will be constructed before each mission. For a waypoint
        trajectory, the input argument is an array of 3D destination
        coordinates. You are free to choose the times of arrival and the path
        taken between the points in any way you like.

        You should initialize parameters and pre-compute values such as
        polynomial coefficients here.

        Inputs:
            points, (N, 3) array of N waypoint coordinates in 3D
        """

        # STUDENT CODE HERE
        self.points = points           # convey the traj points, take the striaght line path
        self.point_num = points.shape[0]        # get N number of points
        self.x_pos = points[:, 0]      # get the x_pos
        self.y_pos = points[:, 1]
        self.z_pos = points[:, 2]
        self.arrival_time = points.shape[0]     # time is 0 ~ N-1, N points in total

        #time_array = range()


    def update(self, t):        # t is not a input
        """
        Given the present time, return the desired flat output and derivatives.

        Inputs
            t, time, s
        Outputs
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s
        """
        x        = np.zeros((3,))
        x_dot    = np.zeros((3,))
        x_ddot   = np.zeros((3,))           # set this to be zero instead of using fitting
        x_dddot  = np.zeros((3,))
        x_ddddot = np.zeros((3,))
        yaw = 0
        yaw_dot = 0

        # STUDENT CODE HERE
        # x is the quadrator position at time t
        # set the quadrator to accelerate and decelerate at a same acceleration

        # =================== Constant acceleration ==========================
        a_const = np.array([4, 4, 4])         # constant acceleration, get from the previous test
        a_const = 4.0
        distance = np.zeros((self.point_num - 1, 3))      # define the array to store the displacements
        direction = np.zeros((self.point_num - 1, 3))
        time_table = np.zeros((self.point_num, 1))
        time_table[0] = 0        # start time = 0
        for i in range(self.point_num-1):
            distance[i, :] = self.points[i+1] - self.points[i]
            direction[i, :] = (self.points[i+1] - self.points[i]) / np.linalg.norm(self.points[i+1] - self.points[i])
            temp_dis = np.linalg.norm(distance[i,:])      # get the vector's norm
            time_table[i+1] =np.sqrt(4.0 * temp_dis/ a_const) + time_table[i]      # fill out the time table for each point
        #print(time_table)
        # locate the t and corresponding position
        if t >= time_table[-1]:     # t >= the last points' time, keep it still at the last point
            x = self.points[-1]
        else:
            for i in range(self.point_num-1):     # search for the t's corresponding section
                # ongoing point is time_table[i], point #i
                if t == time_table[i]:          # right at the currently processing point
                    x = self.points[i,:]
                elif time_table[i] < t < time_table [i + 1]:       # between two points, in section (simplified statement)
                    front_point = self.points[i, :]
                    next_point = self.points[i+1, :]
                    frac_time = t - time_table[i]       # get the time fraction within this section
                    mid_time = (time_table[i+1] - time_table[i]) / 2
                    # now it is between two points
                    if frac_time < mid_time:  # 1. acceleration part
                        # acceleration can be removed
                        #x_ddot = np.dot(a_const, direction[i, :])   # acceleration is along the unit vector direction
                        #print('fractime', frac_time)
                        x_dot = np.dot(float(a_const * frac_time), direction[i,:])
                        x = front_point + np.dot(float(a_const * np.square(frac_time)/2), direction[i,:])
                    elif frac_time == mid_time: # 2. turning point
                        x_dot = np.dot(float(a_const * mid_time), direction[i,:])
                        x = (front_point + next_point) / 2
                    elif frac_time > mid_time:  # 3. deceleration part
                        # acceleration can be removed
                        #x_ddot = np.dot(-1*a_const, direction[i, :])  # acceleration is along the unit vector direction
                        x_dot = np.dot(float(a_const * (2 * mid_time -  frac_time)), direction[i, :])
                        x = next_point - np.dot(float(a_const * np.square(2 * mid_time - frac_time)/2), direction[i, :])
                elif t > time_table[i+1]:   # it is not within this section, continue the next
                    continue

        # =================== Constant speed =================================
        # time = 2* float(t)  # in case the t is not a float type
        # #v_const = np.array([1, 1, 1])
        # if time >= self.point_num - 1:
        #     x = self.points[-1, :]
        # else:
        #     current_section = int(math.floor(time))  # it is the current section, every section spends 1 sec
        #     in_section_time = time - math.floor(time)  # get the fractional part, reveals the location within two points
        #
        #     if in_section_time == 0.0:  # the quadrator is just at one point
        #         x = self.points[current_section, :]
        #     else:
        #         # these are only for the two points of the section
        #         former_point = self.points[current_section, :]
        #         next_point = self.points[current_section + 1, :]
        #         displacement = next_point - former_point  # 1 3 vector
        #         x_dot = displacement
        #         x = np.dot(in_section_time, x_dot) / 2 + former_point  # former coordinate plus the distance covered

        #===================== Constant Speed===========================
        # a_const = 4.0         # constant acceleration, get from the previous test
        # v_const = 1.0
        # distance = np.zeros((self.point_num - 1, 3))      # define the array to store the displacements
        # direction = np.zeros((self.point_num - 1, 3))
        # time_table = np.zeros((self.point_num, 1))
        # time_table[0] = 0        # start time = 0
        # for i in range(self.point_num-1):
        #     distance[i, :] = self.points[i+1] - self.points[i]
        #     direction[i, :] = (self.points[i+1] - self.points[i]) / np.linalg.norm(self.points[i+1] - self.points[i])
        #     temp_dis = np.linalg.norm(distance[i,:])      # get the vector's norm
        #     time_table[i+1] = temp_dis / v_const       # fill out the time table for each point
        # #print(time_table)
        # # locate the t and corresponding position
        # if t >= time_table[-1]:     # t >= the last points' time, keep it still at the last point
        #     x = self.points[-1]
        # else:
        #     for i in range(self.point_num-1):     # search for the t's corresponding section
        #         # ongoing point is time_table[i], point #i
        #         if t == time_table[i]:          # right at the currently processing point
        #             x = self.points[i,:]
        #         elif time_table[i] < t < time_table [i + 1]:       # between two points, in section (simplified statement)
        #             front_point = self.points[i, :]
        #             next_point = self.points[i+1, :]
        #             frac_time = t - time_table[i]       # get the time fraction within this section
        #             mid_time = (time_table[i+1] - time_table[i]) / 2
        #             # now it is between two points
        #             if frac_time < mid_time:  # 1. acceleration part
        #                 # acceleration can be removed
        #                 #x_ddot = np.dot(a_const, direction[i, :])   # acceleration is along the unit vector direction
        #                 #print('fractime', frac_time)
        #                 x_dot = np.dot(float(a_const * frac_time), direction[i,:])
        #                 x = front_point + np.dot(float(a_const * np.square(frac_time)/2), direction[i,:])
        #             elif frac_time == mid_time: # 2. turning point
        #                 x_dot = np.dot(float(a_const * mid_time), direction[i,:])
        #                 x = (front_point + next_point) / 2
        #             elif frac_time > mid_time:  # 3. deceleration part
        #                 # acceleration can be removed
        #                 #x_ddot = np.dot(-1*a_const, direction[i, :])  # acceleration is along the unit vector direction
        #                 x_dot = np.dot(float(a_const * (2 * mid_time -  frac_time)), direction[i, :])
        #                 x = next_point - np.dot(float(a_const * np.square(2 * mid_time - frac_time)/2), direction[i, :])
        #         elif t > time_table[i+1]:   # it is not within this section, continue the next
        #             continue

        # =================== Constant time interval =========================
        # if t >= self.point_num - 1:
        #     x = self.points[-1, :]
        # else:
        #     time = float(t)  # in case the t is not a float type
        #     current_section = int(math.floor(time))  # it is the current section, every section spends 1 sec
        #     in_section_time = time - math.floor(time)  # get the fractional part, reveals the location within two points
        #
        #     if in_section_time == 0.0:  # the quadrator is just at one point
        #         x = self.points[current_section, :]
        #     else:
        #         # these are only for the two points of the section
        #         former_point = self.points[current_section, :]
        #         next_point = self.points[current_section + 1, :]
        #         displacement = next_point - former_point  # 1 3 vector
        #         # then divide the section into acceleration section and deceleration section
        #         if in_section_time < 0.5:  # acceleration part
        #             x_ddot = np.dot(4, displacement)  # constant acceleration to the mid point then decelerate
        #             x_dot = np.dot(in_section_time, x_ddot)
        #             x = np.dot(in_section_time, x_dot) / 2 + former_point  # former coordinate plus the distance covered
        #         elif in_section_time == 0.5:  # turning point/ mid point
        #             x = (former_point + next_point) / 2
        #             x_dot = np.dot(2, displacement)  # reaching the highest speed
        #         elif in_section_time > 0.5:
        #             x_ddot = np.dot(-4, displacement)  # constant acceleration away from mid point
        #             x_dot = np.dot(2, displacement) + np.dot(in_section_time - 0.5, x_ddot)  # plus minus value
        #             x = np.dot(np.square(in_section_time - 0.5), x_ddot) / 2 + (
        #                         former_point + next_point) / 2  # former coordinate plus the distance covered

        flat_output = { 'x':x, 'x_dot':x_dot, 'x_ddot':x_ddot, 'x_dddot':x_dddot, 'x_ddddot':x_ddddot,
                        'yaw':yaw, 'yaw_dot':yaw_dot}
        return flat_output
