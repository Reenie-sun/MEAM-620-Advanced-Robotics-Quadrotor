import numpy as np

from proj1_3.code.graph_search import graph_search
from proj1_3.code.occupancy_map import OccupancyMap # Recommended.


class WorldTraj(object):
    """

    """
    def __init__(self, world, start, goal):
        """
        This is the constructor for the trajectory object. A fresh trajectory
        object will be constructed before each mission. For a world trajectory,
        the input arguments are start and end positions and a world object. You
        are free to choose the path taken in any way you like.

        You should initialize parameters and pre-compute values such as
        polynomial coefficients here.

        Parameters:
            world, World object representing the environment obstacles
            start, xyz position in meters, shape=(3,)
            goal,  xyz position in meters, shape=(3,)

        """

        # You must choose resolution and margin parameters to use for path
        # planning. In the previous project these were provided to you; now you
        # must chose them for yourself. Your may try these default values, but
        # you should experiment with them!
        # self.resolution = np.array([0.25, 0.25, 0.25])
        # self.margin = 0.5

        # for lab:
        self.resolution = np.array([0.125, 0.125, 0.125])
        self.margin = 0.3


        # You must store the dense path returned from your Dijkstra or AStar
        # graph search algorithm as an object member. You will need it for
        # debugging, it will be used when plotting results.

        # You must generate a sparse set of waypoints to fly between. Your
        # original Dijkstra or AStar path probably has too many points that are
        # too close together. Store these waypoints as a class member; you will
        # need it for debugging and it will be used when plotting results.
        # self.points = np.zeros((1,3)) # shape=(n_pts,3)
        self.path = graph_search(world, self.resolution, self.margin, start, goal, astar=True)
        print(self.path)
        exit()
        self.points = self.path.copy()

        # print('points:\n', self.points)
        print(type(self.points))
        print(self.points.shape)

        if np.array_equal(self.points[0, :], self.points[1, :]):
            self.points = np.delete(self.points, 1, 0)
        # ------------------------------------------------------------------------
        # p_curr1 = 0
        # p_curr2 = 1
        # p_check = 2
        # v_curr = (self.points[p_curr2, :] - self.points[p_curr1, :]) / np.sqrt((np.square(self.points[p_curr2, :] - self.points[p_curr1, :])).sum())
        # v_check = (self.points[p_check, :] - self.points[p_curr1, :]) / np.sqrt((np.square(self.points[p_check, :] - self.points[p_curr1, :])).sum())
        #
        # while True:
        #     if np.linalg.norm(v_check - v_curr) < 0.00001:
        #         self.points = np.delete(self.points, p_curr2, 0)
        #         if p_check == self.points.shape[0] - 1:
        #             break
        #     else:
        #         p_curr1 = p_check - 1
        #         p_curr2 = p_curr1 + 1
        #         p_check = p_curr2 + 1
        #         v_curr = (self.points[p_curr2, :] - self.points[p_curr1, :]) / np.sqrt(
        #             (np.square(self.points[p_curr2, :] - self.points[p_curr1, :])).sum())
        #         if p_check == self.points.shape[0] - 1:
        #             break
        #
        #     v_check = (self.points[p_check, :] - self.points[p_curr1, :]) / np.sqrt(
        #         (np.square(self.points[p_check, :] - self.points[p_curr1, :])).sum())
        #
        # print('new points:\n', self.points)
        # print(self.points.shape)
        # self.Ps = self.points
        # ------------------------------------------------------------------------
        ph = 0
        pch = 1
        i = 0
        while True:
            i += 1
            # print(i)
            pathtmp = np.vstack((self.points[ph, :], self.points[pch, :]))
            # flag = world.path_collisions(pathtmp, margin)
            # print(flag.shape)
            # print('flag:', flag)
            if world.path_collisions(pathtmp, self.margin+0.05).shape[0] == 0:
                self.points = np.delete(self.points, pch, 0)
                # pch = pch + 1
            else:
                ph = pch
                pch = pch + 1
            if pch == self.points.shape[0] - 1:
                break

        print('after points:\n', self.points)
        self.Ps = self.points
        # exit()

        # Finally, you must compute a trajectory through the waypoints similar
        # to your task in the first project. One possibility is to use the
        # WaypointTraj object you already wrote in the first project. However,
        # you probably need to improve it using techniques we have learned this
        # semester.

        # STUDENT CODE HERE
        # smooth the trajectory


    def update(self, t):
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
        x_ddot   = np.zeros((3,))
        x_dddot  = np.zeros((3,))
        x_ddddot = np.zeros((3,))
        yaw = 0
        yaw_dot = 0

        # STUDENT CODE HERE
        print('t:', t)

        # print('the end of debugging-------')
        # exit()
        constV = 0.6 # the desired position will be moving at a rate of 1.6m/s    for lab, 0.6m/s

        global nextPos, nextMark, t_spent
        # nextPos stores the desired next position
        # nextMark stores the index of Ps indicating the next waypoint to reach
        # t_spent stores the time spent
        flag = 0

        if t == np.inf:  # simulation starts
            # self.Ps = np.vstack((self.Ps, [0, 0, 0]))
            nextPos = self.Ps[-1]
            # print('in if')
            # print('nextPos:', nextPos)

        # elif np.linalg.norm(self.Ps[-1] - nextPos) <= 0.001:
        #     nextPos = nextPos
        elif t == 0:
            nextPos = self.Ps[0]  # get the initial position
            nextMark = 0  # first waypoint

        else:
            if np.linalg.norm(self.Ps[nextMark] - nextPos) <= 0.01:  # when nextPos reaches the next waypoint
                # if nextMark < self.Ps.shape[0] - 1:
                #     nextMark += 1  # move to the next waypoint
                nextMark = min((nextMark + 1), (self.Ps.shape[0] - 1))
                if nextMark == (self.Ps.shape[0] - 1):  # reached goal, stop at goal
                    nextPos = nextPos
                else:
                    # flag = 1
                    # t_spent = t  # store the time spent before the new waypoints so that we can get the time spent on the new ruote
                    nextPos = nextPos + (self.Ps[nextMark] - nextPos) / np.linalg.norm(
                        self.Ps[nextMark] - nextPos) * 0.002 * constV

            else:  # haven't reached the point
                nextPos = nextPos + (self.Ps[nextMark] - nextPos) / np.linalg.norm(
                    self.Ps[nextMark] - nextPos) * 0.002 * constV
        #

        print('nextPos:', nextPos)
        # print('nextMark:', nextMark)
        # print('distance to next point:', np.linalg.norm(self.Ps[nextMark] - nextPos))

        x = nextPos
        # print(x)


        flat_output = { 'x':x, 'x_dot':x_dot, 'x_ddot':x_ddot, 'x_dddot':x_dddot, 'x_ddddot':x_ddddot,
                        'yaw':yaw, 'yaw_dot':yaw_dot}
        return flat_output
