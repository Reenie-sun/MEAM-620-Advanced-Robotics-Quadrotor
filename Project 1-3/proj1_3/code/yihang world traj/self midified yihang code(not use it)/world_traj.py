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
        self.world = world
        self.start = start
        self.goal = goal
        # You must choose resolution and margin parameters to use for path
        # planning. In the previous project these were provided to you; now you
        # must chose them for yourself. Your may try these default values, but
        # you should experiment with them!
        # self.resolution = np.array([0.25, 0.25, 0.25])     # 0.25
        # self.margin = 0.5
        self.resolution = np.array([0.125, 0.125, 0.125])     # 0.25
        self.margin = 0.3

        # You must store the dense path returned from your Dijkstra or AStar
        # graph search algorithm as an object member. You will need it for
        # debugging, it will be used when plotting results.
        self.path = graph_search(world, self.resolution, self.margin, start, goal, astar=True)


        # You must generate a sparse set of waypoints to fly between. Your
        # original Dijkstra or AStar path probably has too many points that are
        # too close together. Store these waypoints as a class member; you will
        # need it for debugging and it will be used when plotting results.
        # self.points = np.zeros((1,3)) # shape=(n_pts,3)
        self.points = np.array(self.get_sparse_point(self.path))  # convey the traj points, take the striaght line path
        self.Ps = self.points


        # Finally, you must compute a trajectory through the waypoints similar
        # to your task in the first project. One possibility is to use the
        # WaypointTraj object you already wrote in the first project. However,
        # you probably need to improve it using techniques we have learned this
        # semester.
        self.point_num = len(self.points)  # get N number of points

        # STUDENT CODE HERE
    def get_sparse_point(self, path_points):
        '''
        This function takes in the points from graph search and removes some unnecessary ones
        Input: path_points  a list[] of points in metrics
        Output: sparse_points a new list[] of points
        '''

        start = path_points[0]  # first point is the start
        goal = path_points[-1]  # last   goal

        # While not required, we have provided an occupancy map you may use or modify.
        occ_map = OccupancyMap(self.world, self.resolution, self.margin)
        # Retrieve the index in the occupancy grid matrix corresponding to a position in space.
        start_index = tuple(occ_map.metric_to_index(start))
        goal_index = tuple(occ_map.metric_to_index(goal))
        map = occ_map.map  # initialize the map

        map_shape = occ_map.map.shape  # get the shape of the map [x, y, z]
        print(map_shape) ############################### for test
        mean_edge = np.array(map_shape).mean() * np.mean(self.resolution)        # mean is used
        print(mean_edge, type(mean_edge))
        dis_threshold = 0.2*mean_edge   # 0.5 * mean_edge
        print(dis_threshold)

        #=========== Connect the points if not collide ========
        sparse_point = [start]  # return the sparse point list
        available_point = start       # set initial search point to be start point[0]
        check_point = path_points[1]    # checking start from the second point
        points_queue = path_points.copy().tolist()   # pop out the checked point / path_point is np.array, convert to list
        points_queue.pop(0)                 # remove the first start / point_queue must be a list for "pop"  attribute
        # while available_point is not goal:      # is goal is updated then the whole path is checked
        while points_queue:  # point_queue is not empty
            check_point = points_queue[0]       # the queue is being popped
            if self.collision_detect(np.array(available_point), np.array(check_point), occ_map) is True and \
                    np.linalg.norm(np.array(available_point)-np.array(check_point)) < dis_threshold:    # no collision & distance within range, connect!
                points_queue.pop(0)     # pop out the checked 1st point
                last_point = check_point    # current check_point is checked, store into the last point
                if (np.array(check_point) == np.array(goal)).all():  # 1. connection & 2. reached the goal point
                    sparse_point.append(check_point)    # At goal: 1. point_queue[0]== goal is popped; 2. store the check_point=goal into path
            else:       # collide, no pop; update: last_point --> available_point & sparse[], check_point start from this one in the next iter
                sparse_point.append(last_point)    # store the last point into sparse points
                available_point = last_point       # set teh last_point as the available_point
        return sparse_point     # return the sparse point list

    def collision_detect(self, point_pre, point_next, occ_map):     # map is the class object, for occupancy
        if (point_next == point_pre).all():    # the point is the same / the input must be two np.array() for .all() attribute
            return True                # NO collision
        else:
            dir_vec = point_next - point_pre  # it should be a 1-3 np.array
            sample_resolution = np.min(self.resolution)  # use the minimum resolution for sampling
            sample_amount = int(np.max(dir_vec) / sample_resolution)  # get most sampling as integer
            increment_vec = dir_vec / sample_amount  # increase one increment each time
            check_list = []
            for i in range(sample_amount):  # check each sampling points
                curr_point = point_pre + i * increment_vec  # 1st is the start
                check_list.append(curr_point)  # store the point in list
                if occ_map.is_occupied_metric(curr_point) or not occ_map.is_valid_metric(curr_point):  # check if the metric point is in obstacles
                    return False  # curr_point is occupied, False and break
                else:
                    continue
            return True  # NO collision, return True





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
        constV = 0.6  # the desired position will be moving at a rate of 1.6m/s    for lab, 0.6m/s

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
