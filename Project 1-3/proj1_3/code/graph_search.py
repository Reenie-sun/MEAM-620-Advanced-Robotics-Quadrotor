#========== Author: Zhuheng Jiang ===================
#========== Course: MEAM 620 Advanced Robotics ======
#========== Collaborators : Bozhou Zha, Yongxin Guo =============

from heapq import heappush, heappop  # Recommended.
import numpy as np
import math
from flightsim.world import World
from proj1_3.code.occupancy_map import OccupancyMap # Recommended.


#======== Pre-defined functions =================
def neighbor_nodes(index: tuple):  # THE INPUT INDEX TYPE must be a tuple
    search_range = (1, -1, 0)
    neighbor = []
    for x in search_range:
        for y in search_range:
            for z in search_range:
                neighbor.append(tuple((index[0]+x, index[1]+y, index[2]+z)))   # return a list of 26 tuples
    # neighbor.pop(-1)        # pop out the last 0,0,0 the self point
    return neighbor[:-1]


# =========== 1. Dijkstra algorithm ==========
def dijkstra(world, resolution, margin, start, goal):
    # While not required, we have provided an occupancy map you may use or modify.
    occ_map = OccupancyMap(world, resolution, margin)
    # Retrieve the index in the occupancy grid matrix corresponding to a position in space.
    start_index = tuple(occ_map.metric_to_index(start))
    goal_index = tuple(occ_map.metric_to_index(goal))
    map = occ_map.map                # initialize the map
    map_shape = occ_map.map.shape   # get the shape of the map [x, y, z]

    # initialization
    cost_matrix = np.zeros(map_shape)+1E5   # x y z matrix of great values 1E5
    cost_matrix[start_index] = 0        # start point's cost is 0
    cost_q = []         # declare the cost queue
    parent = dict()     # Tag: index; Content:parent index
    heappush(cost_q, (0, start_index))  # push the cost = 0 and start index into cost_q

    # main part of the iteration
    counter = 0
    while cost_q:    # while the queue is not empty
        counter += 1
        # pop out the minimum cost point out of queue
        min_cost_node = heappop(cost_q)
        current_index = min_cost_node[1]    # get the current index
        # not compared with the open nodes, since it is too time-consuming
        for neighbor in neighbor_nodes(current_index):     # iterate every neighbors
            if occ_map.is_valid_index(neighbor) and not occ_map.is_occupied_index(neighbor):   # within map and not collide with obstacles
                total_cost = min_cost_node[0] + np.sqrt(((neighbor[0]-current_index[0])*resolution[0])**2 \
                                                        +((neighbor[1]-current_index[1])*resolution[1])**2 \
                                                        +((neighbor[2]-current_index[2])*resolution[2])**2)
                if total_cost < cost_matrix[neighbor]:  # current node is a better node
                    heappush(cost_q, (total_cost, neighbor))   # push the new cost and the neighbor into the queue
                    cost_matrix[neighbor] = total_cost  # update the cost
                    parent[neighbor] = current_index   # update the parent to the current node

    if cost_matrix[goal_index] != 1E5:     # goal point cost is not inf, it has be searched
        path_points = [goal_index]
        while path_points[-1] is not start_index:  # break when the path's last point is the start point
            path_points.append(parent[path_points[-1]])
        path = np.zeros((len(path_points), 3))
        path_points.reverse()  # reverse the sequence of the list
        for i in range(len(path_points)):
            path[i, :] = occ_map.index_to_metric_center(path_points[i])  # convert the points from index back to metric
        path[0] = start
        path[-1] = goal
        return path
    else:
        return None   # no path found


def aStar(world, resolution, margin, start, goal):
    # While not required, we have provided an occupancy map you may use or modify.
    occ_map = OccupancyMap(world, resolution, margin)
    # Retrieve the index in the occupancy grid matrix corresponding to a position in space.
    start_index = tuple(occ_map.metric_to_index(start))
    goal_index = tuple(occ_map.metric_to_index(goal))
    map = occ_map.map  # initialize the map
    map_shape = occ_map.map.shape  # get the shape of the map [x, y, z]

    # initialization
    cost_matrix = np.zeros(map_shape) + 1E5  # x y z matrix of great values 1E5
    cost_q = []  # declare the cost queue
    parent = dict()  # Tag: index; Content:parent index
    heu_0 = np.sqrt((resolution[0]*(start_index[0]-goal_index[0]))**2 \
                    + (resolution[1]*(start_index[1]-goal_index[1]))**2 \
                    + (resolution[2]*(start_index[2]-goal_index[2]))**2)
    heappush(cost_q, (heu_0, start_index))  # push the cost = 0 and start index into cost_q
    cost_matrix[start_index] = 0  # start point's cost is 0

    # main part of the iteration
    while cost_q:  # while the queue is not empty
        # pop out the minimum cost point out of queue
        min_cost_node = heappop(cost_q)
        current_index = min_cost_node[1]  # get the current index
        # not compared with the open nodes, since it is too time-consuming
        for neighbor in neighbor_nodes(current_index):  # iterate every neighbors
            if occ_map.is_valid_index(neighbor) and not occ_map.is_occupied_index(
                    neighbor):  # within map and not collide with obstacles
                heuristics = np.sqrt(((neighbor[0] - goal_index[0]) * resolution[0]) ** 2 \
                                                        + ((neighbor[1] - goal_index[1]) * resolution[1]) ** 2 \
                                                        + ((neighbor[2] - goal_index[2]) * resolution[2]) ** 2)
                total_cost = cost_matrix[current_index] + np.sqrt(((neighbor[0] - current_index[0]) * resolution[0]) ** 2 \
                                                        + ((neighbor[1] - current_index[1]) * resolution[1]) ** 2 \
                                                        + ((neighbor[2] - current_index[2]) * resolution[2]) ** 2)
                if total_cost < cost_matrix[neighbor]:  # current node is a better node
                    cost_matrix[neighbor] = total_cost  # update the cost
                    parent[neighbor] = current_index  # update the parent to the current node
                    heappush(cost_q, (total_cost+heuristics, neighbor))  # push the new cost and the neighbor into the queue`

    if cost_matrix[goal_index] != 1E5:  # goal point cost is not inf, it has be searched
        path_points = [goal_index]
        while path_points[-1] is not start_index:  # break when the path's last point is the start point
            path_points.append(parent[path_points[-1]])
        path = np.zeros((len(path_points), 3))
        path_points.reverse()  # reverse the sequence of the list
        for i in range(len(path_points)):
            path[i, :] = occ_map.index_to_metric_center(path_points[i])  # convert the points from index back to metric
        path[0] = start
        path[-1] = goal
        return path
    else:
        return None  # no path found


def graph_search(world, resolution, margin, start, goal, astar):
    if astar is True:   # astar switch is turned down
        return aStar(world, resolution, margin, start, goal)   # the name of the function must be different with the boolean
    else:
        return dijkstra(world, resolution, margin, start, goal)

