"""
Implementation of A* with Operator Decomposition as described by Trevor Standley in his 2010 paper
"Finding Optimal Solutions to Cooperative Pathfinding Problems". This implementation, however, must deal with time
uncertainty.

- Tomer Shahar, 2019
"""
from pathfinding.time_error import OutOfTimeError
from pathfinding.custom_heap import OpenListHeap
import math
import time


class ODAStar:

    def __init__(self, conformed_problem):

        self.grid_map = conformed_problem.map
        self.edges_and_weights = conformed_problem.edges_and_weights
        self.startPositions = conformed_problem.start_positions
        self.goalPositions = conformed_problem.goal_positions
        self.open_dict = {}
        self.closed_set = set()
        self.open_list = OpenListHeap()

    def create_solution(self, time_limit, min_best_case=True):
        self.open_list = OpenListHeap()
        self.open_dict = {}  # A dictionary mapping node tuple to the actual node. Used for faster access..
        self.closed_set = set()
        start_time = time.time()
        star_node = StandardState(self.startPositions, {}, self.goalPositions, None, 0, self.grid_map)

        while len(self.open_list.internal_heap) > 0:
            if time.time() - start_time > time_limit:
                raise OutOfTimeError('Ran out of time :-(')
            best_node = self.open_list.pop()  # removes from open_list
            self.__remove_node_from_open(best_node)

            if self.is_goal_node(best_node):
                return best_node.calc_solution()

            successors = best_node.expand(self.grid_map)
            for neighbor in successors:
                if neighbor in self.closed_set:
                    continue

                neighbor_g_val = neighbor[1]

                if neighbor not in self.open_dict:
                    neighbor_node = StandardState(
                        neighbor[0], neighbor[1], self.goalPositions, best_node, neighbor[2], self.grid_map)
                    self.__add_node_to_open(neighbor_node)
                elif neighbor_g_val > self.open_dict[neighbor]:  # No improvement
                    continue



        return None, math.inf  # no solution

    def is_goal_node(self, node):

        if len(node.assigned_moves) > 0:
            return False

        for agent, position in node.current_positions.items():

            if position != self.goalPositions[agent]:
                return False

        return True

    def __remove_node_from_open(self, node):
        node_tuple = node.create_tuple()
        self.open_dict.pop(node_tuple, None)
        self.closed_set.add(node_tuple)

    def __add_node_to_open(self, node):
        self.open_list.push(node)
        key_tuple = node.create_tuple()
        self.open_dict[key_tuple] = node

        return node

    @staticmethod
    def __update_node(neighbor_node, prev_node, g_val, goal_pos, grid_map):
        neighbor_node.prev_node = prev_node
        neighbor_node.g_val = g_val
        neighbor_node.h_val = grid_map.calc_heuristic(neighbor_node.current_position, goal_pos)
        neighbor_node.f_val = g_val[0] + neighbor_node.h_val, g_val[1] + neighbor_node.h_val


class StandardState:
    """
    Class for representing standard states where each agent hasn't been assigned a movement yet.
    """
    def __init__(self, curr_positions, assigned_moves, goals, prev_node, g_val, grid_map):
        self.curr_positions = curr_positions
        self.assigned_moves = assigned_moves
        self.prev_node = prev_node
        self.h_val = self.calc_heuristic(curr_positions, goals, grid_map)
        self.g_val = g_val  # The time to reach the node.
        self.f_val = self.g_val[0] + self.h_val, self.g_val[1] + self.h_val  # heuristic val + cost of predecessor

    @staticmethod
    def calc_heuristic(curr_positions, goals, grid_map):

        heuristic_sum = 0

        for agent, state in goals.items():
            heuristic_sum += grid_map.calc_heuristic(curr_positions[agent], state)

        return heuristic_sum

    def create_tuple(self):
        return self.curr_positions, self.assigned_moves, self.g_val

    def expand(self, grid_map):
        pass


class IntermediateState:
    """
    Class for representing states where at least one agent has been assigned a move.
    """

    def __init__(self, curr_positions, assigned_moves, goals, prev_node, g_val, grid_map):
        pass

    @staticmethod
    def calc_heuristic(curr_positions, goals, grid_map):

        heuristic_sum = 0

        for agent, state in goals.items():
            heuristic_sum += grid_map.calc_heuristic(curr_positions[agent], state)

        return heuristic_sum
