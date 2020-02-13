"""
Implementation of A* with Operator Decomposition as described by Trevor Standley in his 2010 paper
"Finding Optimal Solutions to Cooperative Pathfinding Problems". This implementation, however, must deal with time
uncertainty.

- Tomer Shahar, 2019
"""
from pathfinding.planners.utils.time_error import OutOfTimeError
from pathfinding.planners.utils.custom_heap import OpenListHeap
from pathfinding.planners.utils.operator_decomp_state import ODState
import math
import time

STAY_STILL_COST = 1


class ODAStar:

    def __init__(self, uncertain_problem):

        self.grid_map = uncertain_problem
        self.edges_and_weights = uncertain_problem.edges_and_weights
        self.startPositions = uncertain_problem.start_positions
        self.goalPositions = uncertain_problem.goal_positions
        self.open_dict = {}
        self.closed_set = set()
        self.open_list = OpenListHeap()
        self.objective = 'min_best_case'
        self.max_nodes = 115000 + 340000

    def find_solution(self, time_limit=5, objective='min_best_case', sic=True, min_time_policy=True):
        """
        Computes a solution for the given conformant problem. Returns a tuple
        solution, cost
        :param min_time_policy: How to choose the next agent to move. If true, chooses the agent with the minimal
        current time. Otherwise, just do it in a serial fashion.
        :param sic: If to compute the state's g value as the sum of all costs, or the max cost (makespan)
        :param time_limit: Max time for search, in seconds. Default is 5 minutes.
        :param objective: What we want to minimize
        :return: A tuple
        """
        self.open_list = OpenListHeap()
        self.open_dict = {}  # A dictionary mapping node tuple to the actual node. Used for faster access.
        self.closed_set = set()
        start_time = time.time()
        start_node = ODState(self.startPositions, self.goalPositions, None, None, (0, 0), self.grid_map)
        self.__add_node_to_open(start_node, objective)
        while len(self.open_list.internal_heap) > 0:
            if time.time() - start_time > time_limit:
                print(f"ODA timed out. Nodes expanded: {len(self.closed_set)}")
                print(f'Number of nodes in open: {len(self.open_dict)}')
                raise OutOfTimeError('Ran out of time :-(')
            if len(self.open_dict) + len(self.closed_set) > self.max_nodes:
                raise OutOfTimeError('Expanded too many nodes - Exit prior to memory error :-(')
            best_node = self.open_list.pop()  # removes from open_list
            self.__remove_node_from_open(best_node)

            if self.is_goal_node(best_node):
                return best_node.calc_solution(self.startPositions, sic) + (len(self.closed_set),)

            successors = best_node.expand(self.grid_map, sic, min_time_policy)

            for neighbor in successors:
                neighbor_tuple = neighbor.create_tuple()
                if neighbor_tuple in self.closed_set:  # Has been expanded.
                    continue
                if neighbor_tuple not in self.open_dict:  # Reached for the first time.
                    self.__add_node_to_open(neighbor, objective)
                else:  # We might need to update the node.
                    old_node = self.open_dict[neighbor_tuple]  # Node wac reached before
                    if self.is_new_node_better(neighbor, old_node, objective):
                        self.__update_node(neighbor, old_node)

        return None, math.inf  # no solution

    def is_goal_node(self, node):
        """
        Checks if a given node is a goal node. A node can only be a goal node if all agents have moved, and each agent
        is in their respective goal locations. We must additionally make sure that all agents are at the goal node at a
        global minimum time. This is to detect collisions that occur when an agent waits at the goal node for a long
        time and later on will collide with some other agent crossing it.

        :param node: The given node to check
        :return: True if it's a goal state, otherwise false
        """
        for agent, position in node.curr_positions.items():

            if position['location'] != self.goalPositions[agent]:
                return False

        return True

    @staticmethod
    def get_min_max_time(node):

        max_time = -1
        for agent, position in node.curr_positions.items():

            if position['time'][0] > max_time:
                max_time = position['time'][0]

        return max_time

    def __remove_node_from_open(self, node):
        node_tuple = node.create_tuple()
        self.open_dict.pop(node_tuple, None)
        self.closed_set.add(node_tuple)

    def __add_node_to_open(self, node, objective):
        switcher = {
            'min_best_case': self.push_min_best,
            'min_worst_case': self.push_worst_best
        }
        push_func = switcher[objective]  # Push according to the objective function.
        push_func(node)
        key_tuple = node.create_tuple()
        self.open_dict[key_tuple] = node

        return node

    def push_min_best(self, node):
        self.open_list.push(node, node.f_val[0], node.h_val, node.g_val[0])

    def push_worst_best(self, node):
        self.open_list.push(node, node.f_val[1], node.h_val, node.g_val[1])

    @staticmethod
    def __update_node(new_node, old_node):
        old_node.g_val = new_node.g_val
        old_node.h_val = new_node.h_val
        old_node.f_val = new_node.f_val
        # ToDo: Use heapify?

    def is_new_node_better(self, new_node, old_node, objective):
        """
        Function for comparing the new node to the old node according to a given objective.
        :param new_node: The new state that was generated
        :param old_node: The old state that was reached before
        :param objective: The goal, e.g. minimizing the worst case.
        :return: True if the new state is better.
        """
        switcher = {
            'min_best_case': self.min_best_case,
            'min_worst_case': self.min_worst_case
        }

        compare_func = switcher[objective]
        return compare_func(new_node, old_node)

    @staticmethod
    def min_best_case(new_node, old_node):
        """
        Returns true if the new state has a better best-case than the old node.
        """
        return new_node.g_val[0] < old_node.g_val[0]

    @staticmethod
    def min_worst_case(new_node, old_node):
        """
        Returns true if the new state has a better worst-case than the old node.
        """
        return new_node.g_val[1] < old_node.g_val[1]

