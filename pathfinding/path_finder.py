"""
A class that represents an algorithm for solving the path for multiple agents / single agent in accordance with
a group of constraints.

Currently a naive implementation of A*
"""

import time
import math

# The positions of each parameter in the tuple receives from map.edges
VERTEX_ID = 0
MIN_EDGE_TIME = 1
MAX_EDGE_TIME = 2
STAY_STILL_COST = 1



class constraint_Astar:

    def __init__(self, start_positions, goal_positions, map):
        self.start_positions = start_positions
        self.goal_positions = goal_positions
        self.start_time = time.time()
        self.map = map
        self.constraints = {}

    def compute_individual_paths(self, ct_node, init_positions, constraints, time_limit=1000 * 60):
        """
        Computes the paths for every agent given their initial positions and constraints. Note that the goal positions
        are stored in self.goalPositions.
        The output is a dictionary with a path for each agent (aka a solution)
        """
        solution = {}
        self.constraints = constraints

        if len(constraints) == 0:
            if not self.trivial_solution(init_positions):
                print("Error: No trival solution found")
                return None

        for agent_id, agent_start in init_positions.items():
            if time.time() - self.start_time > time_limit:
                raise OutOfTimeError('Ran out of time while computing individual paths')
            agent_path = self.compute_agent_path(ct_node, agent_id, agent_start, self.goal_positions[agent_id])
            if agent_path[1]:  # Solution found
                solution[agent_id] = agent_path
            else:  # No solution for a particular agent
                print("No solution for agent number " + str(agent_id))
                return None


        return solution

    def compute_agent_path(self, new_CT_node, agent, start_pos, goal_pos):
        """
        Computes the path for a particular agent in a given node. The node should contain the new constraint for this
        agents. Note that the agent is simply an int (the agent's id).

        At the beginning, a quick check is performed to verify whether a solution can potentially exist. That is,
        there is some sort of path for the agent without verifying the constraints.

        returns a tuple containing (agent_id, cost, [ path list])

        *** Currently naive implementation (basic A* with constraints)****
        """
        open_list = []  # Todo: Use a more efficient data structure like tree or something
        open_dict = {}  # A dictionary mapping node tuple to the actual node. Used for faster access..
        closed_set = set()

        start_node = singleAgentNode(agent, start_pos, None, self.map)
        start_node.g_val = 0
        start_node.f_val = self.map.calc_heuristic(start_pos, goal_pos)
        self.__add_node_to_open(open_list, open_dict, start_node)
        while open_list:
            best_node = open_list.pop()  # removes from open_list
            node_tuple = best_node.create_tuple()
            open_dict.pop(node_tuple, None)
            closed_set.add(node_tuple)
            if best_node.current_position == goal_pos:
                return best_node.calc_path()

            successors = best_node.expand(new_CT_node.constraints)
            for neighbor in successors:
                if neighbor in closed_set:
                    continue
                g_val = neighbor[1]
                if neighbor not in open_dict:
                    neighbor_node = singleAgentNode(agent, neighbor[2], best_node, self.map)
                    self.__add_node_to_open(open_list, open_dict, neighbor_node)
                else:
                    neighbor_node = open_dict[neighbor]
                    if g_val >= neighbor_node.g_val:  # No need to update node. Continue iterating successors
                        continue

                self.__update_node(neighbor_node, best_node, g_val, goal_pos)

            open_list = sorted(open_list, key=lambda k: k.f_val, reverse=True)
        print("No Solution!")
        return agent, None, math.inf #no solution

    def set_start_time(self, start_time):
        self.start_time = start_time

    def __add_node_to_open(self, open_list, open_dict, node):
        open_list.append(node)
        key_tuple = node.create_tuple()
        open_dict[key_tuple] = node

        return node

    def __update_node(self, neighbor_node, prev_node, g_val, goal_pos):
        neighbor_node.prev_node = prev_node
        neighbor_node.g_val = g_val
        neighbor_node.f_val = g_val + neighbor_node.calc_heuristic(goal_pos)

    def trivial_path(self, agent, start_pos, goal_pos):
        """
        Function that determines whether a basic solution even exists. This is necessary for cases
        where an agent is "walled-off" and will waste a tremendous amount of time calculating a path that
        doesn't exist.
        """

        open_list = []  # Todo: Use a more efficient data structure like tree or something
        open_dict = {}  # A dictionary mapping node tuple to the actual node. Used for faster access..
        closed_set = set()

        start_node = singleAgentNode(agent, start_pos, None, self.map)
        start_node.g_val = 0
        start_node.f_val = self.map.calc_heuristic(start_pos, goal_pos)
        self.__add_node_to_open(open_list, open_dict, start_node)
        while open_list:
            best_node = open_list.pop()  # removes from open_list
            node_tuple = best_node.current_position
            open_dict.pop(node_tuple, None)
            closed_set.add(node_tuple)
            if best_node.current_position == goal_pos:
                return True

            successors = []
            for edge_tuple in self.map.edges_weights_and_timeSteps[best_node.current_position]:
                successors.append(int(edge_tuple[VERTEX_ID]))  # ToDo: how to properly express successor?
            for neighbor in successors:
                if neighbor in closed_set:
                    continue
                g_val = best_node.g_val + 1
                if neighbor not in open_dict:
                    neighbor_node = singleAgentNode(agent, neighbor, best_node, self.map)
                    open_list.append(neighbor_node)
                    key_tuple = neighbor
                    open_dict[key_tuple] = neighbor_node
                else:
                    neighbor_node = open_dict[neighbor]
                    if g_val >= neighbor_node.g_val:  # No need to update node. Continue iterating successors
                        continue

                self.__update_node(neighbor_node, best_node, g_val, goal_pos)

            open_list = sorted(open_list, key=lambda k: k.f_val, reverse=True)
        return None

    def trivial_solution(self, init_positions):
        for agent_id, agent_start in init_positions.items():
            if not self.trivial_path(agent_id, agent_start, self.goal_positions[agent_id]):
                #  print("No trivial path found for agent " + str(agent_id))
                return None # no solution

        return True

class singleAgentNode:
    """
    The class that represents the nodes being created during the search for a single agent.
    """

    def __init__(self, agent, current_position, prev_node, map):
        self.agent = agent
        self.current_position = current_position
        self.prev_node = prev_node
        self.f_val = math.inf  # heuristic val + cost of predecessor
        if prev_node:
            self.g_val = prev_node.g_val  # The time to reach the node.
            self.time = prev_node.time + 1  # The time when reaching the node
        else:  # if it's the root node..
            self.g_val = math.inf
            self.time = 0
        self.map = map

    def expand(self, vertex_constraints):
        """
        The function that creates all the possible vertices an agent can go to from the current node. For example,
        if a tuple from the map would be (5,1,3) and the current time is t0, the agent can be at vertex 5 at times
        t0+1, t0+2 or t0+3.
        """
        # vertex_ID : (rob_id, time, vertex_ID)
        neighbors = []

        for edge_tuple in self.map.edges_weights_and_timeSteps[self.current_position]:
            for time_val in range(int(edge_tuple[MIN_EDGE_TIME]), int(edge_tuple[MAX_EDGE_TIME])+1):
                    successor = (self.agent, self.time+time_val, int(edge_tuple[VERTEX_ID]))  # ToDo: how to properly express successor?
                    if self.__legal_move(successor,vertex_constraints):
                        neighbors.append(successor)

        stay_still = (self.agent, self.time+STAY_STILL_COST, self.current_position) # Add the option of not moving.
        if stay_still not in vertex_constraints:
            neighbors.append(stay_still)

        return neighbors

    def calc_heuristic(self, goal_position):
        return self.map.calc_heuristic(self.current_position, goal_position)

    def calc_path(self):
        """
        returns the path calculated from the node by traversing the previous nodes.
        """
        path = []
      #  path.append((self.agent, self.current_position))
        curr_node = self
        while curr_node:
            move = (curr_node.g_val, curr_node.current_position)
            path.insert(0, move)
            curr_node = curr_node.prev_node
        return self.agent, path, self.g_val

    def create_tuple(self):
        """
        Converts a node object into a tuple.
        (agent, time, vertex)
        """
        return self.agent, self.time, self.current_position

    def __legal_move(self, successor, vertex_constraints):
        return successor not in vertex_constraints and \
               (self.agent, self.time, (self.current_position, successor[2])) not in vertex_constraints


class OutOfTimeError(Exception):
    def __init__(self, value=None):
        self.value = value

    def __str__(self):
        return repr(self.value)
