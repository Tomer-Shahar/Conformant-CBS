"""
A class that represents an algorithm for solving the path for multiple agents / single agent in accordance with
a group of constraints.

Currently a naive implementation of A*
"""

import time
import math
import networkx

# The positions of each parameter in the tuple receives from map.edges
VERTEX_ID = 0
MIN_EDGE_TIME = 1
MAX_EDGE_TIME = 2
STAY_STILL_COST = 1  # ToDO: Change value to greater than 1. This entails finding a more efficient way to stay still


class ConstraintAstar:

    def __init__(self, ccbs_map):
        self.map = ccbs_map

    """
    Computes the path for a particular agent in a given node. The node should contain the new constraint for this
    agents. Note that the agent is simply an int (the agent's id).

    At the beginning, a quick check is performed to verify whether a solution can potentially exist. That is,
    there is some sort of path for the agent without verifying the constraints.

    returns a tuple containing (agent_id, cost, [ path list])

    *** Currently naive implementation (basic A* with constraints)****
    """

    def compute_agent_path(self, constraints, agent, start_pos, goal_pos, min_best_case=True):
        open_list = []  # Todo: Use a more efficient data structure like tree or something
        open_dict = {}  # A dictionary mapping node tuple to the actual node. Used for faster access..
        closed_set = set()

        start_node = SingleAgentNode(start_pos, None, (0, 0), self.map, goal_pos)
        self.__add_node_to_open(open_list, open_dict, start_node)
        expanded = -1
        while open_list:
            best_node = open_list.pop()  # removes from open_list
            expanded += 1
          #  if expanded % 1000 == 0 and expanded > 1:
          #      print("Nodes expanded: " + str(expanded))
            node_tuple = best_node.create_tuple()
            open_dict.pop(node_tuple, None)
            closed_set.add(node_tuple)
            if best_node.current_position == goal_pos and self.__can_stay_still(agent, best_node, constraints):
                return best_node.calc_path(agent)

            successors = best_node.expand(agent, constraints, self.map)
            for neighbor in successors:
                if neighbor in closed_set:
                    continue
                g_val = neighbor[1]
                if neighbor not in open_dict:
                    neighbor_node = SingleAgentNode(neighbor[0], best_node, neighbor[1], self.map, goal_pos)
                    self.__add_node_to_open(open_list, open_dict, neighbor_node)
                else:
                    neighbor_node = open_dict[neighbor]

                    if min_best_case and g_val[0] >= neighbor_node.g_val[0]:
                        continue  # No need to update node. Continue iterating successors
                    elif not min_best_case and g_val[1] >= neighbor_node.g_val[1]:
                        continue  # No need to update node. Continue iterating successors
                    print("Found a faster path for node " + neighbor_node.current_position)

                self.__update_node(neighbor_node, best_node, g_val, goal_pos, self.map)

            open_list.sort(key=lambda k: k.h_val, reverse=True)  # first sort by secondary key.  # ToDo: Reduce this!!
            if min_best_case:
                open_list.sort(key=lambda k: k.f_val[0], reverse=True)  # This allows tie-breaking. # ToDo: And this!!
            else:
                open_list.sort(key=lambda k: k.f_val[1], reverse=True)  # This allows tie-breaking. # ToDo: And this!!

        # print("No Solution!")
        return agent, None, math.inf  # no solution

    @staticmethod
    def __add_node_to_open(open_list, open_dict, node):
        open_list.append(node)
        key_tuple = node.create_tuple()
        open_dict[key_tuple] = node

        return node

    @staticmethod
    def __update_node(neighbor_node, prev_node, g_val, goal_pos, map):
        neighbor_node.prev_node = prev_node
        neighbor_node.g_val = g_val
        neighbor_node.h_val = map.calc_heuristic(neighbor_node.current_position, goal_pos)
        neighbor_node.f_val = g_val[0] + neighbor_node.h_val, g_val[1] + neighbor_node.h_val

    def trivial_path(self, start_pos, goal_pos):
        """
        Function that determines whether a basic solution even exists. This is necessary for cases
        where an agent is "walled-off" and will waste a tremendous amount of time calculating a path that
        doesn't exist.

        node = (vertex_ID, g_val, h_val)
        """

        open_list = []  # Todo: Use a more efficient data structure like heap
        open_dict = {}  # A dictionary mapping node tuple to the actual node. Used for faster access..
        closed_set = set()

        g_val = 0
        h_val = self.map.calc_heuristic(start_pos, goal_pos)
        start_node = (start_pos, g_val, h_val)
        open_list.append(start_node)
        open_dict[start_pos] = start_node
        while open_list:
            best_node = open_list.pop()  # removes from open_list
            open_dict.pop(best_node[0], None)
            closed_set.add(best_node[0])
            if best_node[0] == goal_pos:
                return True

            successors = []
            for edge_tuple in self.map.edges_and_weights[best_node[0]]:
                successors.append(edge_tuple[VERTEX_ID])
            for neighbor in successors:
                if neighbor in closed_set:
                    continue
                g_val = best_node[1] + 1
                if neighbor not in open_dict:
                    h_val = self.map.calc_heuristic(neighbor, goal_pos)
                    neighbor_node = (neighbor, g_val, h_val)
                    open_list.append(neighbor_node)
                    open_dict[neighbor] = neighbor_node
                else:
                    neighbor_node = open_dict[neighbor]
                    if g_val >= neighbor_node[1]:  # No need to update node. Continue iterating successors
                        continue
                    open_dict[neighbor] = (neighbor, g_val, neighbor_node[2])  # Update the g_val, keep h_val though

            open_list = sorted(open_list, key=lambda k: k[2], reverse=True)  # first sort by secondary key.
            open_list = sorted(open_list, key=lambda k: k[1] + k[2], reverse=True)
        return None

    def trivial_solution(self, init_positions, goal_positions):
        for agent_id, agent_start in init_positions.items():
            if not self.trivial_path(agent_start, goal_positions[agent_id]):
                #  print("No trivial path found for agent " + str(agent_id))
                return None  # no solution

        return True

    @staticmethod
    def __can_stay_still(agent, best_node, constraints):
        """
        A function that verifies that the agent has reached the goal at an appropriate time. Useful when an agent
        reaches the goal in, for example, 5-8 time units however in the solution it takes another agent 10-15 time units.
        Therefor we must verify what happens if this agent stands still all this time (there might be a constraint!)
        """
        for con in constraints:  # ToDo: Iterate over all constraints or send the max time to stand?

            if con[0] == agent and con[1] == best_node.current_position and best_node.g_val[0] <= con[2]:
                return False

        return True

    def dijkstra_solution(self, source_vertex):
        """
        A function that calculates, using Dijkstra's algorithm, the distance from each point in the map to the given
        goal. This will be used as a perfect heuristic. It receives as input the goal position and returns a dictionary
        mapping each coordinate to the distance from it to the goal.

        For now we will use the minimum time taken to pass an edge as the weight, in order to keep the heuristic
        admissible.
        """

        graph = networkx.Graph()
        for vertex, edges in self.map.edges_and_weights.items():
            for edge in edges:
                graph.add_edge(vertex, edge[0], weight=edge[1])
        return networkx.single_source_dijkstra_path_length(graph, source_vertex)


class SingleAgentNode:
    """
    The class that represents the nodes being created during the search for a single agent.
    """

    def __init__(self, current_position, prev_node, time_span, map, goal):
        # self.agent = agent
        self.current_position = current_position
        self.prev_node = prev_node
        self.h_val = map.calc_heuristic(current_position, goal)
        self.g_val = time_span  # The time to reach the node.
        self.f_val = self.g_val[0] + self.h_val, self.g_val[1] + self.h_val  # heuristic val + cost of predecessor

    """
    The function that creates all the possible vertices an agent can go to from the current node. For example,
    if a tuple from the map would be ((3,4), 1, 3) and the current time vector is t0, the agent can be at vertex (3,4)
    at times t0+1, t0+2 or t0+3.
    """
    def expand(self, agent, constraints, search_map):
        neighbors = []

        for edge_tuple in search_map.edges_and_weights[self.current_position]:
            successor_time = (self.g_val[0] + edge_tuple[MIN_EDGE_TIME], self.g_val[1] + edge_tuple[MAX_EDGE_TIME])
            vertex = edge_tuple[VERTEX_ID]
            successor = (vertex, successor_time)
            if self.legal_move(agent, successor, constraints):
                neighbors.append(successor)

        stay_still = (self.current_position, (self.g_val[0] + STAY_STILL_COST, self.g_val[1] + STAY_STILL_COST))
        if self.legal_move(agent, stay_still, constraints):  # Add the option of not moving.
            neighbors.append(stay_still)

        return neighbors

    def calc_path(self, agent):
        """
        returns the path calculated from the node by traversing the previous nodes.
        """
        path = []
        curr_node = self
        while curr_node:
            move = (curr_node.g_val, curr_node.current_position)
            path.insert(0, move)
            curr_node = curr_node.prev_node
        return agent, path, self.g_val

    def create_tuple(self):
        """
        Converts a node object into a tuple.
        (vertex, time)
        """
        return self.current_position, self.g_val

    def legal_move(self, agent, successor, constraints):

        """
        A function that checks if a certain movement is legal. First we check for vertex constraints and then edge
        constraints. The first loop checks for the time range that the agent might be at the next vertex.
        """
        successor_min_time = successor[1][0]
        successor_max_time = successor[1][1]
        current_min_time = self.g_val[0]

        for time_interval in range(successor_min_time, successor_max_time + 1):
            if (agent, successor[0], time_interval) in constraints:
                return False

        edge = min(self.current_position, successor[0]), max(self.current_position, successor[0])

        for time_interval in range(current_min_time+1, successor_max_time - 1):  # Edge constraint
            if (agent, edge, time_interval) in constraints:
                return False

        if successor_min_time == successor_max_time:  # instantaneous traversal
            if (agent, edge, successor_max_time) in constraints:
                return False

        return True


class OutOfTimeError(Exception):
    def __init__(self, value=None):
        self.value = value

    def __str__(self):
        return repr(self.value)
