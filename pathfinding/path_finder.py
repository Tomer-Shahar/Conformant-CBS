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
STAY_STILL_COST = 1  # ToDO: Change value to greater than 1. This entails finding a more efficient way to stay still


class constraint_Astar:

    def __init__(self, start_positions, goal_positions, map):
        self.start_positions = start_positions
        self.goal_positions = goal_positions
        #self.start_time = time.time()
        self.map = map
        self.constraints = {}

    def compute_individual_paths(self, ct_node, init_positions, constraints, min_time, time_limit=1000 * 60):
        """
        Computes the paths for every agent given their initial positions and constraints. Note that the goal positions
        are stored in self.goalPositions.
        The output is a dictionary with a path for each agent (aka a solution)
        """
        solution = {}
        self.constraints = constraints  # ToDO: Can I remove this data member?

        if len(constraints) == 0:
            if not self.trivial_solution(init_positions):
                print("Error: No trival solution found")
                return None

        for agent_id, agent_start in init_positions.items():
            if time.time() - self.start_time > time_limit:
                raise OutOfTimeError('Ran out of time while computing individual paths')
            agent_path = self.compute_agent_path(ct_node, agent_id, agent_start, self.goal_positions[agent_id],
                                                 min_time)
            print("Found path for agent " + str(agent_id))
            if agent_path[1]:  # Solution found
                solution[agent_id] = agent_path
            else:  # No solution for a particular agent
                print("No solution for agent number " + str(agent_id))
                return None

        return solution

    def compute_agent_path(self, new_CT_node, agent, start_pos, goal_pos, min_time, min_best_case=True):
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

        start_node = singleAgentNode(start_pos, None, (0, 0), self.map, goal_pos)
        #start_node.h_val = self.map.calc_heuristic(start_pos, goal_pos)
        #start_node.f_val = (start_node.h_val, start_node.h_val)
        self.__add_node_to_open(open_list, open_dict, start_node)
        expanded = -1
        while open_list:
            best_node = open_list.pop()  # removes from open_list
            expanded += 1
            if expanded % 1000 == 0 and expanded > 1:
                print("Nodes expanded: " + str(expanded))
            node_tuple = best_node.create_tuple()
            open_dict.pop(node_tuple, None)
            closed_set.add(node_tuple)
            if best_node.current_position == goal_pos and self.__min_time_reached(best_node, min_time):
                return best_node.calc_path(agent)

            successors = best_node.expand(agent, new_CT_node.constraints, self.map)
            for neighbor in successors:
                if neighbor in closed_set:
                    continue
                g_val = neighbor[0]
                if neighbor not in open_dict:
                    neighbor_node = singleAgentNode\
                        (neighbor[1], best_node, (neighbor[0][0], neighbor[0][1]), self.map, goal_pos)
                    self.__add_node_to_open(open_list, open_dict, neighbor_node)
                else:
                   # print("Expanded a node in open list")
                    neighbor_node = open_dict[neighbor]
                    # TODO: How to address the g value? Min cost? Max cost? Average?

                    if min_best_case and g_val[0] >= neighbor_node.g_val[0]:
                        continue  # No need to update node. Continue iterating successors
                    elif not min_best_case and g_val[1] >= neighbor_node.g_val[1]:
                        continue  # No need to update node. Continue iterating successors
                    print("Found a faster path for node " + neighbor_node[1])

                self.__update_node(neighbor_node, best_node, g_val, goal_pos, self.map)

            open_list = sorted(open_list, key=lambda k: k.h_val, reverse=True)  # first sort by secondary key.
            if min_best_case:
                open_list = sorted(open_list, key=lambda k: k.f_val[0], reverse=True)  # This allows tie-breaking.
            else:
                open_list = sorted(open_list, key=lambda k: k.f_val[1], reverse=True)  # This allows tie-breaking.

       # print("No Solution!")
        return agent, None, math.inf  # no solution

    def set_start_time(self, start_time):
        self.start_time = start_time

    def __add_node_to_open(self, open_list, open_dict, node):
        open_list.append(node)
        key_tuple = node.create_tuple()
        open_dict[key_tuple] = node

        return node

    def __update_node(self, neighbor_node, prev_node, g_val, goal_pos, map):
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
            for edge_tuple in self.map.edges_weights_and_timeSteps[best_node[0]]:
                successors.append(int(edge_tuple[VERTEX_ID]))  # ToDo: how to properly express successor?
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

    def trivial_solution(self, init_positions):
        for agent_id, agent_start in init_positions.items():
            if not self.trivial_path(agent_start, self.goal_positions[agent_id]):
                #  print("No trivial path found for agent " + str(agent_id))
                return None  # no solution

        return True

    def __min_time_reached(self, best_node, min_time):
        """
        A function that verifies that the agent has reached the goal at an appropriate time. Useful when an agent
        reaches the goal in, for example, 5 time units however in the solution it takes another agent 10 time units.
        Therefor we must verify what happens if this agent stands still all this time (there might be a constraint!)
        """
        return best_node.f_val[0] >= min_time

    def dijkstra_solution(self, source_vertex):
        """
        A function that calculates, using Dijkstra's algorithm, the distance from each point in the map to the given
        goal. This will be used as a perfect heuristic. It receives as input the goal position and returns a dictionary
        mapping each coordinate to the distance from it to the goal.

        For now we will use the minimum time taken to pass an edge as the weight, in order to keep the heuristic
        admissible.
        """
        distances = {}
        # prev = {}
        vertices = []  # list of vertices for which we haven't yet found the shortest path

        for vertex in self.map.edges_weights_and_timeSteps:
            distances[vertex] = math.inf
            vertices.append(vertex)

        distances[source_vertex] = 0
        vertices.sort(key=lambda k: distances[k], reverse=True)
        iteration = 0

        while vertices:
            iteration+=1
            print(iteration)
            best_node = vertices.pop()
            position = self.map.vertex_id_to_coordinate(best_node)
            for edge in self.map.edges_weights_and_timeSteps[best_node]:  # iterate over neighbors
                alternate = distances[best_node] + edge[1]
                if alternate < distances[edge[0]]:
                    distances[edge[0]] = alternate
                    # prev[edge[0]] = curr_vertex

            vertices.sort(key=lambda k: distances[k], reverse=True)

        return distances


class singleAgentNode:
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

    def expand(self, agent, constraints, map):
        """
        The function that creates all the possible vertices an agent can go to from the current node. For example,
        if a tuple from the map would be (5,1,3) and the current time is t0, the agent can be at vertex 5 at times
        t0+1, t0+2 or t0+3.
        """
        # vertex_ID : ( <min_time, max_time> , vertex_ID)
        neighbors = []

        for edge_tuple in map.edges_weights_and_timeSteps[self.current_position]:
            # for time_val in range(int(edge_tuple[MIN_EDGE_TIME]), int(edge_tuple[MAX_EDGE_TIME])+1):
            successor = ((self.g_val[0] + edge_tuple[MIN_EDGE_TIME], self.g_val[1] + edge_tuple[MAX_EDGE_TIME]),
                         int(edge_tuple[VERTEX_ID]))  # ToDo: how to properly express successor?
            if self.__legal_move(agent, successor, constraints):
                neighbors.append(successor)

        stay_still = ((self.g_val[0] + STAY_STILL_COST, self.g_val[1] + STAY_STILL_COST),
                      self.current_position)  # Add the option of not moving.
        if self.__legal_move(agent, stay_still, constraints):
            neighbors.append(stay_still)

        return neighbors

    def calc_path(self, agent):
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
        return agent, path, self.g_val

    def create_tuple(self):
        """
        Converts a node object into a tuple.
        (agent, time, vertex)
        """
        return self.g_val, self.current_position

    def __legal_move(self, agent, successor, vertex_constraints):

        """
        A function that checks if a certain movement is legal. First we check for vertex constraints and then edge
        constraints. The first loop checks for the time range that the agent might be at the next vertex.
        """

        for time_interval in range(successor[0][0], successor[0][1] + 1):
            if (agent, time_interval, successor[1]) in vertex_constraints:
                return False
        for time_interval in range(self.g_val[0], successor[0][1] + 1):  # Edge constraint
            edge = min(self.current_position, successor[1]), max(self.current_position, successor[1])
            if (agent, time_interval, edge) in vertex_constraints:
                return False

        return True


class OutOfTimeError(Exception):
    def __init__(self, value=None):
        self.value = value

    def __str__(self):
        return repr(self.value)
