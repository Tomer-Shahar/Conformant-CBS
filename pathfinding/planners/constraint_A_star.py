"""
A class that represents an algorithm for solving the path for multiple agents / single agent in accordance with
a group of constraints.

Currently a naive implementation of A*
"""

from pathfinding.planners.utils.custom_heap import OpenListHeap
from pathfinding.planners.utils.time_uncertainty_plan import TimeUncertaintyPlan
from pathfinding.planners.utils.time_error import OutOfTimeError

import math
import networkx
import time

# The positions of each parameter in the tuple receives from map.edges
VERTEX_ID = 0
STAY_STILL_COST = 1


class ConstraintAstar:

    def __init__(self, tu_problem):
        self.tu_problem = tu_problem
        self.open_list = OpenListHeap()
        self.open_dict = {}
        self.closed_list = set()
        self.agent = -1

    """
    Computes the path for a particular agent in a given node. The node should contain the new constraint for this
    agents. Note that the agent is simply an int (the agent's id).

    returns a tuple containing (agent_id, cost, [ path list])

    *** Currently naive implementation (basic A* with constraints)****
    """

    def compute_agent_path(self, constraints, agent, start_pos, goal_pos, conf_table, mbc=True, time_limit=5,
                           curr_time=(0, 0), pos_cons=None, suboptimal=False):

        self.open_list = OpenListHeap()
        self.open_dict = {}  # A dictionary mapping node tuple to the actual node. Used for faster access..
        self.closed_list = set()
        self.agent = agent
        start_time = time.time()
        start_node = SingleAgentNode(start_pos, None, curr_time, self.tu_problem, goal_pos, confs_created=0)
        self.__add_node_to_open(start_node, mbc)

        while len(self.open_list.internal_heap) > 0:
            if time.time() - start_time > time_limit:
                raise OutOfTimeError('Ran out of time in low level solver')  # no solution
            best_node = self.open_list.pop()  # removes from open_list

            if best_node.current_position == goal_pos and \
                    self.__can_stay(agent, best_node, constraints, suboptimal):
                #if self.tu_problem.calc_heuristic(start_pos, goal_pos) > 0:
                #    print(f'Ratio: {len(self.closed_list) / self.tu_problem.calc_heuristic(start_pos, goal_pos)}')
                return best_node.calc_path(agent)

            successors = best_node.expand(agent, constraints, conf_table, self.tu_problem, pos_cons, suboptimal)

            self.__remove_node_from_open(best_node)

            for neighbor in successors:
                if neighbor in self.closed_list:
                    continue
                g_val = neighbor[1]
                if neighbor not in self.open_dict:
                    neighbor_node = SingleAgentNode(neighbor[0], best_node, neighbor[1], self.tu_problem, goal_pos,
                                                    neighbor[2])
                    self.__add_node_to_open(neighbor_node, mbc)
                else:  # We've already reached this node but haven't expanded it.
                    neighbor_node = self.open_dict[neighbor]

                    if mbc and g_val[0] >= neighbor_node.g_val[0]:
                        continue  # No need to update node. Continue iterating successors
                    elif not mbc and g_val[1] >= neighbor_node.g_val[1]:
                        continue  # No need to update node. Continue iterating successors
                    print("****** Found a faster path for node *******" + neighbor_node.current_position)
                    self.__update_node(neighbor_node, best_node, g_val, goal_pos, self.tu_problem)
        return TimeUncertaintyPlan.get_empty_plan(agent)  # no solution

    def __remove_node_from_open(self, node):
        node_tuple = node.create_tuple()
        self.open_dict.pop(node_tuple, None)
        self.closed_list.add(node_tuple)

    def __add_node_to_open(self, node, min_best_case):
        if min_best_case:  # minimize the lower time bound
            self.open_list.push(node, node.g_val[0] + node.h_val, node.confs_created, -node.g_val[0])
        else:  # minimize the upper time bound ToDo: What is a better tie breaker? g or conflicts?
            self.open_list.push(node, node.g_val[1] + node.h_val, node.confs_created, -node.g_val[1])
        key_tuple = node.create_tuple()
        self.open_dict[key_tuple] = node

        return node

    @staticmethod
    def __update_node(neighbor_node, prev_node, g_val, goal_pos, grid_map):
        neighbor_node.prev_node = prev_node
        neighbor_node.g_val = g_val
        neighbor_node.h_val = grid_map.calc_heuristic(neighbor_node.current_position, goal_pos)
        neighbor_node.f_val = g_val[0] + neighbor_node.h_val, g_val[1] + neighbor_node.h_val

    def __can_stay(self, agent, best_node, constraints, suboptimal):
        """
        A function that verifies that the agent has reached the goal at an appropriate time. Useful when an agent
        reaches the goal in, for example, 5-8 time units however in the solution it takes another agent 10-15 time
        units.
        Therefor we must verify what happens if this agent stands still all this time (there might be a conflict!)
        :param agent: The agent that's searching
        :param best_node: The current path
        :param constraints: A set of constraints
        :param suboptimal: If true, constraints apply to all agents and not the typical (agent, vertex, time) setting.
        this means that constraints are just (vertex, time) and apply to all agents. Used in prioritized planning since
        there is no need to iterate over all of the constraints.
        :return: True if the agent can stay in the current place, False otherwise.
        """
        if suboptimal:
            if best_node.current_position in constraints:
                # max_t = max(t[1] for t in constraints[best_node.current_position])
                for tick in sorted(constraints[best_node.current_position], key=lambda k: k[1]):
                    if best_node.g_val[0] <= tick[1]:
                        self.__create_wait_node(best_node, tick[1])
                        self.open_dict = {}
                        self.open_list = OpenListHeap()
                        return False
            return True
        if best_node.current_position in constraints:
            for con in constraints[best_node.current_position]:
                if con[0] == agent and best_node.g_val[0] <= con[1][1]:
                    #print('found goal constraint: agent {} is at goal at time {}, constraint at time {}'.format(agent, best_node.g_val[0], con[1][1]))
                    return False  # A constraint was found

        return True

    def dijkstra_solution(self, source_vertex, min_best_case=False):
        """
        A function that calculates, using Dijkstra's algorithm, the distance from each point in the map to the given
        goal. This will be used as a perfect heuristic. It receives as input the goal position and returns a dictionary
        mapping each coordinate to the distance from it to the goal.

        For now we will use the minimum time taken to pass an edge as the weight, in order to keep the heuristic
        admissible.
        """

        graph = networkx.Graph()
        for vertex, edges in self.tu_problem.edges_and_weights.items():
            for edge in edges:
                if min_best_case:
                    graph.add_edge(vertex, edge[0], weight=edge[1][0])
                else:
                    graph.add_edge(vertex, edge[0], weight=edge[1][1])
        try:
            return networkx.single_source_dijkstra_path_length(graph, source_vertex)
        except ValueError:
            print("neg value wut")

    @staticmethod
    def overlapping(time_1, time_2):
        """ Returns true if the time intervals in 'time_1' and 'time_2' overlap.

        1: A<-- a<-->b -->B
        2: a<-- A -->b<---B>
        3: A<-- a<--- B --->b
        4: a<-- A<--->B --->b
        5: A<-->B
           a<-->b
        ===> A <= a <= B or a <= A <= b
        """
        return (time_1[0] <= time_2[0] <= time_1[1]) or (time_2[0] <= time_1[0] <= time_2[1])

    def __create_wait_node(self, best_node, con_time):
        """
        Function called when an agent has a constraint on their goal vertex in the future. We change the current state
        to be one where the agent simply waits at the goal and then continue the search from there.
        :param best_node: The current state
        :param con_time: The time of the constraint on the goal
        :return: Changes the current state.
        """
        curr_pos = best_node.current_position
        temp_best = SingleAgentNode(curr_pos, best_node.prev_node, best_node.g_val, self.tu_problem, curr_pos,
                                    best_node.confs_created)
        delta = best_node.g_val[1] - best_node.g_val[0]
        g_val = (best_node.g_val[0] + 1, best_node.g_val[0] + 1 + delta)
        prev_node = SingleAgentNode(curr_pos, temp_best, g_val, self.tu_problem, curr_pos, best_node.confs_created)
        for g in range(prev_node.g_val[0] + 1, con_time - 1):
            g_val = g, g + delta
            curr_node = SingleAgentNode(curr_pos, prev_node, g_val, self.tu_problem, curr_pos, best_node.confs_created)
            prev_node = curr_node

        best_node.prev_node = prev_node
        best_node.g_val = g_val[0] + 1, g_val[1] + 1


class SingleAgentNode:
    """
    The class that represents the nodes being created during the search for a single agent.
    """

    def __init__(self, current_position, prev_node, g, grid_map, goal, confs_created):
        self.current_position = current_position
        self.prev_node = prev_node
        self.h_val = grid_map.calc_heuristic(current_position, goal)
        self.g_val = g  # The time to reach the node.
        self.confs_created = confs_created
        self.f_val = self.g_val[0] + self.h_val, self.g_val[1] + self.h_val

    """
    The function that creates all the possible vertices an agent can go to from the current node. For example,
    if a tuple from the map would be ((3,4), 1, 3) and the current time vector is t0, the agent can be at vertex (3,4)
    at times t0+1, t0+2 or t0+3.
    """

    def expand(self, agent, constraints, conflict_table, search_map, pos_cons, suboptimal):
        neighbors = []

        still_time = (self.g_val[0] + STAY_STILL_COST, self.g_val[1] + STAY_STILL_COST)  # Add the option of not moving.
        if self.legal_move(agent, self.current_position, still_time, constraints, pos_cons, suboptimal):
            if len(conflict_table) > 0:
                confs_created = self.confs_created + self.count_conflicts(agent, conflict_table, (still_time[1], still_time[1]),
                                                                          (self.current_position, self.current_position))
                stay_still = (self.current_position, still_time, confs_created)
            else:
                stay_still = (self.current_position, still_time, 0)
            neighbors.append(stay_still)

        for edge_tuple in search_map.edges_and_weights[self.current_position]:
            successor_time = (self.g_val[0] + edge_tuple[1][0], self.g_val[1] + edge_tuple[1][1])
            vertex = edge_tuple[VERTEX_ID]
            if self.legal_move(agent, vertex, successor_time, constraints, pos_cons, suboptimal):
                if len(conflict_table) > 0:
                    confs_created = self.confs_created + self.count_conflicts(agent, conflict_table, successor_time,
                                                                              (self.current_position, vertex))
                    successor = (vertex, successor_time, confs_created)
                else:
                    successor = (vertex, successor_time, 0)

                neighbors.append(successor)

        return neighbors

    @staticmethod
    def count_conflicts(agent, conflict_table, succ_time, edge):
        """
        Checks how many new conflict will arise from traversing this edge, whether it leads to an edge conflict or a
        vertex conflict.
        :param agent: The agent we're currently planning for.
        :param conflict_table: The Conflict Avoidance Table.
        :param succ_time: The time we'll be occupying the vertex
        :param edge: The edge we're traversing. If we are staying in the same spot then it'll be (v, v) and not (v, u)
        :return: Number of overlapping time ticks.
        """
        new_confs = 0
        for other, locations in conflict_table.items():
            if other == agent:
                continue
            if edge[1] in locations:
                for pres in locations[edge[1]]:
                    if (pres[0] <= succ_time[0] <= pres[1]) or (succ_time[0] <= pres[0] <= succ_time[1]):
                        new_confs += min(succ_time[1], pres[1]) - max(succ_time[0], pres[0]) + 1
            if edge in locations:
                for pres in locations[edge]:
                    if (pres[0][0] <= succ_time[0] <= pres[0][1]) or (succ_time[0] <= pres[0][0] <= succ_time[1]):
                        new_confs += min(succ_time[1], pres[0][1]) - max(succ_time[0], pres[0][0]) + 1

        return new_confs

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
        return TimeUncertaintyPlan(agent, path, self.g_val)

    def create_tuple(self):
        """
        Converts a node object into a tuple.
        (vertex, time)
        """
        return self.current_position, self.g_val, self.confs_created

    def legal_move(self, agent, vertex, succ_time, constraints, pos_cons, suboptimal):  # ToDo: reduce this bottleneck

        """
        A function that checks if a certain movement is legal. First we check for vertex constraints and then edge
        constraints. The loop checks for the time range that the agent might be at the next vertex.
        vertex - The node the agent is traveling to
        time - The arrival time at vertex
        constraints - A set of constraints.

        Returns false if move is illegal. Returns true if move is fine.
        """
        edge = min(self.current_position, vertex), max(self.current_position, vertex)
        edge_time = self.calc_edge_time(succ_time)
        if suboptimal:
            if edge in constraints:
                for tick in constraints[edge]:
                    if (tick[0] <= edge_time[0] <= tick[1]) or (edge_time[0] <= tick[0] <= edge_time[1]):
                        return False
            if vertex in constraints:
                for tick in constraints[vertex]:
                    if (tick[0] <= succ_time[0] <= tick[1]) or (succ_time[0] <= tick[0] <= succ_time[1]):
                        return False
            return True
        if pos_cons:  # Only check positive constraints if it exists
            for tick in range(succ_time[0], succ_time[1] + 1):
                if (agent, vertex, tick) not in pos_cons:
                    return False

        if edge in constraints:
            for con in constraints[edge]:
                if con[0] == agent and (
                        (con[1][0] <= edge_time[0] <= con[1][1]) or (edge_time[0] <= con[1][1] <= edge_time[1])):
                    return False
        if vertex in constraints:
            for con in constraints[vertex]:
                if con[0] == agent and (
                        (con[1][0] <= succ_time[0] <= con[1][1]) or (succ_time[0] <= con[1][1] <= succ_time[1])):
                    return False
        return True

    def calc_edge_time(self, succ_time):
        """
        Calculates how much time an agent will occupy the edge it is traversing
        :param succ_time: The time at successor node
        :return:
        """
        if (succ_time[0] - self.g_val[0], succ_time[1] - self.g_val[1]) == (1, 1):
            return self.g_val[0], succ_time[1]
        return self.g_val[0], succ_time[1]
