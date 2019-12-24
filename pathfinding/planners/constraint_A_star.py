"""
A class that represents an algorithm for solving the path for multiple agents / single agent in accordance with
a group of constraints.

Currently a naive implementation of A*
"""

from pathfinding.planners.utils.custom_heap import OpenListHeap
from pathfinding.planners.utils.time_error import OutOfTimeError
from pathfinding.planners.utils.time_uncertainty_plan import TimeUncertainPlan
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
        self.closed_set = set()
        self.agent = 1

    """
    Computes the path for a particular agent in a given node. The node should contain the new constraint for this
    agents. Note that the agent is simply an int (the agent's id).

    returns a tuple containing (agent_id, cost, [ path list])

    *** Currently naive implementation (basic A* with constraints)****
    """

    def compute_agent_path(self, constraints, agent, start_pos, goal_pos, conf_table,
                           min_best_case=True, time_limit=5, curr_time=(0, 0), pos_cons=None, suboptimal=False):

        self.open_list = OpenListHeap()
        self.open_dict = {}  # A dictionary mapping node tuple to the actual node. Used for faster access..
        self.closed_set = set()
        self.agent = agent
        start_time = time.time()
        start_node = SingleAgentNode(start_pos, None, curr_time, self.tu_problem, goal_pos, conflicts_created=0)
        self.__add_node_to_open(start_node, min_best_case)

        while len(self.open_list.internal_heap) > 0:
            if time.time() - start_time > time_limit:
                raise OutOfTimeError('Ran out of time :-(')
            best_node = self.open_list.pop()  # removes from open_list
            self.__remove_node_from_open(best_node)

            if best_node.current_position == goal_pos and self.__can_stay(agent, best_node, constraints, suboptimal):
                return best_node.calc_path(agent)

            successors = best_node.expand(agent, constraints, conf_table, self.tu_problem, pos_cons)
            for neighbor in successors:
                if neighbor in self.closed_set:
                    continue
                g_val = neighbor[1]
                if neighbor not in self.open_dict:
                    neighbor_node = SingleAgentNode(neighbor[0], best_node, neighbor[1], self.tu_problem, goal_pos,
                                                    neighbor[2])
                    self.__add_node_to_open(neighbor_node, min_best_case)
                else:
                    neighbor_node = self.open_dict[neighbor]

                    if min_best_case and g_val[0] >= neighbor_node.g_val[0]:
                        continue  # No need to update node. Continue iterating successors
                    elif not min_best_case and g_val[1] >= neighbor_node.g_val[1]:
                        continue  # No need to update node. Continue iterating successors
                    # print("Found a faster path for node " + neighbor_node.current_position)
                self.__update_node(neighbor_node, best_node, g_val, goal_pos, self.tu_problem)
        return TimeUncertainPlan(agent, None, math.inf)  # no solution

    def __remove_node_from_open(self, node):
        node_tuple = node.create_tuple()
        self.open_dict.pop(node_tuple, None)
        self.closed_set.add(node_tuple)

    def __add_node_to_open(self, node, min_best_case):
        if min_best_case:  # minimize the lower time bound
            self.open_list.push(node, node.f_val[0], node.conflicts_created, node.h_val)
        else:  # minimize the upper time bound
            self.open_list.push(node, node.f_val[1], node.conflicts_created, node.h_val)
        key_tuple = node.create_tuple()
        try:
            self.open_dict[key_tuple] = node
        except MemoryError:
            print('memory error in add node to open')

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
        """

        for con in constraints:  # ToDo: Iterate over all constraints or send the max time to stand?
            if con[0] == agent and con[1] == best_node.current_position and best_node.g_val[0] <= con[2][1]:
                if suboptimal:
                    self.__create_wait_node(best_node, con[2][0])
                    self.open_dict = {}
                    self.open_list = OpenListHeap()
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

        return networkx.single_source_dijkstra_path_length(graph, source_vertex)

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
        try:
            if (time_1[0] <= time_2[0] <= time_1[1]) or (time_2[0] <= time_1[0] <= time_2[1]):
                return True
        except TypeError:
            print("fudge - type error in overlapping")

        return False

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
                                    best_node.conflicts_created)
        delta = best_node.g_val[1] - best_node.g_val[0]
        g_val = (best_node.g_val[0] + 1, best_node.g_val[0] + 1 + delta)
        prev_node = SingleAgentNode(curr_pos, temp_best, g_val, self.tu_problem, curr_pos, best_node.conflicts_created)
        for g in range(prev_node.g_val[0]+1, con_time-1):
            g_val = g, g + delta
            curr_node = SingleAgentNode(curr_pos, prev_node, g_val, self.tu_problem, curr_pos, best_node.conflicts_created)
            prev_node = curr_node

        best_node.prev_node = prev_node
        best_node.g_val = g_val[0] + 1, g_val[1] + 1
        best_node.f_val = best_node.g_val
        return False


class SingleAgentNode:
    """
    The class that represents the nodes being created during the search for a single agent.
    """

    def __init__(self, current_position, prev_node, g, grid_map, goal, conflicts_created):
        self.current_position = current_position
        self.prev_node = prev_node
        self.h_val = grid_map.calc_heuristic(current_position, goal)
        self.g_val = g  # The time to reach the node.
        self.f_val = self.g_val[0] + self.h_val, self.g_val[1] + self.h_val  # heuristic val + cost of predecessor
        self.conflicts_created = conflicts_created

    """
    The function that creates all the possible vertices an agent can go to from the current node. For example,
    if a tuple from the map would be ((3,4), 1, 3) and the current time vector is t0, the agent can be at vertex (3,4)
    at times t0+1, t0+2 or t0+3.
    """

    def expand(self, agent, constraints, conflict_table, search_map, pos_cons):
        neighbors = []

        for edge_tuple in search_map.edges_and_weights[self.current_position]:
            successor_time = (self.g_val[0] + edge_tuple[1][0], self.g_val[1] + edge_tuple[1][1])
            vertex = edge_tuple[VERTEX_ID]
            if self.legal_move(agent, vertex, successor_time, constraints, pos_cons):
                if len(conflict_table) > 0:
                    confs_created = self.conflicts_created
                    confs_created = self.check_if_conflicts(agent, conflict_table, confs_created, successor_time,
                                                            vertex)
                    successor = (vertex, successor_time, confs_created)
                else:
                    successor = (vertex, successor_time, 0)

                neighbors.append(successor)

        still_time = (self.g_val[0] + STAY_STILL_COST, self.g_val[1] + STAY_STILL_COST) # Add the option of not moving.
        if self.legal_move(agent, self.current_position, still_time, constraints, pos_cons):
            if len(conflict_table) > 0:
                confs_created = self.conflicts_created
                confs_created = self.check_if_conflicts(agent, conflict_table, confs_created,
                                                        (still_time[1], still_time[1]), self.current_position)
                stay_still = (self.current_position, still_time, confs_created)
            else:
                stay_still = (self.current_position, still_time, 0)
            neighbors.append(stay_still)

        return neighbors

    @staticmethod
    def check_if_conflicts(agent, conflict_table, confs_created, successor_time, vertex):
        for tick in range(successor_time[0], successor_time[1] + 1):
            if (tick, vertex) in conflict_table:  # At least 1 agent is there.
                for other_agent in conflict_table[(tick, vertex)]:
                    if other_agent != agent:
                        confs_created += 1
        return confs_created

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
        return TimeUncertainPlan(agent, path, self.g_val)

    def create_tuple(self):
        """
        Converts a node object into a tuple.
        (vertex, time)
        """
        return self.current_position, self.g_val, self.conflicts_created

    @staticmethod
    def is_single_tick_move_illegal(agent, edge, vertex, succ_time, constraints):
        if succ_time[0] == succ_time[1]:
            return (agent, edge, succ_time[0]) in constraints or (agent, vertex, succ_time[0]) in constraints

    def is_movement_in_constraints(self, agent, vertex, edge, constraints, succ_time):
        for con in constraints:
            if agent == con[0] and \
                    ((vertex == con[1] and ConstraintAstar.overlapping((con[2], con[2]), succ_time))
                     or
                     (edge == con[1] and ConstraintAstar.overlapping((con[2], con[2]), (self.g_val[0], succ_time[1])))):
                return True

        return False

    def legal_move(self, agent, vertex, succ_time, constraints, pos_cons):  # ToDo: reduce this bottleneck

        """
        A function that checks if a certain movement is legal. First we check for vertex constraints and then edge
        constraints. The loop checks for the time range that the agent might be at the next vertex.
        vertex - The node the agent is traveling to
        time - The arrival time at vertex
        constraints - A set of constraints.

        Returns false if move is illegal. Returns true if move is fine.
        """
        edge = min(self.current_position, vertex), max(self.current_position, vertex)
        edge_occupation = self.calc_edge_time(succ_time)
        if pos_cons:  # Only check positive constraints if it exists
            for tick in range(succ_time[0], succ_time[1]+1):
                if (agent, vertex, tick) not in pos_cons:
                    return False
        for con in constraints:
            if agent == con[0] and \
                    ((vertex == con[1] and ConstraintAstar.overlapping((con[2]), succ_time))
                     or
                     (edge == con[1] and ConstraintAstar.overlapping(con[2], edge_occupation))):
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
        return self.g_val[0], succ_time[1] - 1
