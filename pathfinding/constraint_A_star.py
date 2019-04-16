"""
A class that represents an algorithm for solving the path for multiple agents / single agent in accordance with
a group of constraints.

Currently a naive implementation of A*
"""

from pathfinding.custom_heap import OpenListHeap
from pathfinding.time_error import OutOfTimeError
import math
import networkx
import time

# The positions of each parameter in the tuple receives from map.edges
VERTEX_ID = 0
STAY_STILL_COST = 1


class ConstraintAstar:

    def __init__(self, ccbs_map):
        self.grid_map = ccbs_map
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

    def compute_agent_path(self, constraints, agent, start_pos, goal_pos, conf_table, min_best_case=True,
                           time_limit=200000):

        self.open_list = OpenListHeap()
        self.open_dict = {}  # A dictionary mapping node tuple to the actual node. Used for faster access..
        self.closed_set = set()
        self.agent = agent
        start_time = time.time()
        start_node = SingleAgentNode(start_pos, None, (0, 0), self.grid_map, goal_pos, conflicts_created=0)
        self.__add_node_to_open(start_node)

        while len(self.open_list.internal_heap) > 0:
            if time.time() - start_time > time_limit:
                raise OutOfTimeError('Ran out of time :-(')
            best_node = self.open_list.pop()  # removes from open_list
            self.__remove_node_from_open(best_node)

            if best_node.current_position == goal_pos and self.__can_stay_still(agent, best_node, constraints):
                return best_node.calc_path(agent)

            successors = best_node.expand(agent, constraints, conf_table, self.grid_map)
            for neighbor in successors:
                if neighbor in self.closed_set:
                    continue
                g_val = neighbor[1]
                if neighbor not in self.open_dict:
                    neighbor_node = SingleAgentNode(neighbor[0], best_node, neighbor[1], self.grid_map, goal_pos,
                                                    neighbor[2])
                    self.__add_node_to_open(neighbor_node)
                else:
                    neighbor_node = self.open_dict[neighbor]

                    if min_best_case and g_val[0] >= neighbor_node.g_val[0]:
                        continue  # No need to update node. Continue iterating successors
                    elif not min_best_case and g_val[1] >= neighbor_node.g_val[1]:
                        continue  # No need to update node. Continue iterating successors
                    # print("Found a faster path for node " + neighbor_node.current_position)
                # ToDO: Is the open list updated?
                self.__update_node(neighbor_node, best_node, g_val, goal_pos, self.grid_map)
        print("No solution?")
        return agent, None, math.inf  # no solution

    def __remove_node_from_open(self, node):
        node_tuple = node.create_tuple()
        self.open_dict.pop(node_tuple, None)
        self.closed_set.add(node_tuple)

    def __add_node_to_open(self, node):
        self.open_list.push(node, node.f_val[0], node.h_val, node.conflicts_created)
        key_tuple = node.create_tuple()
        self.open_dict[key_tuple] = node

        return node

    def __update_node(self, neighbor_node, prev_node, g_val, goal_pos, grid_map):
        neighbor_node.prev_node = prev_node
        neighbor_node.g_val = g_val
        neighbor_node.h_val = grid_map.calc_heuristic(neighbor_node.current_position, goal_pos)
        neighbor_node.f_val = g_val[0] + neighbor_node.h_val, g_val[1] + neighbor_node.h_val
        #  self.open_list.heapify() ToDO: Use heapify??

    def __can_stay_still(self, agent, best_node, constraints):
        """
        A function that verifies that the agent has reached the goal at an appropriate time. Useful when an agent
        reaches the goal in, for example, 5-8 time units however in the solution it takes another agent 10-15 time
        units.
        Therefor we must verify what happens if this agent stands still all this time (there might be a conflict!)
        """
        can_stay = True
        agent_cons = set()
        for con in constraints:  # ToDo: Iterate over all constraints or send the max time to stand?
            if con[0] == agent and con[1] == best_node.current_position and best_node.g_val[0] <= con[2]:
                agent_cons.add(con)
                can_stay = False

        return can_stay

    def dijkstra_solution(self, source_vertex):
        """
        A function that calculates, using Dijkstra's algorithm, the distance from each point in the map to the given
        goal. This will be used as a perfect heuristic. It receives as input the goal position and returns a dictionary
        mapping each coordinate to the distance from it to the goal.

        For now we will use the minimum time taken to pass an edge as the weight, in order to keep the heuristic
        admissible.
        """

        graph = networkx.Graph()
        for vertex, edges in self.grid_map.edges_and_weights.items():
            for edge in edges:
                graph.add_edge(vertex, edge[0], weight=edge[1][0])
        return networkx.single_source_dijkstra_path_length(graph, source_vertex)

    def add_stay_still_node(self, goal_node, agent_cons):
        """
        Adds to the open list the option of just staying at the current node.
        Iterate through all time ticks
        :param agent_cons: All constraints for the goal node.
        :param goal_node: The node that is located at the goal position but might not be able to stay there.
        """
        curr_node = goal_node
        min_time = min(con[2] for con in agent_cons)
        for tick in range(goal_node.g_val[0], min_time):
            new_time = curr_node.g_val[0] + STAY_STILL_COST, curr_node.g_val[1] + STAY_STILL_COST
            goal = goal_node.current_position
            if (self.agent, goal, new_time[0]) not in agent_cons:  # Safe to add!
                new_node = SingleAgentNode(goal, curr_node, new_time, self.grid_map, goal, conflicts_created=0)
                curr_node = new_node
            else:  # Curr node was the best we got.
                if not curr_node.create_tuple() in self.closed_set:
                    self.__add_node_to_open(curr_node)
                    return
        if not curr_node.create_tuple() in self.closed_set:
            self.__add_node_to_open(curr_node)

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

        if (time_1[0] <= time_2[0] <= time_1[1]) or (time_2[0] <= time_1[0] <= time_2[1]):
            return True

        return False


class SingleAgentNode:
    """
    The class that represents the nodes being created during the search for a single agent.
    """

    def __init__(self, current_position, prev_node, time_span, grid_map, goal, conflicts_created):
        self.current_position = current_position
        self.prev_node = prev_node
        self.h_val = grid_map.calc_heuristic(current_position, goal)
        self.g_val = time_span  # The time to reach the node.
        self.f_val = self.g_val[0] + self.h_val, self.g_val[1] + self.h_val  # heuristic val + cost of predecessor
        self.conflicts_created = conflicts_created

    """
    The function that creates all the possible vertices an agent can go to from the current node. For example,
    if a tuple from the map would be ((3,4), 1, 3) and the current time vector is t0, the agent can be at vertex (3,4)
    at times t0+1, t0+2 or t0+3.
    """

    def expand(self, agent, constraints, conflict_table, search_map):
        neighbors = []

        for edge_tuple in search_map.edges_and_weights[self.current_position]:
            successor_time = (self.g_val[0] + edge_tuple[1][0], self.g_val[1] + edge_tuple[1][1])
            vertex = edge_tuple[VERTEX_ID]
            if self.legal_move(agent, vertex, successor_time, constraints):
                confs_created = self.conflicts_created
                for tick in successor_time:
                    for other_agent in conflict_table:
                        if other_agent == agent:
                            continue
                        if (tick, vertex) in conflict_table[other_agent]:
                            confs_created += 1
                successor = (vertex, successor_time, confs_created)
                neighbors.append(successor)

        still_time = (self.g_val[0] + STAY_STILL_COST, self.g_val[1] + STAY_STILL_COST)
        if self.legal_move(agent, self.current_position, still_time, constraints):  # Add the option of not moving.
            confs_created = self.conflicts_created
            for other_agent in conflict_table:
                if other_agent == agent:
                    continue
                if still_time[1] in conflict_table[other_agent]:
                    confs_created += 1
            stay_still = (self.current_position, still_time, confs_created)
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

    def legal_move(self, agent, vertex, time, constraints):

        """
        A function that checks if a certain movement is legal. First we check for vertex constraints and then edge
        constraints. The first loop checks for the time range that the agent might be at the next vertex.
        """
        succ_min_time = time[0]
        succ_max_time = time[1]
        curr_min_time = self.g_val[0]
        edge = min(self.current_position, vertex), max(self.current_position, vertex)

        for con in constraints:
            if agent == con[0] and \
                    ((vertex == con[1] and ConstraintAstar.overlapping((con[2], con[2]),
                                                                       (succ_min_time, succ_max_time)))
                     or (edge == con[1] and ConstraintAstar.overlapping((con[2], con[2]),
                                                                        (curr_min_time, succ_max_time)))):
                return False

        # for con in constraints:
        #    if agent == con[0] and edge == con[1] and \
        #            ConstraintAstar.overlapping((con[2], con[2]), (curr_min_time, succ_max_time)):
        #        return False, -1

        if succ_min_time == succ_max_time:  # instantaneous traversal
            if (agent, edge, succ_max_time) in constraints:
                return False, -1

        return True
