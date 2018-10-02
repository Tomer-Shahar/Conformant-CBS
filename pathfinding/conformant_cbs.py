"""Implementation of a conformant variation of the Conlift-Based Search and Meta-agent Conflict based
search algorithms from

Meta-agent Conflict-Based Search for Optimal Multi-Agent Path Finding by
Guni Sharon, Roni Stern, Ariel Felner and Nathan Sturtevant

This implementation uses code from gswagner's github at: https://github.com/gswagner/mstar_public
and maze generation code from https://github.com/boppreh/maze

Basic idea is independent planning, then check for pairwise collisions.  Branch
into two separate searches, which each require that one robot avoid that
particular space-time position.  The search over the conflict tree continues
until a set of collision free paths are found.

constraints will be of the form:
((robot_ids),(disallowed states,...))
where each disallowed state will have the form (time,coord,[coord2]), where the optional second coord indicates that
this is an edge constraint.
The time for an edge constraint refers to the time at which edge traversal is finished.  Not using objects so I can
use constraints in dictionary keys. Constraints will be ordered by time.

This project adds two aspects to the planner:
1) The edges are weighted.
2) The time that it takes to pass an edge is not static, meaning that it won't necessarily be completed in 1 time-step,
but it can be anywhere between (for example) 2-5.

*Note that a weight of 1 for all edges and a time range of (1,1) is simply the regular CBS.

The goal is to find an optimal solution that guarantees that no two agents will collide.

Written by: Tomer Shahar, AI search lab, department of Software & Information Systems Engineering.
-- August 2018
"""
import time
import math
import copy
from pathfinding.path_finder import constraint_Astar

AGENT_INDEX = 0
PATH_INDEX = 1
COST_INDEX = 2
STAY_STILL_COST = 1


class ConformantCbsPlanner:
    """
    The class that represents the cbs solver. The input is a conformedMap - a map that also contains the weights
    of the edges and the time step range to complete the transfer.

    map - the map the search is conducted on

    edge_weights_and_timeSteps - a 2 dimensional array where each cell A[i,j] contains the weight of the edge from
    i to j, and the time range in the format of (i,j,(t1,t2)) where t1 is the minimal time and t2 is the maximal time.
    A null cell means the movement is impossible.

    startPositions - a list containing all the start positions.
    goalPositions - a list containing all goal positions.
    (Each position is represented as a (x,y) tuple.

    constraints - a set of constraints.

    """

    def __init__(self, conformed_map):

        self.map = conformed_map.map
        self.edges_weights_and_timeSteps = conformed_map.edges_weights_and_timeSteps
        self.paths = {}
        self.closed = {}
        self.sub_search = {}  # What is this for?
        self.startPositions = conformed_map.start_positions
        self.goalPositions = conformed_map.goal_positions
        self.constraints = set()  # ToDo: Which data structure to use for the constraints?

        self.planner = constraint_Astar(self.startPositions, self.goalPositions, conformed_map)

    def find_solution(self, time_limit=5 * 60 * 1000):
        """
        The main function - returns a solution consisting of a path for each agent and the total cost of the solution.
        This is an implementation of CBS' basic pseudo code. The main difference comes in the creation of constraints
        in the low-level search.

        root - the root node. Contains no constraints.
        open - the open list.
        time_limit - Unsurprisingly, the maximum time for this function to run.
        """
        self.start_time = time.time()
        self.planner.set_start_time(self.start_time)
        root = constraint_node()
        root.constraints = self.constraints
        root.solution = self.planner.compute_individual_paths(root, self.startPositions, self.constraints, -1)
        if not root.solution:
            return None
        root.cost = self.__compute_paths_cost(root.solution)
        print("Trivial solution found. Cost is between " + str(root.cost[0]) + " and " + str(root.cost[1]))

        open_nodes = [root]  # initialize the list with root
        nodes_expanded = 0

        while open_nodes:
            if time.time() - self.start_time > time_limit:
                raise OutOfTimeError('Ran out of time :-(')

            best_node = self.__get_best_node(open_nodes)
            nodes_expanded += 1
            if nodes_expanded % 50 == 1 and nodes_expanded > 1:
                print("Nodes expanded: " + str(nodes_expanded))
            # print("Validating node number " + str(nodes_expanded))
            new_constraints = self.__validate_solution(best_node.solution)

            if not new_constraints:  # Meaning that new_constraints is null, i.e there are no new constraints. Solved!
                cost = self.__compute_paths_cost(best_node.solution)
                print("Solution found - noded expanded: " + str(nodes_expanded))
                #  return self.__fill_in_solution(best_node.solution,), cost
                return best_node.solution, cost

            for new_con in new_constraints:  # There are only 2 new constraints, we will insert each one into "open"
                new_node = constraint_node(new_constraint=new_con, parent=best_node)
                min_time = self.__get_max_path_time(new_node.solution)
                new_node.solution[new_con[AGENT_INDEX]] = self.planner.compute_agent_path(
                    new_node, new_con[AGENT_INDEX],
                    self.startPositions[new_con[0]],
                    self.goalPositions[new_con[0]],
                    min_time)  # compute the path for a single agent.
                new_node.cost = self.__compute_paths_cost(new_node.solution)  # compute the cost

                if new_node.cost[0] < math.inf:  # If the minimum time is less than infinity..
                    self.__insert_open_node(open_nodes, new_node)
            open_nodes = sorted(open_nodes, key=lambda k: k.cost, reverse=True)

    def __compute_paths_cost(self, solution):
        """
        A function that computes the cost for each agent to reach their goal given a solution.
        Useful for the SIC heuristic.
        """
        min_cost = 0
        max_cost = 0

        for agent, path in solution.items():
            min_cost += path[2][0]
            max_cost += path[2][1]
        return min_cost, max_cost

    def __validate_solution(self, solution):
        """
        Given a solution, this function will validate it, i.e checking if any conflicts arise. If there are, returns two
        constraints: For a conflict (a1,a2,v1,v2,t1,t2), meaning agents a1 and a2 cannot both be at vertex v1 or v2 or
        the edge (v1,v2) between t1 and t2, the function will return ((a1,v1,v2,t1,t2), (a2,v1,v2,t1,t2))
        """
        max_time = self.__get_max_path_time(solution)

        filled_solution = self.__add_stationary_moves(solution, max_time)  # inserts missing time steps

        new_constraints = self.__check_vertex_conflict(filled_solution, max_time)
        if new_constraints:
            return new_constraints

        # Check for the trickier edge conflicts
        new_edge_constraints = self.__check_edge_conflict(filled_solution)
        return new_edge_constraints

    def __get_max_path_time(self, solution):
        max_time = solution[1][COST_INDEX][1]
        for agent, path in solution.items():
            if path[COST_INDEX][1] > max_time:
                max_time = path[COST_INDEX][1]
        return max_time

    @staticmethod
    def __create_movement_tuples(solution):
        """
        converts each path in solution to a tuple of ( (u,v), (t1,t2) ) where (u,v) is an edge and t1 is when the agent
        began the movement across it and t2 is when the agent completed the movement.

        For easier comparison, 'u' and 'v' will be sorted.
        """
        new_solution = {}

        for agent, path in solution.items():
            new_path = []
            for move in range(0, len(path[PATH_INDEX]) - 1):
                start_vertex = min(path[PATH_INDEX][move][1], path[PATH_INDEX][move + 1][1])
                start_time = path[PATH_INDEX][move][0][0]
                next_vertex = max(path[PATH_INDEX][move][1], path[PATH_INDEX][move + 1][1])
                finish_time = path[PATH_INDEX][move + 1][0][1]
                new_path.append(((start_vertex, next_vertex), (start_time, finish_time)))

            new_solution[agent] = (agent, new_path, solution[agent][COST_INDEX])

        return new_solution

    def __get_best_node(self, open_list):
        """
        this function returns the best node in the open list. I used a function since the best node might be something
        else depending on the search type (lowest cost vs shortest time etc.)
        """
        return open_list.pop()

    def __insert_open_node(self, open_nodes, new_node):
        """
        Simply inserts new_node into the list open_nodes and sorts the list. I used a function for modularity's
        sake - sometimes what is considered to be the "best node" is the lowest cost, while sometimes least conflicts
        or shortest time.
        """

        open_nodes.append(new_node)

    def __check_vertex_conflict(self, solution, max_time):
        """
        This function checks if at a certain time interval there might be another agent in the same vertex as the one
        given. Basically, for each agent iterate over all other agents. If another agent is at the same vertex, AND
        the time intervals overlap, extract a conflict from it.
        """

        for agent_i, path_i in solution.items():
            for move_i in path_i[PATH_INDEX]:
                interval_i = move_i[0]
                for time_step in range(interval_i[0], interval_i[1]+1):
                    for agent_j, path_j in solution.items():
                        if agent_i == agent_j:
                            continue
                        for move_j in path_j[PATH_INDEX]:
                            interval_j = move_j[0]
                            if interval_j[0] > interval_i[1]:
                                break # the min time is greater than the max time of the given interval.
                            if move_i[1] == move_j[1] and self.__overlapping(interval_i, interval_j):
                                return self.__extract_conflict \
                                    (interval_i, interval_j, move_i[1], solution, agent_i, agent_j)

        return None

    def __add_stationary_moves(self, solution, max_time):
        """
        Appends to each path the time steps where the agent waits at his goal.
        """
        new_solution = {}
        for agent, path in solution.items():
            new_path = copy.deepcopy(path[PATH_INDEX])
            last_move = path[PATH_INDEX][-1]
            path_min_time = last_move[0][0]
            path_max_time = last_move[0][1]
            if path_min_time < max_time:  # The agent is gonna stay at the end at the same position.
                for time in range(1, max_time - path_min_time + 1):
                    stationary_move = ((path_min_time + time*STAY_STILL_COST,
                                        path_max_time + time*STAY_STILL_COST), last_move[1])
                    new_path.append(stationary_move)
            new_solution[agent] = (agent, new_path, max_time)

        return new_solution

    def __fill_in_solution(self, solution):
        """
        Receives a solution that contains only the vertices of the paths, and fills in the spots where the agent is
        travelling an edge. i.e if a path is [(0,time=0),(4,time=3)], we know that the agent was on the edge (0,4)
        at time 1 and 2 -> filled_path = [(0,time=0),((0,4),time=1),((0,4),time=2),(4,time=3)]
        """
        max_time = len(solution[1][PATH_INDEX]) - 1
        for agent, path in solution.items():
            if path[COST_INDEX] > max_time:
                max_time = path[COST_INDEX]

        for agent, path in solution.items():
            filled_path = []
            for i in range(0, len(path[PATH_INDEX]) - 1):
                curr_node = path[PATH_INDEX][i]
                next_node = path[PATH_INDEX][i + 1]
                if curr_node[0] + 1 == next_node[0]:  # i.e they are sequential
                    filled_path.append(curr_node)
                else:
                    filled_path.append(curr_node)
                    for time in range(curr_node[0] + 1, next_node[0]):
                        edge_move = (time, (curr_node[1], next_node[1]))
                        filled_path.append(edge_move)
            last_move = path[PATH_INDEX][-1]
            filled_path.append(last_move)
            if last_move[0] < max_time:  # The agent is gonna stay at the end at the same position.
                curr_time = len(filled_path)
                for time in range(curr_time, max_time + 1):
                    stationary_move = (time, last_move[1])
                    filled_path.append(stationary_move)
            new_path = (agent, filled_path, max_time)
            solution[agent] = new_path

        return solution

    def __extract_conflict(self, interval_i, interval_j, vertex, solution, agent_i, agent_j):
        """
        Helper function. We know that at that time interval there is some conflict between two given agents.
        """

        constraint_interval = max(interval_i[0], interval_j[0]), min(interval_i[1], interval_j[1])
        constraint_1 = (agent_i, constraint_interval, vertex)
        constraint_2 = (agent_j, constraint_interval, vertex)

        return constraint_1, constraint_2

    def __check_edge_conflict(self, filled_solution):
        """
        Checks for edge conflicts by creating a path represented by tuples, where each tuple contains the edge being
        traversed and the (start time, end time). All of the edges traversed are inserted into a dictionary mapping
        edges to times being traversed and the agent traversing it. If the traversal times overlap, there is a conflict
        and it will be returned.
        """

        tuple_solution = self.__create_movement_tuples(filled_solution)
        positions = {}

        for agent, path in tuple_solution.items():
            for move in path[PATH_INDEX]:
                if move[0] not in positions:
                    positions[move[0]] = set()
                    positions[move[0]].add((agent, move[1]))
                else:
                    for traversal in positions[move[0]]:
                        if traversal[0] != agent and self.__overlapping(move[1], traversal[1]):
                            return self.__extract_edge_conflict(traversal[0], agent, traversal[1], move[1], move[0])


        return None

    def __extract_edge_conflict(self, agent_1, agent_2, time_1, time_2, edge):
        """
        We know there's a conflict at some edge, and agent1 cannot begin traversing it at time 1 and agent 2 cannot
        begin traversing it at time 2.

        returns the appropriate constraints.
        """
        """path_1 = solution[agent_1][PATH_INDEX]
        path_2 = solution[agent_2][PATH_INDEX]
        con_1, con_2 = None, None

        for move in range(0, len(path_1) - 1):
            if path_1[move][0] == time_1:
                edge = (path_1[move][1], path_1[move + 1][1])
                con_1 = (agent_1, time_1, edge)
                break
        for move in range(0, len(path_1) - 1):
            if path_2[move][0] == time_2:
                edge = (path_2[move][1], path_2[move + 1][1])
                con_2 = (agent_2, time_2, edge)
                break

        return con_1, con_2"""

        min_constraint_time = max(time_1[0], time_2[0])
        max_constraint_time = min(time_1[1], time_2[1])

        con_1 = agent_1, (min_constraint_time, max_constraint_time), edge
        con_2 = agent_2, (min_constraint_time, max_constraint_time), edge

        return con_1, con_2



    def __overlapping(self, time_1, time_2):
        """
        Helper function - returns true if the time intervals in 'time_1' and 'time_2' overlap.

        1: A<-- a<-->b -->B
        2: a<-- A -->b<---B>
        3: A<-- a<--- B --->b
        4: a<-- A<--->B --->b
        5: A<-->B
           a<-->b
        ===> A < a < B or a < A < b
        """

        if (time_1[0] <= time_2[0] <= time_1[1]) or \
                (time_2[0] <= time_1[0] <= time_2[1]):
            return True

        return False

    def get_min_max_cost(self, prev_vertex, vertex):
        """
        Returns the minimum and maximum time to traverse a given edge (source vertex and end vertex)
        """
        if prev_vertex == vertex:
            return 0, 0  # Cost for standing still
        for edge in self.edges_weights_and_timeSteps[prev_vertex]:
            if edge[0] == vertex:
                return edge[1], edge[2]


class constraint_node:
    """
    The class that represents a node in the CT. Contains the current path for each agent and the contraint added to
    this node. We do not need to save all of the constraints, as they can be extrapolated from the parent nodes by
    traversing the path from the current node to the root.

    solution - a dictionary mapping agent ID to the path.
    parent - a pointer to the previous node.
    cost - the cost of the solution. i.e the sum of the costs of each path.
    """

    def __init__(self, new_constraint=None, parent=None):

        if parent:
            self.constraints = self.__append_constraints(parent.constraints, new_constraint)
            self.solution = copy.deepcopy(parent.solution)
        else:
            self.constraints = new_constraint
            self.solution = None

        self.parent = parent
        self.cost = math.inf  # Default value higher than any possible int

    def __append_constraints(self, parent_constraints, new_constraint):
        """
        Adds new constraints to the parent constraints.
        """
        new_constraints = copy.deepcopy(parent_constraints)
        for time_interval in range(new_constraint[1][0], new_constraint[1][1]+1):
            new_constraints.add((new_constraint[0], time_interval, new_constraint[2]))
        return new_constraints


class OutOfTimeError(Exception):
    def __init__(self, value=None):
        self.value = value

    def __str__(self):
        return repr(self.value)
