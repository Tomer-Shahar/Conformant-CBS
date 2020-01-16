"""Implementation of a conformant variation of the Conlift-Based Search and Meta-agent Conflict based
search algorithms from

Meta-agent Conflict-Based Search for Optimal Multi-Agent Path Finding by
Guni Sharon, Roni Stern, Ariel Felner and Nathan Sturtevant

This implementation uses maze generation code from https://github.com/boppreh/maze

Basic idea is independent planning, then check for pairwise collisions.  Branch
into two separate searches, which each require that one robot avoids that
particular space-time position.  The search over the conflict tree continues
until a set of collision free paths are found.

constraints will be of the form:
(agent_ID, conflict_node, time) <--- vertex constraint
(agent_ID, (prev_node, conflict_node), time) <--- edge constraint

where each disallowed state will have the form (agent, node/edge, time)
The time for an edge constraint refers to the time at which the agent occupies the edge. Not using objects so I can
use constraints in dictionary keys.

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

from pathfinding.planners.constraint_A_star import ConstraintAstar as Cas
from pathfinding.planners.utils.constraint_node import ConstraintNode
from pathfinding.planners.utils.time_error import OutOfTimeError
from pathfinding.planners.utils.time_uncertainty_solution import TimeUncertainSolution
from pathfinding.planners.utils.custom_heap import OpenListHeap
STAY_STILL_COST = 1


class CBSTUPlanner:
    """
    The class that represents the cbs solver. The input is a conformedMap - a map that also contains the weights
    of the edges and the time step range to complete the transfer.

    constraints - a set of constraints.

    """

    def __init__(self, tu_problem):

        self.tu_problem = tu_problem
        self.final_constraints = None
        self.start_time = 0
        self.planner = Cas(tu_problem)
        self.curr_time = (0, 0)
        self.root = None

    def find_solution(self, min_best_case=False, time_limit=60, soc=True, use_cat=True, existing_cons=None,
                      curr_time=(0, 0)):
        """
        The main function - returns a solution consisting of a path for each agent and the total cost of the solution.
        This is an implementation of CBS' basic pseudo code. The main difference comes in the creation of constraints
        in the low-level search.

        root - the root node. Contains no constraints.
        open - the open list.
        sum_of_costs - How to count the solution cost. If it's true, the solution cost is the  sum of all individual
        paths. Otherwise the cost is the longest path in the solution.
        time_limit - Unsurprisingly, the maximum time for this function to run (in seconds)
        """
        self.start_time = time.time()
        self.curr_time = curr_time
        self.root = ConstraintNode(parent=None, use_cat=use_cat)
        if existing_cons:
            self.root.constraints = self.root.constraints | existing_cons
        self.compute_all_paths_and_conflicts(self.root, use_cat, min_best_case)

        if not self.root.solution:
            print("No solution for cbs root node")
            return None
        self.root.solution.compute_solution_cost()
        open_nodes = OpenListHeap()
        self.__insert_open_node(open_nodes, self.root, min_best_case)  # initialize the list with root
        closed_nodes = set()
        while open_nodes:
            if time.time() - self.start_time > time_limit:
                print(f'Number of CT nodes expanded: {len(closed_nodes)}')
                print(f'Number of CT nodes in open: {len(open_nodes.internal_heap)}')
                raise OutOfTimeError('Ran out of time :-(')

            best_node = self.__get_best_node(open_nodes)
            self.__insert_into_closed_list(closed_nodes, best_node)
            new_constraints = self.validate_solution(best_node)

            if not new_constraints:  # Meaning that new_constraints is null, i.e there are no new constraints. Solved!
                print(f'Number of CT nodes expanded: {len(closed_nodes)}')
                print(f'Number of CT nodes in open: {len(open_nodes.internal_heap)}')
                return self.create_solution(best_node, len(closed_nodes), soc)

            #print(f'Conflicting agents: {best_node.conflicting_agents} at location {next(iter(new_constraints[0]))[1]} and time {next(iter(new_constraints[0]))[2]} ')
            for new_con_set in new_constraints:  # There are only 2 new constraints, we will insert each one into "open"
                new_node = ConstraintNode(new_constraints=new_con_set, parent=best_node)
                if frozenset(new_node.constraints) in closed_nodes:
                    print('Already tried')
                    continue
                agent = next(iter(new_con_set))[0]  # Ugly line to extract agent index.
                time_passed = time.time() - self.start_time
                new_plan = self.planner.compute_agent_path(
                    new_node.constraints, agent,
                    self.tu_problem.start_positions[agent],
                    self.tu_problem.goal_positions[agent],
                    new_node.conflict_table,
                    min_best_case, time_limit=time_limit - time_passed,
                    curr_time=self.curr_time)  # compute the path for a single agent.
                if new_plan.path:
                    new_node.update_solution(new_plan, use_cat)
                    new_node.solution.compute_solution_cost(soc)  # compute the cost

                    if new_node.solution.cost[0] < math.inf:  # If the minimum time is less than infinity..
                        self.__insert_open_node(open_nodes, new_node, min_best_case)
        print("Empty open list - No Solution")
        return TimeUncertainSolution.empty_solution()

    def create_solution(self, best_node: ConstraintNode, nodes_expanded: int, soc: bool):
        """
        Compiles together all necessary parameters for the final solution that will be returned.
        :param best_node: The node containing the solution
        :param nodes_expanded: Number of nodes expanded
        :param soc: True if we are computing sum of costs, False otherwise.
        :return: The proper Time Uncertain Solution.
        """
        best_node.solution.time_to_solve = time.time() - self.start_time
        best_node.solution.compute_solution_cost(soc)
        best_node.solution.nodes_expanded = nodes_expanded
        self.final_constraints = best_node.constraints
        best_node.solution.constraints = set(best_node.constraints)
        best_node.solution.sic = self.root.solution.cost
        return best_node.solution

    def create_root(self, min_best_case, use_cat=False):
        self.root = ConstraintNode(use_cat=use_cat)
        self.compute_all_paths_and_conflicts(self.root, use_cat, min_best_case)
        #  print("Computed root node")
        self.root.parent = ConstraintNode()
        if not self.root.solution:
            print("No solution for cbs root node")
            return None
        self.root.solution.compute_solution_cost()

        return self.root

    def validate_solution(self, node):
        """
        Given a solution, this function will validate it.
        i.e checking if any conflicts arise. If there are, returns two constraints: For a conflict (a1,a2,v1,v2,t1,t2),
        meaning agents a1 and a2 cannot both be at vertex v1 or v2 or the edge (v1,v2) between t1 and t2, the function
        will return ((a1,v1,v2,t1,t2), (a2,v1,v2,t1,t2))
        """
        # node.solution.add_stationary_moves()  # inserts missing time steps
        node.solution.create_movement_tuples()

        constraints = self.__check_previously_conflicting_agents(node.solution, node.conflicting_agents)
        if constraints:
            return constraints

        new_constraints = self.__check_vertex_conflict(node)
        if new_constraints:
            node.add_conflicting_agents(new_constraints[0], new_constraints[1])
            return new_constraints

        # Check for edge conflicts
        new_edge_swap_constraints = self.check_edge_swap_conflict(node.solution, node)
        if new_edge_swap_constraints:
            node.add_conflicting_agents(new_edge_swap_constraints[0], new_edge_swap_constraints[1])
            return new_edge_swap_constraints

        return None

    def __check_vertex_conflict(self, node):  # ToDo: reduce runtime of this
        """
        This function checks if at a certain time interval there might be another agent in the same vertex as the one
        given. Basically, we assume that for most cases, each node is visited by a single agent. We do not want to
        waste time for those nodes. The function maintains a dictionary that maps each node to the agent and time step
        it was there.
        """

        visited_nodes = {}  # A dictionary containing all the nodes visited
        for agent_i in node.solution.paths.keys():
            plan_i = node.solution.paths[agent_i]
            for move_i in plan_i.path:
                interval_i = move_i[0]
                if not move_i[1] in visited_nodes:  # First time an agent has visited this node
                    visited_nodes[move_i[1]] = {(agent_i, interval_i)}  # Add the interval to the set.
                    continue
                else:  # Some other agent HAS been to this node..
                    conf = self.__check_conf_in_visited_node(visited_nodes, move_i, interval_i, agent_i)
                    if conf:
                        return conf
                    visited_nodes[move_i[1]].add((agent_i, interval_i))  # No conflict, add to vertex set.
        return None

    def __check_conf_in_visited_node(self, visited_nodes, move_i, interval_i, agent_i):
        for occupancy in visited_nodes[move_i[1]]:  # Iterate over the times agents have been at this node
            if occupancy[0] != agent_i and Cas.overlapping(interval_i, occupancy[1]):  # There is a conflict.
                return self.extract_vertex_conflict_constraints(
                    interval_i, occupancy[1], move_i[1], agent_i, occupancy[0])
        return None

    @staticmethod
    def __can_skip_conflict_check(agent_i, agent_j, prev_agents):
        """
        Used when validating a solution. If agent i is equal to agent j, there is no need to check for a conflict.
        """
        if agent_i == agent_j:
            return True

        if prev_agents:
            return (agent_i == prev_agents[0] and agent_j == prev_agents[1]) or \
                   (agent_j == prev_agents[0] and agent_i == prev_agents[1])

    def extract_vertex_conflict_constraints(self, interval_i, interval_j, vertex, agent_i, agent_j):
        """
        Helper function. We know that at that time interval there is some conflict between two given agents on the
        given vertex. This function creates the proper tuple of constraints.

        Interval example:     ______________
                        _____|_\_\_\_\_|<---- The time we want to isolate - Maximal time of overlap.

            Constraints will be of the form (agent, conflict_node, time)
        """
        t = self.__pick_times_to_constrain(interval_i, interval_j)
        return {(agent_i, vertex, t)}, {(agent_j, vertex, t)}

    def check_edge_swap_conflict(self, filled_solution, node):
        """ Checks for edge conflicts by creating a path represented by tuples, where each tuple contains the edge being
        traversed and the (start time, end time). All of the edges traversed are inserted into a dictionary mapping
        edges to times being traversed and the agent traversing it. If the traversal times overlap, there is a conflict
        and it will be returned. Note that for a conflict to arise the time intervals must be "strongly overlapping",
        i.e (16,17) and (17,18) don't count, however (17,18) and (17,18) do conflict. This is because in an edge
        conflict, if it was a swap type conflict it would be discovered earlier during the vertex conflict check.

        An edge constraint for agent 'a' on edge 'e' at time 't' means that agent 'a' cannot BEGIN to traverse 'e' at
        time 't'.
        """

        # A dictionary containing the different edges being traversed in the solution and the times and agents
        # traversing them.
        positions = {}

        for agent, path in node.solution.tuple_solution.items():
            for move in path:
                edge = move[1]
                if edge not in positions:
                    positions[edge] = set()
                if move[0][1] - move[0][0] > 1:  # Edge weight is more than 1
                    occ_time = move[0][0], move[0][1] - 1
                    positions[edge].add((agent, (move[0][0], move[0][1] - 1), move[2]))
                    for traversal in positions[edge]:
                        if traversal[0] != agent:
                            if traversal[2] == move[2] and self.strong_overlapping(occ_time, traversal[1]):
                                return self.get_edge_constraint(traversal[0], agent, traversal[1], occ_time, edge)
                            elif traversal[2] != move[2] and Cas.overlapping(occ_time, traversal[1]):
                                return self.get_edge_constraint(traversal[0], agent, traversal[1], occ_time, edge)
                else:
                    # Agent begins to travel at move[0][0] and arrives at move[0][1]
                    positions[edge].add((agent, move[0], move[2]))
                    for traversal in positions[edge]:
                        if traversal[0] != agent and self.strong_overlapping(move[0], traversal[1]):
                            return self.get_edge_constraint(traversal[0], agent, traversal[1], move[0], edge)
        return None

    def get_edge_constraint(self, agent_i, agent_j, interval_i, interval_j, conflict_edge):
        """
        We know there's a conflict at some edge, and agent1 cannot BEGIN traversing it at time 1 and agent 2 cannot
        begin traversing it at time 2. Time 1 and time 2 must be computed through the given intervals and the time to
        traverse the conflicting edge.

        returns the appropriate set of constraints: (agent_ID, (prev_node, conflict_node), time)

        Note: In the case of an edge with a traversal time of 1, theoretically, the agent does not spend any time on it
        and arrives at the end of the edge instantly at 1 time tick. We must address these kind of edges differently.
        :param agent_i: First agent
        :param agent_j: Second agent
        :param interval_i: The interval between BEGINNING of movement until ARRIVAL at next vertex for first agent
        :param interval_j: Same, but for second agent
        :param conflict_edge: The edge where the conflict occurs
        :return: The appropriate constraints
        """
        edge_min, edge_max = 0, 0  # The edge traversal times
        agent_i_constraints = set()
        agent_j_constraints = set()
        start_vertex = conflict_edge[0]
        for movement in self.tu_problem.edges_and_weights[start_vertex]:
            end_vertex = movement[0]
            travel_time = movement[1]
            if end_vertex == conflict_edge[1]:
                edge_min, edge_max = travel_time[0], travel_time[1]
                break

        t = self.__pick_times_to_constrain(interval_i, interval_j)
        if edge_min == edge_max == 1:
            return {(agent_i, conflict_edge, t)}, {(agent_j, conflict_edge, t)}

        agent_i_constraints.add((agent_i, conflict_edge, t))
        agent_j_constraints.add((agent_j, conflict_edge, t))

        return agent_i_constraints, agent_j_constraints

    def compute_all_paths_and_conflicts(self, root, use_cat, min_best_case=False):
        """
        A function that computes the paths for all agents, i.e a solution. Used for the root node before any constraints
        are added.
        """

        for agent_id, agent_start in self.tu_problem.start_positions.items():
            agent_plan = self.planner.compute_agent_path(
                root.constraints, agent_id, agent_start, self.tu_problem.goal_positions[agent_id], root.conflict_table,
                min_best_case=min_best_case, curr_time=self.curr_time)
            if agent_plan.path:  # Solution found
                root.solution.paths[agent_id] = agent_plan
            else:  # No solution for a particular agent
                print("No solution for agent number " + str(agent_id))
                root.solution = None
        root.solution.add_stationary_moves()  # inserts missing time steps
        self.validate_solution(root)
        if use_cat:
            for agent, conf_plan in root.solution.paths.items():
                for move in conf_plan.path:
                    if (move[1]) not in root.conflict_table:
                        root.conflict_table[(move[1])] = []
                    root.conflict_table[(move[1])].append((agent, move[0]))

    @staticmethod
    def __get_best_node(open_list):
        """
        this function returns the best node in the open list. I used a function since the best node might be something
        else depending on the search type (lowest cost vs shortest time etc.)
        """
        return open_list.pop()

    @staticmethod
    def __insert_open_node(open_nodes, new_node, min_best_case):
        """
        Simply inserts new_node into the list open_nodes and sorts the list. I used a function for modularity's
        sake - sometimes what is considered to be the "best node" is the lowest cost, while sometimes least conflicts
        or shortest time.
        """
        if min_best_case:
            open_nodes.push(new_node, new_node.solution.cost[0], new_node.solution.cost[1], len(new_node.conflict_table))
        else:
            open_nodes.push(new_node, new_node.solution.cost[1], new_node.solution.cost[0], len(new_node.conflict_table))

    @staticmethod
    def strong_overlapping(time_1, time_2):
        """ Returns true if the time intervals in 'time_1' and 'time_2' overlap strongly
        A strong overlap means that the times are fully overlapping, not just a singular common tick at the end.
        Basically change the '<=' to '<'

        1: A<-- a<-->b -->B
        2: a<-- A -->b<---B>
        3: A<-- a<--- B --->b
        4: a<-- A<--->B --->b
        ===> A < a < B or a < A < b
        """

        if (time_1[0] <= time_2[0] < time_1[1]) or (time_2[0] <= time_1[0] < time_2[1]):
            return True

        return False

    @staticmethod
    def __insert_into_closed_list(closed_nodes, new_node):

        closed_nodes.add(frozenset(new_node.constraints))

    @staticmethod
    def __pick_times_to_constrain(interval_i, interval_j, t='max'):
        """
        Returns the time ticks to constrain in the conflict interval. For example if the intervals are (4,17) and (8,22),
        the conflicting interval will be (8,17). Note that the format for constraining e.g. time tick 4 is (4, 4). This
        is so we can introduce range constraints with ease later.
        :param interval_i: Agent i's interval for occupying a resource
        :param interval_j: Agent j's interval for occupying a resource
        :param t: what time ticks to constrain. Can be max or min. Any other value will choose the average time.
        :return: The time tick to constraints. Currently returns the maximum time of the CONFLICT interval.
        """
        # Choose the minimum between the max times in the conflict interval
        conf_interval = max(interval_i[0], interval_j[0]), min(interval_i[1], interval_j[1])
        if t == 'max':
            return conf_interval[1], conf_interval[1]
        elif t == 'min':
            return conf_interval[0], conf_interval[0]
        else:
            return int((conf_interval[0] + conf_interval[1]) / 2), int((conf_interval[0] + conf_interval[1]) / 2)

    def __check_previously_conflicting_agents(self, solution, prev_agents):
        """
        We check if the agents that previously collided still collide. Empirically, this is usually the case
        and is one of the drawbacks of CBS. This attempts to speed it up.
        :param solution: The given solution of the node
        :param prev_agents: The previously colliding agents
        :return: The proper constraint if one exists, otherwise "None"
        """
        if not prev_agents:
            return None

        agent_i = prev_agents[0]
        agent_j = prev_agents[1]

        visited_nodes = {}  # A dictionary containing all the nodes visited
        for agent in [agent_i, agent_j]:
            for move in solution.paths[agent].path:
                interval = move[0]
                if not move[1] in visited_nodes:  # First time an agent has visited this node
                    visited_nodes[move[1]] = {(agent, interval)}  # Add the interval to the set.
                    continue
                else:  # Some other agent HAS been to this node..
                    conf = self.__check_conf_in_visited_node(visited_nodes, move, interval, agent)
                    if conf:
                        return conf
                    visited_nodes[move[1]].add((agent, interval))  # No conflict, add to vertex set.

        positions = {}

        for agent in [agent_i, agent_j]:
            path = solution.tuple_solution[agent]
            for move in path:
                edge = move[1]
                if edge not in positions:
                    positions[edge] = set()
                if move[0][1] - move[0][0] > 1:  # Edge weight is more than 1
                    occ_time = move[0][0], move[0][1] - 1
                    positions[edge].add((agent, (move[0][0], move[0][1] - 1), move[2]))
                    for traversal in positions[edge]:
                        if traversal[0] != agent:
                            if traversal[2] == move[2] and self.strong_overlapping(occ_time, traversal[1]):
                                return self.get_edge_constraint(traversal[0], agent, traversal[1], occ_time, edge)
                            elif traversal[2] != move[2] and Cas.overlapping(occ_time, traversal[1]):
                                return self.get_edge_constraint(traversal[0], agent, traversal[1], occ_time, edge)
                else:
                    # Agent begins to travel at move[0][0] and arrives at move[0][1]
                    positions[edge].add((agent, move[0], move[2]))
                    for traversal in positions[edge]:
                        if traversal[0] != agent and self.strong_overlapping(move[0], traversal[1]):
                            return self.get_edge_constraint(traversal[0], agent, traversal[1], move[0], edge)
        return None
