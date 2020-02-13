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

from pathfinding.planners.constraint_A_star import ConstraintAstar as Cas
from pathfinding.planners.utils.constraint_node import ConstraintNode as Cn
from pathfinding.planners.utils.time_error import OutOfTimeError
from pathfinding.planners.utils.time_uncertainty_solution import TimeUncertaintySolution
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
        self.open_nodes = OpenListHeap()
        self.closed_nodes = set()
        self.min_best_case = False
        self.soc = True
        self.use_cat = True

    def find_solution(self, min_best_case=False, time_lim=60, soc=True, use_cat=True, existing_cons=None,
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
        self.min_best_case = min_best_case
        self.soc = soc
        self.use_cat = use_cat
        self.root = self.create_root(use_cat, soc, existing_cons)
        self.__insert_open_node(self.root)  # initialize the list with root

        if not self.root.sol:
            return None

        while self.open_nodes:
            if time.time() - self.start_time > time_lim:
                print(f'Number of CT nodes expanded: {len(self.closed_nodes)}')
                print(f'Number of CT nodes in open: {len(self.open_nodes.internal_heap)}')
                raise OutOfTimeError('Ran out of time :-(')

            best_node = self.open_nodes.pop()
            self.__insert_into_closed_list(best_node)
            if best_node.conflicts['count'] == 0:  # Meaning that there are no conflicts.
                return self.create_solution(best_node)

            new_constraints, is_cardinal = self.find_cardinal_conflict(best_node, time_lim)

            if not is_cardinal and self.find_bypass(best_node, new_constraints, time_lim):
                continue
            for new_con_set in new_constraints:  # There are only 2 new constraints, we will insert each one into "open"
                new_cn = self.__generate_constraint_node(new_con_set, best_node, time_lim)
                if new_cn:  # If a solution wasn't found, it'll be null.
                    self.__insert_open_node(new_cn)

        print("Empty open list - No Solution")
        return TimeUncertaintySolution.empty_solution()

    def create_solution(self, best_node):
        """
        Compiles together all necessary parameters for the final solution that will be returned.
        :param best_node: The node containing the solution
        :return: The proper Time Uncertain Solution.
        """
        # print(f'Number of CT nodes expanded: {len(self.closed_nodes)}')
        # print(f'Number of CT nodes in open: {len(self.open_nodes.internal_heap)}')
        best_node.solution.time_to_solve = time.time() - self.start_time
        best_node.solution.compute_solution_cost(self.soc)
        best_node.solution.nodes_expanded = len(self.closed_nodes)
        self.final_constraints = best_node.constraints
        best_node.solution.constraints = set(best_node.constraints)
        best_node.solution.sic = self.root.solution.cost
        return best_node.solution

    def create_root(self, use_cat=False, sum_of_costs=True, existing_cons=None):
        self.root = Cn(use_cat=use_cat)
        self.compute_all_paths_and_conflicts(self.root, use_cat)
        if not self.root.sol:
            print("No solution for cbs root node")
            return None
        self.root.sol.compute_solution_cost(sum_of_costs)
        if existing_cons:
            self.root.constraints = Cn.append_constraints(self.root.constraints, existing_cons)
        return self.root

    def find_cardinal_conflict(self, node, time_lim):
        """
        Tries to find cardinal, semi-cardinal or non-cardinal conflicts in the given node, in that order of priority.
        If the conflict found is cardinal, the function will return a tuple of <constraint, True> otherwise it will
        return <constraint, False> in order to assess later on if it's cardinal or not.
        :param time_lim: The time limit to pass over. This function can be quite long
        :param node: The node who we're searching a solution for.
        :return: A tuple of the constraint and a boolean indicating if it's cardinal or not (for the bypass)
        """
        constraint = None
        start = time.time()
        for loc, conflicts in node.conflicts.items():
            if loc == 'count':
                continue
            if time.time() - start > time_lim:
                raise OutOfTimeError('Ran out of time when searching for cardinal conflicts')
            for conf in conflicts:
                constraints_to_check, conf_type = self.get_conflict_type(conf + (loc,), node, time_lim)
                if conf_type == 'cardinal':
                    return constraints_to_check, True
                elif conf_type == 'semi_cardinal':
                    constraint = constraints_to_check, False
                elif constraint is None:  # Only update a non-cardinal conflict if we haven't found even a semi one
                    constraint = constraints_to_check, False

        return constraint

    def get_conflict_type(self, conflict, node, time_lim):
        """
        given a constraint and father node, we check what type of constraint this is. If it's cardinal, then we
        :param time_lim:
        :param conflict: The conflict we are checking
        :param node: The node whose children we are generating
        :return: a tuple <constraints, conflict type>
        """
        if type(conflict[4][0]) == int:
            constraints = self.extract_vertex_constraints(*conflict)
        else:
            constraints = self.extract_edge_constraint(*conflict)
        c1 = self.__generate_constraint_node(next(iter(constraints[0])), node, time_lim)  # ToDo: Make sure this works + fill new conflicts!!
        c2 = self.__generate_constraint_node(next(iter(constraints[1])), node, time_lim)
        if self.min_best_case:
            node_cost, c1_cost, c2_cost = node.solution.cost[0], c1.sol.cost[0], c2.sol.cost[0]
        else:
            node_cost, c1_cost, c2_cost = node.solution.cost[1], c1.sol.cost[1], c2.sol.cost[1]
        if node_cost + 1 == c1_cost == c2_cost:
            return constraints, 'cardinal'
        elif c1_cost + c2_cost == 2 * node_cost + 1:
            return constraints, 'semi-cardinal'
        else:
            return constraints, 'non-cardinal'

    def find_conflict(self, node):
        """
        Given a solution, this function will validate it.
        i.e checking if any conflicts arise. If there are, returns two constraints: For a conflict (a1,a2,v1,v2,t1,t2),
        meaning agents a1 and a2 cannot both be at vertex v1 or v2 or the edge (v1,v2) between t1 and t2, the function
        will return ((a1,v1,v2,t1,t2), (a2,v1,v2,t1,t2))
        :param node: The node whose solution we are validating.
        :returns: A tuple where the first item is the new constraints and the second is a Boolean indicating whether
        the conflict found is cardinal or not (True for cardinal, False otherwise)
        """
        # node.solution.add_stationary_moves()  # inserts missing time steps
        node.solution.create_movement_tuples()
        constraints = self.__check_previously_conflicting_agents(node.solution, node.conflicting_agents)
        if constraints:
            return constraints

        new_vertex_constraints = node.__find_all_vertex_conflicts()
        if new_vertex_constraints:
            node.add_conflicting_agents(new_vertex_constraints[0], new_vertex_constraints[1])
            return new_vertex_constraints

        # Check for edge conflicts
        new_edge_swap_constraints = node.__find_all_edge_conflicts()
        if new_edge_swap_constraints:
            node.add_conflicting_agents(new_edge_swap_constraints[0], new_edge_swap_constraints[1])
            return new_edge_swap_constraints

        return None

    def __check_conf_in_visited_node(self, visited_nodes, move_i, interval_i, agent_i):
        """
        Checks for a conflict in a node that at least two agents have been at, possible in overlapping times.
        :param visited_nodes: A dictionary of nodes that agents have been at
        :param move_i: The node location
        :param interval_i: The interval of the visiting agent
        :param agent_i: The agent's ID
        :return: Returns a constraint if there is a conflict, otherwise None
        """
        for occupancy in visited_nodes[move_i[1]]:  # Iterate over the times agents have been at this node
            if occupancy[0] != agent_i and Cas.overlapping(interval_i, occupancy[1]):  # There is a conflict.
                return self.extract_vertex_constraints(interval_i, occupancy[1], move_i[1], agent_i, occupancy[0])
        return None

    def extract_vertex_constraints(self, agent_i, agent_j, interval_i, interval_j, vertex):
        """
        Helper function. We know that at that time interval there is some conflict between two given agents on the
        given vertex. This function creates the proper tuple of constraints.

        Interval example:     ______________
                        _____|_\_\_\_\_|<---- The time we want to isolate - Maximal time of overlap.

            Constraints will be of the form (agent, conflict_node, time)
        """
        t = self.__pick_times_to_constrain(interval_i, interval_j)
        return {(agent_i, vertex, t)}, {(agent_j, vertex, t)}

    def extract_edge_constraint(self, agent_i, agent_j, interval_i, interval_j, conflict_edge):
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

    def compute_all_paths_and_conflicts(self, root, use_cat):
        """
        A function that computes the paths for all agents, i.e a solution. Used for the root node before any constraints
        are added.
        :param root:
        :param use_cat:
        :return:
        """

        for agent_id, agent_start in self.tu_problem.start_positions.items():
            agent_plan = self.planner.compute_agent_path(
                root.constraints, agent_id, agent_start, self.tu_problem.goal_positions[agent_id], root.conflict_table,
                min_best_case=self.min_best_case, curr_time=self.curr_time)
            if agent_plan.path:  # Solution found
                root.sol.paths[agent_id] = agent_plan
            else:  # No solution for a particular agent
                print("No solution for agent number " + str(agent_id))
                root.solution = None
                return
        root.sol.add_stationary_moves()  # inserts missing time steps
        root.find_all_conflicts()

        if use_cat:
            for agent, conf_plan in root.sol.paths.items():
                for move in conf_plan.path:
                    root.conflict_table[(move[1])].add((agent, move[0]))

    @staticmethod
    def __get_best_node(open_list):
        """
        this function returns the best node in the open list. I used a function since the best node might be something
        else depending on the search type (lowest cost vs shortest time etc.)
        """
        return open_list.pop()

    def __insert_open_node(self, new_node):
        """
        Simply inserts new_node into the list open_nodes and sorts the list. I used a function for modularity's
        sake - sometimes what is considered to be the "best node" is the lowest cost, while sometimes least conflicts
        or shortest time.
        """
        if self.min_best_case:
            self.open_nodes.push(new_node, new_node.sol.cost[0], new_node.sol.cost[1], new_node.conflicts['count'])
        else:
            self.open_nodes.push(new_node, new_node.sol.cost[1], new_node.sol.cost[0], new_node.conflicts['count'])

    def __insert_into_closed_list(self, new_node):
        """
        Inserts the new node into the closed list. We use a function since inserting the node into the closed
        list demands that we create frozen set out of all constraints for hashing.
        :param new_node: The node to insert.
        """
        self.closed_nodes.add(frozenset((k, tuple(val)) for k, val in new_node.constraints.items()))

    @staticmethod
    def __pick_times_to_constrain(interval_i, interval_j, t='max'):
        """
        Returns the time ticks to constrain in the conflict interval. For example if the intervals are (4,17) and (8,22)
        , the conflicting interval will be (8,17). Note that the format for constraining e.g. time tick 4 is (4, 4).
        This is so we can introduce range constraints with ease later.
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
                            if traversal[2] == move[2] and Cn.strong_overlapping(occ_time, traversal[1]):
                                return self.extract_edge_constraint(traversal[0], agent, traversal[1], occ_time, edge)
                            elif traversal[2] != move[2] and Cas.overlapping(occ_time, traversal[1]):
                                return self.extract_edge_constraint(traversal[0], agent, traversal[1], occ_time, edge)
                else:
                    # Agent begins to travel at move[0][0] and arrives at move[0][1]
                    positions[edge].add((agent, move[0], move[2]))
                    for traversal in positions[edge]:
                        if traversal[0] != agent and Cn.strong_overlapping(move[0], traversal[1]):
                            return self.extract_edge_constraint(traversal[0], agent, traversal[1], move[0], edge)
        return None

    def __generate_constraint_node(self, new_cons, best_node, time_limit):
        new_node = Cn(new_constraints=new_cons, parent=best_node)
        if frozenset((k, tuple(val)) for k, val in new_node.constraints.items()) in self.closed_nodes:
            return None
        agent = next(iter(new_cons))[0]  # Ugly line to extract agent index.
        time_passed = time.time() - self.start_time
        new_plan = self.planner.compute_agent_path(
            new_node.constraints, agent,
            self.tu_problem.start_positions[agent],
            self.tu_problem.goal_positions[agent],
            new_node.conflict_table,
            self.min_best_case, time_limit=time_limit - time_passed,
            curr_time=self.curr_time)  # compute the path for a single agent.
        if new_plan.path:
            new_node.update_solution(new_plan, self.use_cat)
            new_node.sol.compute_solution_cost(self.soc)  # compute the cost
            new_node.find_all_conflicts()
            return new_node
        else:
            return None

    def find_bypass(self, best_node, new_constraints, time_lim):
        """
        Performs the bypass maneuver of ICBS. We look at the two immediate children of the best_node, and if one of
        them offers a helpful bypass, we replace best_node's solution with that child's solution. There is no need to
        add a new node to the CT, just update the cost of best_node, insert it into open and continue.
        Note that best_node should be chosen again for expansion since it previously had the best cost.s
        :param time_lim: Time limit
        :param best_node: The current best node in open
        :param new_constraints: The new constraints found for the path of best_node
        :return: If a bypass was found, updates best_node and returns true. Otherwise False.
        """

        for new_con_set in new_constraints:  # There are only 2 new constraints, we will insert each one into "open"
            child = self.__generate_constraint_node(new_con_set, best_node, time_lim)
            if child:
                if ((self.min_best_case and best_node.solution.cost[0] == child.sol.cost[0]) or
                    (not self.min_best_case and best_node.solution.cost[1] == child.sol.cost[1])) and \
                        child.conflicts['count'] < best_node.conflicts['count']:
                    agent = next(iter(new_con_set))[0]  # Ugly line to extract agent index.
                    best_node.sol.paths[agent] = child.sol.paths[agent]
                    self.__insert_open_node(best_node)
                    return True

        return False  # No bypass found
