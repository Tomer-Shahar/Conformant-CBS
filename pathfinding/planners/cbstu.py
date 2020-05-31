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
import math
import time

from pathfinding.planners.constraint_A_star import ConstraintAstar as Cas
from pathfinding.planners.utils.constraint_node import ConstraintNode as Cn
from pathfinding.planners.utils.time_error import OutOfTimeError
from pathfinding.planners.utils.time_uncertainty_solution import TimeUncertaintySolution
from pathfinding.planners.utils.custom_heap import OpenListHeap
from collections import defaultdict

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
        self.computed_c_nodes = {}  # Dictionary mapping constraints, conf_num -> Constraint Node

    def find_solution(self, min_best_case=False, time_lim=60, soc=True, use_cat=True, existing_cons=None,
                      curr_time=(0, 0), use_pc=True, use_bp=True):
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
        self.__initialize_class_variables(curr_time, min_best_case, soc, use_cat)
        if not self.create_root(existing_cons):
            raise OutOfTimeError("Couldn't find initial solution")
        try:
            while self.open_nodes:
                if time.time() - self.start_time > time_lim:
                    raise OutOfTimeError('Ran out of time :-(')

                best_node = self.open_nodes.pop()
                self.__insert_into_closed_list(best_node)
                if best_node.conf_num == 0:  # Meaning that there are no conflicts.
                    return self.create_solution(best_node)
                new_constraints, c1, c2, is_cardinal = self.find_best_conflict(best_node, time_lim, use_pc)
                if use_bp and not is_cardinal and self.can_bypass(best_node, new_constraints, c1, c2):
                    continue
                self.__insert_open_node(c1)
                self.__insert_open_node(c2)
            print("Empty open list - No Solution")
            return TimeUncertaintySolution.empty_solution(self.open_nodes.entry_count)

        except OutOfTimeError:  # Ran out of time. Return an empty solution.
            return TimeUncertaintySolution.empty_solution(self.open_nodes.entry_count)

    def __initialize_class_variables(self, curr_time, min_best_case, soc, use_cat):
        """
        Resets all the variables and initializes them with default values
        """
        self.start_time = time.time()
        self.curr_time = curr_time
        self.min_best_case = min_best_case
        self.soc = soc
        self.use_cat = use_cat
        self.computed_c_nodes = {}  # Dictionary mapping constraints, conf_num -> Constraint Node
        self.final_constraints = None
        self.open_nodes = OpenListHeap()
        self.closed_nodes = set()

    def create_solution(self, best_node):
        """
        Compiles together all necessary parameters for the final solution that will be returned.
        :param best_node: The node containing the solution
        :return: The proper Time Uncertain Solution.
        """

        best_node.sol.time_to_solve = time.time() - self.start_time
        best_node.sol.compute_solution_cost(self.soc)
        best_node.sol.nodes_generated = self.open_nodes.entry_count
        self.final_constraints = best_node.constraints
        best_node.sol.constraints = best_node.constraints
        best_node.sol.sic = self.root.sol.cost
        return best_node.sol

    def create_root(self, existing_cons=None):
        self.root = Cn()
        self.compute_all_paths_and_conflicts(self.root)
        if not len(self.root.sol.paths):
            print("No solution for cbs root node")
            empty_root = Cn()
            empty_root.sol = TimeUncertaintySolution.empty_solution(nodes_generated=0)
            return empty_root
        self.root.sol.compute_solution_cost(self.soc)
        if existing_cons:
            self.root.constraints = Cn.append_constraints(self.root.constraints, existing_cons)
        self.__insert_open_node(self.root)  # initialize the list with root
        return self.root

    def find_best_conflict(self, node, time_lim, use_pc):
        """
        Tries to find cardinal, semi-cardinal or non-cardinal conflicts in the given node, in that order of priority.
        If the conflict found is cardinal, the function will return a tuple of <constraint, True> otherwise it will
        return <constraint, False> in order to assess later on if it's cardinal or not.
        :param use_pc: Whether to use prioritizing conflicts or not.
        :param time_lim: The time limit to pass over. This function can be quite long
        :param node: The node who we're searching a solution for.
        :return: A tuple of the constraint and a boolean indicating if it's cardinal or not (for the bypass)
        """
        if not use_pc:
            new_con, c1, c2 = self.generate_children_nodes(node, time_lim)
            return new_con, c1, c2, True
        semi_cardinals, non_cardinals = [], []
        sorted_constraints = self.get_sorted_constraints(node.conflicts)
        for constraint in sorted_constraints:
            c1, c2, conf_type = self.get_conflict_type(constraint, node, time_lim)
            if conf_type == 'cardinal':
                return constraint, c1, c2, True
            elif conf_type == 'semi-cardinal':
                semi_cardinals.append((constraint, c1, c2, False))
            else:  # Only update a non-cardinal conf if we haven't found even a semi one
                non_cardinals.append((constraint, c1, c2, False))

        return semi_cardinals[0] if len(semi_cardinals) > 0 else non_cardinals[0]

    def generate_children_nodes(self, node, time_lim):
        new_constraints = self.find_single_conflict(node)
        res = [new_constraints]
        for new_conf in new_constraints:
            c = self.generate_constraint_node(new_conf, node, time_lim)
            res.append(c)

        return tuple(res)

    def get_conflict_type(self, constraints, node, time_lim):
        """
        given a constraint and father node, we check what type of constraint this is. If it's cardinal, then we
        :param time_lim:
        :param constraints: The constraints we are checking
        :param node: The node whose children we are generating
        :return: a tuple <constraints, conflict type>
        """

        c1 = self.generate_constraint_node(constraints[0], node, time_lim)
        c2 = self.generate_constraint_node(constraints[1], node, time_lim)

        if self.min_best_case:
            node_cost, c1_cost, c2_cost = node.sol.cost[0], c1.sol.cost[0], c2.sol.cost[0]
        else:
            node_cost, c1_cost, c2_cost = node.sol.cost[1], c1.sol.cost[1], c2.sol.cost[1]
        if node_cost < c1_cost and node_cost < c2_cost:
            return c1, c2, 'cardinal'
        elif (c1_cost > node_cost and c2_cost == node_cost) or (c2_cost > node_cost and c1_cost == node_cost):
            return c1, c2, 'semi-cardinal'
        else:
            return c1, c2, 'non-cardinal'

    def find_single_conflict(self, node):
        """
        Given a solution, this function will validate it.
        i.e checking if any conflicts arise. If there are, returns two constraints: For a conflict (a1,a2,v1,v2,t1,t2),
        meaning agents a1 and a2 cannot both be at vertex v1 or v2 or the edge (v1,v2) between t1 and t2, the function
        will return ((a1,v1,v2,t1,t2), (a2,v1,v2,t1,t2))
        :param node: The node whose solution we are validating.
        :returns: A tuple where the first item is the new constraints and the second is a Boolean indicating whether
        the conflict found is cardinal or not (True for cardinal, False otherwise)
        """
        node.sol.create_movement_tuples()

        new_vertex_constraints, count = node.find_all_vertex_conflicts()
        for vertex, conflicts in new_vertex_constraints.items():
            for conflict in conflicts:
                node.add_conflicting_agents(conflict[0], conflict[1])
                return self.extract_vertex_constraints(*conflict, vertex)

        # Check for edge conflicts
        new_edge_swap_constraints, count = node.find_all_edge_conflicts()
        for edge, conflicts in new_edge_swap_constraints.items():
            for conflict in conflicts:
                node.add_conflicting_agents(conflict[0], conflict[1])
                return self.extract_edge_cons(*conflict, edge)

        return set()

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
                return self.extract_vertex_constraints(agent_i, occupancy[0], interval_i, occupancy[1], move_i[1])
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

    def extract_edge_cons(self, agent_i, agent_j, interval_i, interval_j, dir_i, dir_j, conflict_edge):
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
        :param dir_j: The direction agent i is going ('f' for forwards, 'b' for backwards)
        :param dir_i: The direction agent j is going ('f' for forwards, 'b' for backwards)
        :param conflict_edge: The edge where the conflict occurs
        :return: The appropriate constraints
        """

        agent_i_constraints = set()
        agent_j_constraints = set()

        if dir_i == dir_j and interval_i[1] != interval_j[1]:  # same direction
            if interval_i[1] < interval_j[1]:  # i arrives first, his last tick is not a problem.
                interval_i = interval_i[0], interval_i[1] - 1
            else:  # j arrives first, his last tick is not a problem.
                interval_j = interval_j[0], interval_j[1] - 1

        t = self.__pick_times_to_constrain(interval_i, interval_j)
        agent_i_constraints.add((agent_i, conflict_edge, t))
        agent_j_constraints.add((agent_j, conflict_edge, t))

        return agent_i_constraints, agent_j_constraints

    def compute_all_paths_and_conflicts(self, root):
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
                mbc=self.min_best_case, curr_time=self.curr_time)
            if len(agent_plan.path):  # Solution found
                root.sol.paths[agent_id] = agent_plan
            else:  # No solution for a particular agent
                print("No solution for agent number " + str(agent_id))
                root.solution = None
                return
        root.sol.add_stationary_moves()  # inserts missing time steps
        root.conflicts = root.find_all_conflicts()

        if self.use_cat:
            root.update_conflict_avoidance_table()

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
            self.open_nodes.push(new_node, new_node.sol.cost[0], new_node.conf_num, new_node.sol.cost[1])
        else:
            self.open_nodes.push(new_node, new_node.sol.cost[1], new_node.conf_num, new_node.sol.cost[0])

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

        visited_nodes = defaultdict(set)  # A dictionary containing all the nodes visited
        for agent in [agent_i, agent_j]:
            for move in solution.paths[agent].path:
                interval = move[0]
                if not move[1] in visited_nodes:  # First time an agent has visited this node
                    visited_nodes[move[1]].add((agent, interval))  # Add the interval to the set.
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
                    for pres in positions[edge]:
                        if pres[0] != agent:
                            if pres[2] == move[2] and Cn.strong_overlapping(occ_time, pres[1]):
                                return self.extract_edge_cons(pres[0], agent, pres[1], move[0], pres[2], occ_time, edge)
                            elif pres[2] != move[2] and Cas.overlapping(occ_time, pres[1]):
                                return self.extract_edge_cons(pres[0], agent, pres[1], move[0], pres[2], occ_time, edge)
                else:
                    # Agent begins to travel at move[0][0] and arrives at move[0][1]
                    positions[edge].add((agent, move[0], move[2]))
                    for pres in positions[edge]:
                        if pres[0] != agent and Cn.strong_overlapping(move[0], pres[1]):
                            return self.extract_edge_cons(pres[0], agent, pres[1], move[0], pres[2], move[2], edge)
        return None

    def generate_constraint_node(self, new_cons, best_node, time_limit):
        merged_constraints = Cn.append_constraints(best_node.constraints, new_cons).items()
        key_cons = frozenset((k, tuple(val)) for k, val in merged_constraints)
        if key_cons in self.closed_nodes:  # We already expanded this node
            empty_node = Cn()
            empty_node.sol = TimeUncertaintySolution.empty_solution()
            return empty_node
        if key_cons in self.computed_c_nodes:
            return self.computed_c_nodes[key_cons]  # Node has been computed but not inserted into closed yet.
        new_node = Cn(new_constraints=new_cons, parent=best_node)
        agent = next(iter(new_cons))[0]  # Ugly line to extract agent index.
        time_passed = time.time() - self.start_time
        new_plan = self.planner.compute_agent_path(
            new_node.constraints, agent,
            self.tu_problem.start_positions[agent],
            self.tu_problem.goal_positions[agent],
            best_node.conflict_table,
            self.min_best_case, time_limit=time_limit - time_passed,
            curr_time=self.curr_time)  # compute the path for a single agent.
        if time.time() - self.start_time > time_limit:
            raise OutOfTimeError('Ran out of time :-(')
        new_node.update_solution(new_plan, self.use_cat, self.soc)
        new_node.update_conflicts(agent)
        self.computed_c_nodes[key_cons] = new_node
        return new_node

    def can_bypass(self, best_node, new_constraints, c1, c2):
        """
        Performs the bypass maneuver of ICBS. We look at the two immediate children of the best_node, and if one of
        them offers a helpful bypass, we replace best_node's solution with that child's solution. There is no need to
        add a new node to the CT, just update the cost of best_node, insert it into open and continue.
        Note that best_node should be chosen again for expansion since it previously had the best cost.s
        :param c1: First child, generated from the first new constraint
        :param c2: Second child, generated from the second new constraint
        :param best_node: The current best node in open
        :param new_constraints: The new constraints found for the path of best_node
        :return: If a bypass was found, updates best_node and returns true. Otherwise False.
        """
        children = [c1, c2]
        for idx, new_con_set in enumerate(new_constraints):
            child = children[idx]  # There are only 2 new constraints, we will insert each one into "open"
            if ((self.min_best_case and best_node.sol.cost[0] == child.sol.cost[0]) or
                (not self.min_best_case and best_node.sol.cost[1] == child.sol.cost[1])) and \
                    child.conf_num < best_node.conf_num:
                agent = next(iter(new_con_set))[0]  # Ugly line to extract agent index.
                best_node.update_solution(child.sol.paths[agent], self.use_cat, self.soc)
                best_node.conf_num = child.conf_num
                best_node.conflicts = child.conflicts
                self.__insert_open_node(best_node)
                return True

        return False  # No bypass found, just

    def get_sorted_constraints(self, all_conflicts):
        """
        Receives a dictionary of conflicts creates constraints out of them. Then sorts the constraints based on the
        time that is constrained. This is because we'd rather sort earlier conflicts first.
        :param all_conflicts: A dictionary of location - > [list of conflicts]
        :return:
        """
        result = []
        for loc, conflicts in all_conflicts.items():
            for conf in conflicts:
                conflict = conf + (loc,)
                if type(conflict[-1][0]) == int:  # Vertex constraint
                    constraints = self.extract_vertex_constraints(*conflict)
                else:  # Edge constraint
                    constraints = self.extract_edge_cons(*conflict)
                result.append(constraints)

        result.sort(key=lambda x: next(iter(x[0]))[2][0], reverse=False)  # ToDo: Change this or not?
        return result
