import copy
import math
from collections import defaultdict
from pathfinding.planners.utils.time_uncertainty_solution import TimeUncertaintySolution
from pathfinding.planners.constraint_A_star import ConstraintAstar as Cas


class ConstraintNode:
    """
    The class that represents a node in the CT. Contains the current path for each agent and the contraint added to
    this node. We do neot need to save all of the constraints, as they can be extrapolated from the parent nodes by
    traversing the path from the current node to the root.
    """

    def __init__(self, new_constraints=None, parent=None):
        """

        :param new_constraints:
        :param parent: a pointer to the previous node.
        """
        if parent:
            self.constraints = self.append_constraints(parent.constraints, new_constraints)
            self.conflicting_agents = parent.conflicting_agents
            self.conf_num = math.inf
            self.copy_solution(parent)
            self.conflict_table = {agent: moves for agent, moves in parent.conflict_table.items()}
            #self.conflict_table = copy.deepcopy(parent.conflict_table)
        else:
            self.constraints = defaultdict(list)
            self.sol = TimeUncertaintySolution()
            self.conflicting_agents = None  # the agents that conflicted previously (most likely to conflict again)
            self.conflict_table = {}  # Maps agent - > {set of <time, location >}
            self.conflicts = defaultdict(list)
            self.conf_num = 0

        self.parent = parent
        # self.cost = math.inf  # Default value higher than any possible int

    def copy_solution(self, parent):

        self.sol = TimeUncertaintySolution()
        self.sol.copy_solution(parent.sol)

    def add_conflicting_agents(self, agent_i, agent_j):
        """
        Receives two constraints (i.e sets) that each contain a single tuple. We simply want to assign the previously
        conflicting agents to the "conflicting_agents" field.
        """

        self.conflicting_agents = (agent_i, agent_j)

    @staticmethod
    def append_constraints(parent_constraints, new_constraints):
        """
        Combines the parents constraints and the new constraint(s).
        :param parent_constraints: a default dict of the parent constraints
        :param new_constraints: A set containing constraints of the form (a, v, t)
        :return: A new default dict
        """
        new_con_dict = copy.deepcopy(parent_constraints)
        for con in new_constraints:
            new_con_dict[con[1]].append((con[0], con[2]))  # Maps v -> (agent, time)
        return new_con_dict

    def update_solution(self, new_plan, use_cat=True, soc=True):
        """
        Updates the plan for a particular agent in the constraint node. Also updates the conflict table by iterating
        over the old solution and removing the moves the agent previously did. Then, adds the required stationary moves
        to the new path and inserts the moves from the new path into the CAT.

        :param soc: Sum of costs. used for computing solution cost
        :param use_cat: Boolean value, use conflict avoidance table or not
        :param new_plan: The new time uncertainty plan
        :return: updates the CAT and the node's solution.
        """
        if not new_plan.path:
            self.sol = TimeUncertaintySolution.empty_solution(self.sol.nodes_generated)
            self.sol.paths[new_plan.agent] = new_plan
            return
        if not use_cat:
            self.sol.paths[new_plan.agent] = new_plan
            self.sol.create_movement_tuples(agents=[new_plan.agent])
            return

        self.sol.paths[new_plan.agent] = new_plan
        new_moves = self.sol.add_stationary_moves()
        self.sol.create_movement_tuples(agents=[new_plan.agent])
        self.update_conflict_avoidance_table(new_plan.agent, new_moves)
        self.sol.compute_solution_cost(sum_of_costs=soc)  # compute the cost

    def update_conflict_avoidance_table(self, agents=None, new_moves=None):
        """
        Updates the CAT for all agents given. If non are given, update all of them. Assumes that the path is up to date
        including the tuple solution.
        :param new_moves: Additional moves that we want to add, such as stationary moves we added at the end.
        :param agents: Agents to update the CAT for
        :return: Updates the parameter 'conflict_table'
        """
        agents = [agents] if agents else self.sol.paths.keys()
        for agent in agents:
            cat = defaultdict(set)
            for move in self.sol.paths[agent].path:
                cat[move[1]].add(move[0])
            for move in self.sol.tuple_solution[agent]:
                cat[move[1]].add((move[0], move[2]))

            self.conflict_table[agent] = cat
        if new_moves:
            for move in new_moves:
                self.conflict_table[move[0]][move[2]].add(move[1])

    def find_all_conflicts(self, agents=None):
        """
        Similar to find_conflict, except it will find ALL of the conflicts and return them including a count of them.
        :return: A dictionary mapping nodes to conflicts, with another value of 'count' with number of conflicts.
        """
        agents = agents if agents else list(self.sol.paths.keys())
        vertex_conflicts, vertex_count = self.find_all_vertex_conflicts(agents)
        self.sol.create_movement_tuples()
        edge_conflicts, edge_count = self.find_all_edge_conflicts(agents)
        self.conf_num = vertex_count + edge_count
        self.conflicts.update(vertex_conflicts)
        self.conflicts.update(edge_conflicts)

        return self.conflicts

    def find_all_vertex_conflicts(self, agents_to_check=None):
        """
        This function checks if at a certain time interval there might be another agent in the same vertex as the one
        given. Basically, we assume that for most cases, each node is visited by a single agent. We do not want to
        waste time for those nodes. The function maintains a dictionary that maps each node to the agent and time step
        it was there.
        :param agents_to_check: The agents who might be conflicting
        :return: A dictionary of all conflicts.
        """

        visited_nodes = {}  # A dictionary containing all the nodes visited
        cn = defaultdict(list)
        count = 0
        agents_to_check = agents_to_check if agents_to_check else list(self.sol.paths.keys())
        for agent_i in agents_to_check:
            path_i = self.sol.paths[agent_i].path
            for move_i in path_i:
                interval = move_i[0]
                if not move_i[1] in visited_nodes:  # First time an agent has visited this node
                    visited_nodes[move_i[1]] = {(agent_i, interval)}  # Add the interval to the set.
                    continue
                for occupy in visited_nodes[move_i[1]]:  # Iterate over the times agents have been at this node
                    if occupy[0] != agent_i and Cas.overlapping(interval, occupy[1]):  # There's a conflict.
                        #t_range = max(occupy[1][0], interval[0]), min(occupy[1][1], interval[1])
                        cn[move_i[1]].append((agent_i, occupy[0], interval, occupy[1]))
                        count += 1  # (t_range[1] - t_range[0] + 1) ToDo: Set a correct range for this AND EDGES

                visited_nodes[move_i[1]].add((agent_i, interval))  # No conflict, add to vertex set.
        return cn, count

    def find_all_edge_conflicts(self, agents_to_check=None):
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
        positions = defaultdict(set)
        cn = defaultdict(list)  # A dictionary mapping {edge -> (agent1, agent2, agent1_time, agent2_time)}
        count = 0
        if not agents_to_check:
            agents_to_check = list(self.sol.paths.keys())
        for agent in agents_to_check:
            path = self.sol.tuple_solution[agent]
            for move in path:
                edge = move[1]
                positions[edge].add((agent, move[0], move[2]))
                for pres in positions[edge]:
                    if pres[0] != agent:  # Use different overlap tests for same direction or different directions
                        if pres[2] != move[2] and not Cas.overlapping(move[0], pres[1]):  # Opposite directions
                            continue  # no conflict
                        if pres[2] == move[2]:  # Trickier conflict option, same direction
                            occ_i = move[0][0]+1, move[0][1]-1  # Actual occupation times
                            occ_j = pres[1][0]+1, pres[1][1]-1
                            if not Cas.overlapping(occ_i, occ_j):
                                continue
                        cn[edge].append((pres[0], agent, pres[1], move[0], pres[2], move[2]))
                        count += 1  # (t_rng[1] - t_rng[0])
        return cn, count

        """
                if move[0][1] - move[0][0] > 1:  # Edge weight is more than 1
                    for pres in positions[edge]:
                        if pres[0] != agent:  # Use different overlap tests for same direction or different directions
                            if pres[2] != move[2] and not Cas.overlapping(move[0], pres[1]):
                                continue  # no conflict
                            if pres[2] == move[2]:
                                # The actual times the agents occupy the edge:
                                occ_1 = move[0][0], move[0][1] - 1  # Same direction -> last time tick doesn't matter
                                occ_2 = pres[1][0], pres[1][1] - 1
                                if not self.strong_overlapping(occ_1, occ_2):
                                    continue
                                #t_rng = max(occ_time[0], pres[1][0]), min(occ_time[1], pres[1][1])
                            cn[edge].append((pres[0], agent, pres[1], move[0], pres[2], move[2]))
                            count += 1  # (t_rng[1] - t_rng[0])
                else:  # Edge weight is 1
                    # Agent begins to travel at move[0][0] and arrives at move[0][1]
                    for pres in positions[edge]:
                        if pres[0] != agent and self.strong_overlapping(move[0], pres[1]):
                            cn[edge].append((pres[0], agent, pres[1], move[0], pres[2], move[2]))
                            count += 1
                """

    @staticmethod
    def strong_overlapping(time_1, time_2):
        """ Returns true if the time intervals in 'time_1' and 'time_2' overlap strongly
        A strong overlap means that the times are fully overlapping, not just a singular common tick at the end.
        Basically change the '<=' to '<'

        1: A<-- a<-->b -->B
        2: a<-- A -->b<---B>
        3: A<-- a<--- B --->b
        4: a<-- A<--->B --->b
        ===> A <= a < B or a <= A < b
        """

        if (time_1[0] <= time_2[0] < time_1[1]) or (time_2[0] <= time_1[0] < time_2[1]):
            return True

        return False

    def update_conflicts(self, agent):
        """
        called after the node just computed a new plan. This means that the conflicts will change so we must update
        the conflict dictionary (that contains all the current conflicts) and update the count of conflicts accordingly.

        Currently just calls 'find all conflicts'
        :param agent: The agent we are updating for.
        :return: Updates the self.conflicts parameter.
        """
        if self.sol.cost[0] == math.inf:
            return

        self.conflicts = copy.deepcopy(self.parent.conflicts)
        self.conf_num = self.parent.conf_num
        old_plan = self.parent.sol.paths[agent]
        for old_move in old_plan.path:  # Iterate over the old moves and delete the conflicts they led to
            for conflict in self.parent.conflicts[old_move[1]]:  # ToDo: Are there more conflicts or moves in a path?
                if old_plan.agent in conflict:
                    #conf_time = min(conflict[2][1], conflict[3][1]) - max(conflict[2][0], conflict[3][0]) + 1
                    if conflict in self.conflicts[old_move[1]]:
                        self.conf_num -= 1  # conf_time
                        self.conflicts[old_move[1]].remove(conflict)

        for old_move in self.parent.sol.tuple_solution[old_plan.agent]:  # remove old edge conflicts
            for conflict in self.parent.conflicts[old_move[1]]:
                if old_plan.agent in conflict:
                    # conf_time = min(conflict[2][1], conflict[3][1]) - max(conflict[2][0], conflict[3][0])
                    if conflict in self.conflicts[old_move[1]]:
                        self.conf_num -= 1  # conf_time
                        self.conflicts[old_move[1]].remove(conflict)

        self.find_new_vertex_conflicts(old_plan.agent)
        self.find_new_edge_conflicts(old_plan.agent)
        self.parent.conflicts = defaultdict(list, {k: v for k, v in self.parent.conflicts.items() if v != []})
        self.conflicts = defaultdict(list, {k: v for k, v in self.conflicts.items() if v != []})

    def find_new_vertex_conflicts(self, agent):
        """
        Uses the CAT to find new conflicts for a given agent. Note that the CAT must already be updated to the new
        path!
        :return: Returns a dictionary of all conflicts this agent is involved in
        """
        other_agents = {other for other in self.sol.paths if other != agent}
        for move in self.sol.paths[agent].path:  # iterate over all the NEW moves
            for other_agent in other_agents:  # Check if other agents have been there
                if move[1] in self.conflict_table[other_agent]:
                    for other_pres in self.conflict_table[other_agent][move[1]]:  # iter over presences in this location
                        if Cas.overlapping(move[0], other_pres):  # There's a conflict
                            # conf_time = min(presence[1][1], move[0][1]) - max(presence[1][0], move[0][0]) + 1
                            self.conf_num += 1  # conf_time
                            self.conflicts[move[1]].append((agent, other_agent, move[0], other_pres))

    def find_new_edge_conflicts(self, agent):
        """
        Uses the CAT to find new conflicts for a given agent. Note that the CAT must already be updated to the new
        path!
        :return: Returns a dictionary of all conflicts this agent is involved in
        """
        other_agents = {other for other in self.sol.paths if other != agent}
        for move in self.sol.tuple_solution[agent]:
            for other in other_agents:
                if move[1] in self.conflict_table[other]:
                    for other_pres in self.conflict_table[other][move[1]]:  # iter over presences in this location
                        if other_pres[1] != move[2]:  # opposite directions
                            if not Cas.overlapping(move[0], other_pres[0]):
                                continue  # no conflict
                        else:  # Same direction
                            occ_1 = move[0][0], move[0][1] - 1  # Same direction -> last time tick doesn't matter
                            occ_2 = other_pres[0][0], other_pres[0][1] - 1
                            if not self.strong_overlapping(occ_1, occ_2):
                                continue  # It's not a conflict
                        self.conflicts[move[1]].append((agent, other, move[0], other_pres[0], move[2], other_pres[1]))
                        self.conf_num += 1  # conf_time
