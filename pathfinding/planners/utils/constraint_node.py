import copy
from collections import defaultdict
from pathfinding.planners.utils.time_uncertainty_solution import TimeUncertaintySolution
from pathfinding.planners.constraint_A_star import ConstraintAstar as Cas


class ConstraintNode:
    """
    The class that represents a node in the CT. Contains the current path for each agent and the contraint added to
    this node. We do not need to save all of the constraints, as they can be extrapolated from the parent nodes by
    traversing the path from the current node to the root.
    """

    def __init__(self, new_constraints=None, parent=None, use_cat=True):
        """

        :param new_constraints:
        :param parent: a pointer to the previous node.
        :param use_cat: whether to use the CAT or not.
        """
        if parent:
            self.constraints = self.append_constraints(parent.constraints, new_constraints)
            self.copy_solution(parent)
            self.conflicting_agents = parent.conflicting_agents
            self.conflict_table = parent.conflict_table.copy()

        else:
            self.constraints = defaultdict(list)
            self.sol = TimeUncertaintySolution()
            self.conflicting_agents = None  # the agents that conflicted previously (most likely to conflict again)
            self.conflict_table = defaultdict(list)  # Maps location - > [list of <agent, time at location >]
            self.conflicts = {}

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

    def update_solution(self, new_plan, use_cat):
        """
        Updates the plan for a particular agent in the constraint node. Also updates the conflict table by iterating
        over the old solution and removing the moves the agent previously did. Then, adds the required stationary moves
        to the new path and inserts the moves from the new path into the CAT.

        :param use_cat: Boolean value, use conflict avoidance table or not
        :param new_plan: The new time uncertainty plan
        :return: updates the CAT and the node's solution.
        """
        self.sol.paths[new_plan.agent] = new_plan
        if not new_plan.path:
            return
        self.sol.add_stationary_moves()  # inserts missing time steps

        if use_cat:
            for move in self.parent.sol.paths[new_plan.agent].path:
                for presence in list(self.conflict_table[move[1]]):
                    if presence[0] == new_plan.agent:
                        self.conflict_table[move[1]].remove(presence)
            for move in self.parent.sol.tuple_solution[new_plan.agent]:
                for presence in list(self.conflict_table[move[1]]):
                    if presence[0] == new_plan.agent:
                        self.conflict_table[move[1]].remove(presence)

            for move in new_plan.path:
                self.conflict_table[move[1]].append((new_plan.agent, move[0]))
            for move in self.sol.tuple_solution[new_plan.agent]:
                self.conflict_table[move[1]].append((new_plan.agent, move[0], move[2]))

    def find_all_conflicts(self, agents=None):  # ToDO: Reduce this
        """
        Similar to find_conflict, except it will find ALL of the conflicts and return them including a count of them.
        :return: A dictionary mapping nodes to conflicts, with another value of 'count' with number of conflicts.
        """
        agents = agents if agents else list(self.sol.paths.keys())
        vertex_conflicts = self.find_all_vertex_conflicts(agents)
        self.sol.create_movement_tuples()
        edge_conflicts = self.find_all_edge_conflicts(agents)
        count = vertex_conflicts['count'] + edge_conflicts['count']
        conflicts = {**vertex_conflicts, **edge_conflicts, 'count': count}
        return conflicts

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
        cn['count'] = 0
        if not agents_to_check:
            agents_to_check = list(self.sol.paths.keys())
        for agent_i in agents_to_check:
            plan_i = self.sol.paths[agent_i]
            for move_i in plan_i.path:
                interval = move_i[0]
                if not move_i[1] in visited_nodes:  # First time an agent has visited this node
                    visited_nodes[move_i[1]] = {(agent_i, interval)}  # Add the interval to the set.
                    continue
                for occupy in visited_nodes[move_i[1]]:  # Iterate over the times agents have been at this node
                    if occupy[0] != agent_i and Cas.overlapping(interval, occupy[1]):  # There's a conflict.
                        t_range = max(occupy[1][0], interval[0]), min(occupy[1][1], interval[1])
                        cn[move_i[1]].append((agent_i, occupy[0], interval, occupy[1]))
                        cn['count'] += (t_range[1] - t_range[0] + 1)

                visited_nodes[move_i[1]].add((agent_i, interval))  # No conflict, add to vertex set.
        return cn

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
        cn['count'] = 0
        if not agents_to_check:
            agents_to_check = list(self.sol.paths.keys())
        for agent in agents_to_check:
            path = self.sol.tuple_solution[agent]
            for move in path:
                edge = move[1]
                if move[0][1] - move[0][0] > 1:  # Edge weight is more than 1
                    occ_time = move[0][0], move[0][1] - 1
                    positions[edge].add((agent, (move[0][0], move[0][1] - 1), move[2]))
                    for pres in positions[edge]:
                        if pres[0] != agent:  # Use different overlap tests for same direction or different directions
                            if (pres[2] == move[2] and self.strong_overlapping(occ_time, pres[1])) or\
                                    (pres[2] != move[2] and Cas.overlapping(occ_time, pres[1])):
                                t_rng = max(occ_time[0], pres[1][0]), min(occ_time[1], pres[1][1])
                                cn[edge].append((pres[0], agent, pres[1], occ_time))
                                cn['count'] += (t_rng[1] - t_rng[0])
                else:  # Edge weight is 1
                    # Agent begins to travel at move[0][0] and arrives at move[0][1]
                    positions[edge].add((agent, move[0], move[2]))
                    for pres in positions[edge]:
                        if pres[0] != agent and self.strong_overlapping(move[0], pres[1]):
                            # cn[edge].extend([(pres[0], agent, tick) for tick in range(t_rng[0], t_rng[1] + 1)])
                            cn[edge].append((pres[0], agent, pres[1], move[0]))
                            cn['count'] += 1
        return cn

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

    def update_conflicts(self, old_plan):
        """
        called after the node just computed a new plan. This means that the conflicts will change so we must update
        the conflict dictionary (that contains all the current conflicts) and update the count of conflicts accordingly.

        Currently just calls 'find all conflicts'
        :param old_plan: The previous plan. We use this to delete previous conflicts
        :return: Updates the self.conflicts parameter.
        """

        self.conflicts = copy.deepcopy(self.parent.conflicts)
        for old_move in old_plan.path:  # Iterate over the old moves and delete the conflicts they lead to
            for conflict in self.conflicts[old_move[1]]:
                if old_plan.agent in conflict:
                    conf_time = min(conflict[2][1], conflict[3][1]) - max(conflict[2][0], conflict[3][0]) + 1
                    self.conflicts['count'] -= conf_time
                    self.conflicts[old_move[1]].remove(conflict)

        self.find_new_vertex_conflicts(old_plan.agent)
        self.find_new_edge_conflicts(old_plan.agent)

    def find_new_vertex_conflicts(self, agent):
        """
        Uses the CAT to find new conflicts for a given agent. Note that the CAT must already be updated to the new
        path!
        :return: Returns a dictionary of all conflicts this agent is involved in
        """

        for move in self.sol.paths[agent].path:  # iterate over all the NEW moves
            for presence in self.conflict_table[move[1]]:  # iterate over presences in this location
                if presence[0] != agent and Cas.overlapping(move[0], presence[1]):  # There's a conflict
                    conf_time = min(presence[1][1], move[0][1]) - max(presence[1][0], move[0][0]) + 1
                    self.conflicts['count'] += conf_time
                    self.conflicts[move[1]].append((agent, presence[0], move[0], presence[1]))

    def find_new_edge_conflicts(self, agent):
        """
        Uses the CAT to find new conflicts for a given agent. Note that the CAT must already be updated to the new
        path!
        :return: Returns a dictionary of all conflicts this agent is involved in
        """

        for move in self.sol.tuple_solution[agent]:
            for presence in self.conflict_table[move[1]]:  # iterate over presences in this location
                if presence[0] != agent:
                    if (move[2] == presence[2] and self.strong_overlapping(move[1], presence[1])) or\
                       (move[2] != presence[2] and Cas.overlapping(move[1], presence[1])):
                        occupy_1 = move[1][0], move[1][1] - 1
                        occupy_2 = presence[1][0], presence[1][1] - 1
                        conf_time = min(occupy_1[1], occupy_2[1]) - max(occupy_1[0], occupy_2[0])
                        self.conflicts[move[1]].append((agent, presence[0], move[1], presence[1], move[2]))
                        self.conflicts['count'] += conf_time
