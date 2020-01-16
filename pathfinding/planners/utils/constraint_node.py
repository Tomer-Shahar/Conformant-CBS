import copy

from pathfinding.planners.utils.time_uncertainty_plan import TimeUncertainPlan
from pathfinding.planners.utils.time_uncertainty_solution import TimeUncertainSolution


class ConstraintNode:
    """
    The class that represents a node in the CT. Contains the current path for each agent and the contraint added to
    this node. We do not need to save all of the constraints, as they can be extrapolated from the parent nodes by
    traversing the path from the current node to the root.

    solution - a ConformantSolution object.
    parent - a pointer to the previous node.
    conflicting_agents - the agents that conflicted previously (most likely to conflict again)
    conflict_table - a dictionary mapping agent_id to a set containing tuples of <Time, Location> for each position
    in the agents path.
    """

    def __init__(self, new_constraints=None, parent=None, use_cat=True):

        if parent:
            self.constraints = self.__append_constraints(parent.constraints, new_constraints)
            self.copy_solution(parent)
            self.conflicting_agents = parent.conflicting_agents
            if use_cat:
                self.conflict_table = parent.conflict_table.copy()
            else:
                self.conflict_table = {}
        else:
            self.constraints = {}
            self.solution = TimeUncertainSolution()
            self.conflicting_agents = None
            self.conflict_table = {}

        self.parent = parent
        # self.cost = math.inf  # Default value higher than any possible int

    def copy_solution(self, parent):

        self.solution = TimeUncertainSolution()
        self.solution.copy_solution(parent.solution)

    def add_conflicting_agents(self, con_1, con_2):
        """
        Receives two constraints (i.e sets) that each contain a single tuple. We simply want to assign the previously
        conflicting agents to the "conflicting_agents" field.
        """
        agent_i = None
        agent_j = None
        for con in con_1:
            agent_i = con[0]

        for con in con_2:
            agent_j = con[0]

        self.conflicting_agents = (agent_i, agent_j)

    @staticmethod
    def __append_constraints(parent_constraints, new_constraints):
        """
        Combines the parents constraints and the new constraint(s).
        """
        new_con_dict = parent_constraints.copy()  # ToDo: Can we remove the copy function all together?
        for con in new_constraints:
            if con not in new_con_dict:
                new_con_dict[con[1]] = []
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
        if use_cat:
            for move in self.solution.paths[new_plan.agent_id].path:
                min_time, max_time = move[0][0], move[0][1]
                for tick in range(min_time, max_time + 1):
                    if (tick, move[1]) in self.conflict_table and\
                            new_plan.agent_id in self.conflict_table[(tick, move[1])]:
                        self.conflict_table[(tick, move[1])].remove(new_plan.agent_id)

        self.solution.paths[new_plan.agent_id] = new_plan
        self.solution.add_stationary_moves()  # inserts missing time steps
        if use_cat:
            for move in new_plan.path:
                min_time, max_time = move[0][0], move[0][1]
                for tick in range(min_time, max_time + 1):
                    if (tick, move[1]) in self.conflict_table:
                        self.conflict_table[(tick, move[1])].add(new_plan.agent_id)
                    else:
                        self.conflict_table[(tick, move[1])] = {new_plan.agent_id}


