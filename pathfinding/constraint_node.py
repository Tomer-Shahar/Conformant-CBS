import copy
from pathfinding.conformant_solution import ConformantSolution


class ConstraintNode:
    """
    The class that represents a node in the CT. Contains the current path for each agent and the contraint added to
    this node. We do not need to save all of the constraints, as they can be extrapolated from the parent nodes by
    traversing the path from the current node to the root.

    solution - a ConformantSolution object.
    parent - a pointer to the previous node.
    conflicting_agents - the agents that conflicted previously (most likely to conflict again)
    open_conflicts - a set containing all known conflicts currently. Used in the low-level solver.
    """

    def __init__(self, new_constraints=None, parent=None):

        if parent:
            self.constraints = self.__append_constraints(parent.constraints, new_constraints)
            self.copy_solution(parent)
            self.conflicting_agents = parent.conflicting_agents
            self.open_conflicts = parent.open_conflicts
        else:
            self.constraints = frozenset()
            self.solution = ConformantSolution()
            self.conflicting_agents = None
            self.open_conflicts = set()

        self.parent = parent
        # self.cost = math.inf  # Default value higher than any possible int

    def copy_solution(self, parent):

        self.solution = ConformantSolution()
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
        Adds new constraints to the parent constraints.
        We use a frozen set so that the constraints can act as a key in the closed list.
        """
        con_set = set(copy.deepcopy(parent_constraints))
        con_set.update(new_constraints)
        new_constraints = frozenset(con_set)
        return new_constraints
