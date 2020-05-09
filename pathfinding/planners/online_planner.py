"""
Abstract class for online planners.
"""
import copy


class OnlinePlanner:

    def __init__(self, tu_problem, offline_planner):
        self.initial_plan = None
        self.current_plan = None
        self.current_state = None
        self.tu_problem = tu_problem
        self.offline_planner = offline_planner(tu_problem)
        self.current_state = {'time': 0,
                              'cost': 0,  # Agents that are at a vertex
                              'at_vertex': copy.deepcopy(self.tu_problem.start_positions),
                              'in_transition': {}}  # Agents that are transitioning.
