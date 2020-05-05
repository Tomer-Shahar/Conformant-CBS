"""
Virtual class for online planners.
"""


class OnlinePlanner:

    def __init__(self, tu_problem, offline_planner):
        self.initial_plan = None
        self.current_plan = None
        self.current_state = None
        self.tu_problem = tu_problem
        self.offline_planner = offline_planner(tu_problem)

