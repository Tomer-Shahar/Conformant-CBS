"""
Class representing a conformant plan.
"""

import math


class TimeUncertaintyPlan:

    def __init__(self, agent_id, path, cost):
        """
        :param agent_id: The agent number (i.e 1,2,3..)
        :param path: A list containing tuples of the form <time range, coordinate> where both time range and coordinate
        are tuples as well. For example ((1, 10), (4,5)) meaning between times 1 and 10 the agent might be at 4,5
        :param cost: The cost of the path. A tuple <min cost, max cost>
        """
        self.agent = agent_id
        self.path = path
        self.cost = cost

    def compute_cost(self):
        """
        Computes the definitive cost of the path, mainly needed when an agent reaches the goal position but then moves.
        :return: Updates the correct cost.
        """
        if not self.path:
            self.cost = math.inf, math.inf
        goal = self.path[-1][1]
        self.cost = 0, 0
        if len(self.path) == 1:
            self.cost = self.path[0][0]

        first_time_at_goal = self.path[-1]
        for presence in reversed(self.path):
            if presence[1] != goal:
                self.cost = first_time_at_goal[0]
                break
            if presence[1] == goal:
                first_time_at_goal = presence

    @staticmethod
    def get_empty_plan(agent):
        return TimeUncertaintyPlan(agent, [], (math.inf, math.inf))
