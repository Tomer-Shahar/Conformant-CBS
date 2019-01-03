"""
Class representing a conformant plan.
"""


class ConformantPlan:

    def __init__(self, agent_id, path, cost):
        """
        :param agent_id: The agent number (i.e 1,2,3..)
        :param path: A list containing tuples of the form <time range, coordinate> where both time range and coordinate
        are tuples as well. For example ((1, 10), (4,5)) meaning between times 1 and 10 the agent might be at 4,5
        :param cost: The cost of the path. A tuple <min cost, max cost>
        """
        self.agent_id = agent_id
        self.path = path
        self.cost = cost
