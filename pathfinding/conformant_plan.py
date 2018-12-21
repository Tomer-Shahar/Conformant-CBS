"""
Class representing a conformant plan.
"""


class ConformantPlan:

    def __init__(self, plan):
        self.agent_id = plan[0]
        self.path = plan[1]
        self.cost = plan[2]
