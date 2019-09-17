"""
An online variant of cbstu. During plan execution, the agents receive information about their current time and other
agent's current state. This will allow to re-plan with additional information, hopefully leading to higher quality
results in terms of Sum of Costs.
"""

from pathfinding.planners.cbstu import CBSTUPlanner


class OnlineCBSTU:

    def __init__(self, tu_problem):
        """
        Initialize the planner.
        :param tu_problem: a time-uncertainty problem. This is an object that must be capable of updating the current
        state of the world.
        """
        self.tu_problem = tu_problem
        self.edges_and_weights = tu_problem.edges_and_weights
        self.offline_cbstu_planner = CBSTUPlanner(tu_problem)
        self.initial_plan = None
        self.current_plan = None
        self.current_state = None

    def find_initial_path(self):
        """
        Uses the offline planner to find an initial solution. Also creates the current state of the world, where the
        time is 0 and all agents are just in their start positions. During the execution of the plan the current state
        will update. Note that the planner does NOT know when each agent will finish traversing the edge, but instead
        this info is injected during execution from the simulator.
        :return: A solution for the uncertain problem.
        """
        self.initial_plan = self.offline_cbstu_planner.find_solution()
        self.current_plan = self.initial_plan
        self.current_state = {'time': 0,
                              'cost': 0,
                              'at_vertex': self.tu_problem.start_positions,  # Agents that are at a vertex
                              'in_transition': {}}  # Agents that are transitioning.

    def update_current_state(self, curr_time):
        """
        Extract new info from the current state, such as where each agent is and what the current time is. Both provide
        useful information. Some agents will have completed their actions and some will currently be traversing, so they
        will be moved to the list of agents that are at a vertex.
        """
        pass

    def create_new_plans(self):
        """
        Replan for the agents that updated their location.
        :return: New plans for the agents that are at a vertex.
        """
        for agent in self.current_state['at_vertex']:  # Agents that have completed their action, including waiting.
            pass

    def at_goal_state(self):
        """
        Function for verifying if we are at a goal state and can halt the search.
        :return: True if goal state, False otherwise.
        """
        if len(self.current_state['in_transition']) > 0:  # If even one agent is still moving, this isn't a goal state
            return False

        for agent, loc in self.current_state['at_vertex'].items():
            if loc != self.tu_problem.goal_positions[agent]:  # If even one agent is not at their goal state
                return False

        return True  # All agents are at their goal states

    def update_and_replan(self, time):
        """
        Function called when the planner receives an update about the current state of the world. Replans for all agents
        that are at a vertex and require re-planning.
        :return: a new plan for the required agents.
        """
        self.update_current_state(time)  # Update and extract info from the new state.
        return self.create_new_plans()
