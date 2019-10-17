"""
An online variant of cbstu. During plan execution, the agents receive information about their current time and other
agent's current state. This will allow to re-plan with additional information, hopefully leading to higher quality
results in terms of Sum of Costs.
"""
import copy

from pathfinding.planners.cbstu import CBSTUPlanner
from pathfinding.planners.constraint_A_star import ConstraintAstar
from pathfinding.planners.utils.time_uncertainty_plan import TimeUncertainPlan


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

    def find_initial_path(self, min_best_case=False, soc=True, time_limit=60, initial_sol=None):
        """
        Uses the offline planner to find an initial solution. Also creates the current state of the world, where the
        time is 0 and all agents are just in their start positions. During the execution of the plan the current state
        will update. Note that the planner does NOT know when each agent will finish traversing the edge, but instead
        this info is injected during execution from the simulator.
        :return: A solution for the uncertain problem.
        """
        if initial_sol:
            self.initial_plan = initial_sol
            self.current_plan = copy.deepcopy(initial_sol)
            self.offline_cbstu_planner.final_constraints = initial_sol.constraints
        else:
            self.initial_plan = self.offline_cbstu_planner.find_solution(min_best_case, time_limit, soc, use_cat=True)
            self.current_plan = copy.deepcopy(self.initial_plan)
        self.current_state = {'time': 0,
                              'cost': 0,                    # Agents that are at a vertex
                              'at_vertex': copy.deepcopy(self.tu_problem.start_positions),
                              'in_transition': {}}  # Agents that are transitioning.

    def update_current_state(self, curr_time, sensed_agents):
        """
        Extract new info from the current state, such as where each agent is and what the current time is. Both provide
        useful information. Some agents will have completed their actions and some will currently be traversing, so they
        will be moved to the list of agents that are at a vertex.
        """
        pass

    def create_new_plans(self, sensing_agents, curr_time):
        """
        Replan for the agents that updated their location.
        :return: New plans for the agents that are at a vertex.
        """
        for agent in sensing_agents:  # Agents that have completed their action, including waiting.
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

    def plan_distributed(self, graphs, constraints, sensing_agents, time):
        """
        Lets agents replan after sensing, but without communication. Each agent plans for itself while taking into
        account possible constraints.
        :param time: current time
        :param graphs: the path that each agent can plan in.
        :param constraints: A set of constraints for all agents
        :param sensing_agents: Agents that performed a sensing action
        :return: Creates new plans for each agent that sensed.
        """

        for agent, loc in sensing_agents.items():
            agent_constraints = self.offline_cbstu_planner.final_constraints | constraints[agent]
            planner = ConstraintAstar(graphs[agent])
            new_plan = planner.compute_agent_path(agent_constraints, agent, loc, graphs[agent].goal_positions[agent],
                                                  set(), False, curr_time=(time, time))
            self.current_plan.paths[agent] = new_plan

        self.current_plan.add_stationary_moves()
