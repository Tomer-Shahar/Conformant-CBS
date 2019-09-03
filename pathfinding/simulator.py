"""
Simulator that runs an online version of CBSTU, with sensing. It executes the initial plan and periodically updates the
planner with new information gained. The simulator imitates this by giving each agent a chance to sense the current
time. This occurs when the agent is at a node (waiting or having just arrived).
"""

from pathfinding.planners.utils.map_reader import TimeUncertaintyProblem
from pathfinding.planners.online_cbstu import *
import random


class MAPFSimulator:

    def __init__(self, tu_problem, sensing_prob=1, cooperation=True):
        """
        :param tu_problem: A time uncertainty problem
        :param sensing_prob: The probability to sense at each node
        :param cooperation: Mode of cooperation. Irrelevant if there's no sensing. Otherwise, it's either full
        cooperation (True) or no cooperation (False). Full cooperation means agents can communicate with each other. No
        cooperation means agents replan for themselves only.
        """

        self.tu_problem = tu_problem
        self.sensing_prob = sensing_prob
        self.cooperation = cooperation
        self.online_CSTU = OnlineCBSTU(tu_problem)
        self.final_solution = {}
        for agent in self.tu_problem.start_positions:
            self.final_solution[agent] = [self.tu_problem.start_positions[agent]]

    def begin_execution(self):

        self.online_CSTU.find_initial_path()
        time = 0

        while not self.at_goal_state():
            self.execute_next_step(time)
            self.online_CSTU.update_current_state(time)
            self.online_CSTU.create_new_plans()
            time += 1

    def execute_next_step(self, time):
        """
        Executes the next action for all agents that are at a vertex and updates the current state accordingly. Also
        inserts into the transitioning agents dict the ones that just performed an action that isn't waiting (and
        removes them from the list of agents that are at a vertex)
        :return:
        """
        static_agents = self.online_CSTU.current_state['at_vertex']
        at_vertex = {}
        in_transition = {}
        for agent in static_agents:
            action = self.get_next_action(agent)
            actual_time = random.randint(action[2][0], action[2][1])  # Randomly choose real traversal time
            self.final_solution[agent].append((action[0], actual_time))
            if action[0] != action[1]:  # Not a wait action
                self.online_CSTU.current_state['at_vertex'].pop(agent, None)
                self.online_CSTU.current_state['in_transition'][agent] = action[1], actual_time

    def at_goal_state(self):
        """
        Function for verifying if we are at a goal state and can halt the search.
        :return: True if goal state, False otherwise.
        """
        if len(self.online_CSTU.current_state['in_transition']) > 0:
            return False  # If even one agent is still moving, this isn't a goal state

        for agent, loc in self.online_CSTU.current_state['at_vertex'].items():
            if loc != self.tu_problem.goal_positions[agent]:  # If even one agent is not at their goal state
                return False

        return True  # All agents are at their goal states

    def get_next_action(self, agent):
        """
        Extract from the current plan the next action that will be taken.
        :param agent: The agent for the needed action
        :return: The action done including possible completion times.
        """
        start_location = self.online_CSTU.current_plan.paths[agent].path[0]
        destination = self.online_CSTU.current_plan.paths[agent].path[1]
        return start_location[1], destination[1], destination[0]

