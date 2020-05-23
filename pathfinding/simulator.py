"""
Simulator that runs an online version of CBSTU, with sensing. It executes the initial plan and periodically updates the
planner with new information gained. The simulator imitates this by giving each agent a chance to sense the current
time. This occurs when the agent is at a node (waiting or having just arrived).
"""

from pathfinding.planners.utils.time_uncertainty_solution import *
from pathfinding.planners.online_cbstu import *
from pathfinding.planners.online_worst_case_cbs import OnlinePessimisticCBS
from pathfinding.planners.prioritized_planner import *
from pathfinding.planners.utils.time_error import OutOfTimeError
import time
import random


class MAPFSimulator:

    def __init__(self, tu_problem, sensing_prob=1, solver='cbstu', edge_dist='uniform'):
        """
        :param tu_problem: A time uncertainty problem
        :param sensing_prob: The probability to sense at each node
        """

        self.tu_problem = tu_problem
        self.sensing_prob = sensing_prob
        self.communication = True

        if solver == 'pessimistic':
            self.sensing_prob = 1
            self.communication = False
            self.online_planner = OnlinePessimisticCBS(tu_problem)
        elif solver == 'cbstu':
            self.online_planner = OnlineCBSTU(tu_problem, self.sensing_prob == 1)  # The online solver
        elif solver == 'prioritized':
            self.online_planner = PrioritizedPlanner(tu_problem)
        else:
            print('Invalid planner chosen')
        self.final_solution = TimeUncertaintySolution()  # Maintain the final path taken by agents
        self.arrival_times = {0: []}  # Maintain a dictionary of when agents will arrive to their next destination
        self.sim_time = 0
        self.real_weights = {}  # The actual times it'll take to traverse the edges.
        self.generate_real_weights(distribution=edge_dist)

        for agent in self.tu_problem.start_positions:
            self.final_solution.paths[agent] = TimeUncertaintyPlan(
                                                agent_id=agent,
                                                path=[((0, 0), self.tu_problem.start_positions[agent])],
                                                cost=(0, 0))

    def generate_real_weights(self, distribution='uniform'):
        """
        Assigns a fixed weight for each edge. The weight is chosen based on distribution given.
        :return: Updates the real_weights parameter.
        """

        for vertex_i, edges in self.tu_problem.edges_and_weights.items():
            self.real_weights[vertex_i, vertex_i] = 1
            for vertex_j in edges:
                if distribution == 'uniform':
                    weight = random.randint(vertex_j[1][0], vertex_j[1][1])
                elif distribution == 'min':
                    weight = vertex_j[1][0]
                elif distribution == 'max':
                    weight = vertex_j[1][1]
                else:
                    weight = random.randint(vertex_j[1][0], vertex_j[1][1])

                self.real_weights[(vertex_i, vertex_j[0])] = weight

    def begin_execution(self, min_best_case=False, use_pc=True, use_bp=True, soc=True, time_limit=60,
                        communication=True, initial_sol=False):
        """
        The main function of the simulator. Finds an initial solution and runs it, occasionally sensing and broadcasting
        if need be.
        :param use_bp: Use bypass or not for cbstu
        :param use_pc: Prioritize conflicts or not for cbstu
        :param initial_sol: In case we already computed the initial valid solution and don't want to waste time.
        :param time_limit: Time limit for total execution, NOT including finding an initial path.
        :param soc: If to use sum of costs or makespan
        :param min_best_case: Boolean value. If to minimize the best case or worst case.
        :param communication: Mode of cooperation. Irrelevant if there's no sensing. Otherwise, it's either full
        cooperation (True) or no cooperation (False). Full cooperation means agents can communicate with each other. No
        cooperation means agents replan for themselves only (distributed planning).

        :return: The path that was ultimately taken for each agent.
        """
        self.create_initial_solution(min_best_case, soc, use_pc, use_bp, time_limit=time_limit, initial_sol=initial_sol)
        if self.online_planner.initial_plan.paths == {}:  # No initial solution was found.
            raise OutOfTimeError('Reached time limit')
        self.communication = communication
        self.sim_time = 0
        start_time = time.time()
        graphs, constraints = None, {}
        pos_cons = None
        if not communication:
            graphs, constraints, pos_cons = self.online_planner.create_plan_graphs_and_constraints()

        while True:
            if time.time() - start_time >= time_limit:
                raise OutOfTimeError('Reached time limit')
            if self.at_goal_state():
                break
            sensing_agents = self.simulate_sensing_and_broadcast()
            replan_time = time_limit - (time.time() - start_time)
            if len(sensing_agents) > 0:  # If at least 1 agent sensed.
                if self.communication:
                    self.online_planner.create_new_centralized_plan(self.sim_time, sensing_agents, replan_time)
                else:
                    self.online_planner.plan_distributed(graphs, constraints, pos_cons, sensing_agents, self.sim_time)
            self.execute_next_step()
            self.sim_time += 1

        self.final_solution.time_to_solve = time.time() - start_time + self.online_planner.initial_plan.time_to_solve
        self.final_solution.nodes_generated = self.online_planner.initial_plan.nodes_generated
        self.final_solution.compute_solution_cost()
        #self.print_final_solution()
        return self.final_solution

    def simulate_sensing_and_broadcast(self):
        """
        Simulates the sensing of location and time, and broadcasting if need be. The simulator always keeps the actual
        arrival times of the agents, but only if they "sense", they get to know what their location and time is. The
        broadcasting is done by updating the online planner.

        :return: a dictionary of agents that were able to sense.
        """
        sensed_agents = {}

        # Iterate over agents that were on the move and decide if they already arrived to their destination
        # This is data for the simulator only
        if self.sim_time in self.arrival_times:
            for arrival in self.arrival_times[self.sim_time]:
                self.online_planner.current_state['at_vertex'][arrival[0]] = arrival[1]
                self.online_planner.current_state['in_transition'].pop(arrival[0], None)

        if self.sensing_prob == 0:
            return {}

        # Iterate over agents that performed a move action
        for agent, location in self.online_planner.current_state['at_vertex'].items():
            if self.sensing_prob >= random.random():  # Agent can sense the current time
                sensed_agents[agent] = location
                self.final_solution.paths[agent].path[-1] = (self.sim_time, self.sim_time), location
            elif len(self.final_solution.paths[agent].path) > 1:
                final_path = self.final_solution.paths[agent].path
                waited = final_path[-1][1] == final_path[-2][1]  # If last location and one before that are equal
                no_tu = final_path[-1][0][1] - final_path[-1][0][0] == 0  # if the last movement's uncertainty is 0
                if waited and no_tu:
                    sensed_agents[agent] = location
                    self.final_solution.paths[agent].path[-1] = (self.sim_time, self.sim_time), location
        if self.communication:  # Send the info to the central planner.
            self.online_planner.update_current_state(self.sim_time, sensed_agents)

        return sensed_agents

    def execute_next_step(self):
        """
        Executes the next action for all agents that are at a vertex and updates the current state accordingly. Also
        inserts into the transitioning agents dict the ones that just performed an action that isn't waiting (and
        removes them from the list of agents that are at a vertex)
        :return: Update the list of arrival times and the online planner state
        """
        # Agents that are currently in a vertex
        for agent in list(self.online_planner.current_state['at_vertex'].keys()):
            action = self.get_next_action(agent)
            actual_time = self.get_actual_traversal_time(action)
            self.final_solution.paths[agent].path.append((action[2], action[1]))
            if action[0] != action[1]:  # Not a wait action, so not at vertex anymore
                self.online_planner.current_state['at_vertex'].pop(agent, None)
                self.online_planner.current_state['in_transition'][agent] = action
            if actual_time not in self.arrival_times:
                self.arrival_times[actual_time] = []
            self.arrival_times[actual_time].append((agent, action[1]))

    def get_actual_traversal_time(self, action):
        # Randomly choose real traversal time
        edge_weight = self.real_weights[action[0], action[1]]
        actual_time = edge_weight + self.sim_time
        return actual_time

    def at_goal_state(self):
        """
        Function for verifying if we are at a goal state and can halt the search.
        :return: True if goal state, False otherwise.
        """

        if self.sim_time in self.arrival_times:
            agent_num = len(self.tu_problem.start_positions)
            if agent_num == len(self.arrival_times[self.sim_time]):  # All agents are arriving somewhere
                for presence in self.arrival_times[self.sim_time]:
                    if presence[1] != self.tu_problem.goal_positions[presence[0]]:
                        return False

        for agent, loc in self.online_planner.current_state['at_vertex'].items():
            if loc != self.tu_problem.goal_positions[agent]:  # If even one agent is not at their goal state
                return False

        if len(self.online_planner.current_state['in_transition']) > 0:
            return False

        return True  # All agents are at their goal states

    def get_next_action(self, agent):
        """
        Extract from the current plan the next action that will be taken.
        :param agent: The agent for the needed action
        :return: The action done including possible completion times.
        """
        start_location = self.online_planner.current_plan.paths[agent].path[0][1]
        if len(self.online_planner.current_plan.paths[agent].path) >= 2:
            destination = self.online_planner.current_plan.paths[agent].path[1]
            self.online_planner.current_plan.paths[agent].path.pop(0)
            return start_location, destination[1], destination[0]
        else:  # Reached the end of the path.
            return start_location, start_location, (self.sim_time + 1, self.sim_time + 1)

    def print_final_solution(self):
        print('--------------------------------')
        print('Initial Path:')
        init_cost = self.online_planner.initial_plan.cost
        print(f'Initial Cost: {init_cost}')
        print(f'Initial Time Uncertainty: {init_cost[1] - init_cost[0]}')
        print('Final Path Taken:')
        print(f'Final Cost: {self.final_solution.cost}')
        print(f'Final Time Uncertainty: {self.final_solution.cost[1] - self.final_solution.cost[0]}')

    def create_initial_solution(self, min_best_case, soc, use_pc, use_bp, time_limit=300, initial_sol=None):
        self.online_planner.find_initial_path(min_best_case, soc, use_pc, use_bp, time_limit, initial_sol)

    def calc_solution_true_cost(self, solution):
        """
        Calculates the actual cost of the proposed solution if it were to be executed. Uses the real weights that the
        simulator keeps for each edge to do so.
        :param solution:
        :return:
        """

        true_cost = 0

        for agent, sol in solution.paths.items():
            prev_presence = sol.path[0]
            for curr_presence in sol.path[1:]:
                if prev_presence[1] != curr_presence[1]:  # It's an actual movement
                    try:
                        true_cost += self.real_weights[prev_presence[1], curr_presence[1]]
                    except KeyError:
                        print(prev_presence[1], curr_presence[1])
                else:  # it's a curr_move
                    true_cost += 1
                if curr_presence[1] == self.tu_problem.goal_positions[agent] and \
                        curr_presence[0] == solution.paths[agent].cost:
                    # agent reached the goal and doesn't need to move
                    break  # reached goal
                prev_presence = curr_presence

        return true_cost
