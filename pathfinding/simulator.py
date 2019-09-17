"""
Simulator that runs an online version of CBSTU, with sensing. It executes the initial plan and periodically updates the
planner with new information gained. The simulator imitates this by giving each agent a chance to sense the current
time. This occurs when the agent is at a node (waiting or having just arrived).
"""

from pathfinding.planners.utils.time_uncertainty_solution import *
from pathfinding.planners.online_cbstu import *
from pathfinding.planners.utils.map_reader import *
import random


class MAPFSimulator:

    def __init__(self, tu_problem, sensing_prob=1):
        """
        :param tu_problem: A time uncertainty problem
        :param sensing_prob: The probability to sense at each node

        """

        self.tu_problem = tu_problem
        self.sensing_prob = sensing_prob
        self.online_CSTU = OnlineCBSTU(tu_problem)  # The online solver
        self.final_solution = TimeUncertainSolution()  # Maintain the final path taken by agents
        self.arrival_times = {}  # Maintain a dictionary of when agents will arrive to their next destination
        self.fixed_weights = {}  # The actual traversal times
        for agent in self.tu_problem.start_positions:
            self.final_solution.paths[agent] = TimeUncertainPlan(agent,
                                                                 [((0, 0), self.tu_problem.start_positions[agent])]
                                                                 , (0, 0))

    def begin_execution(self, communication=True):
        """
        The main function of the simulator. Finds an initial solution and runs it, occasionally sensing and broadcasting
        if need be.
        :param communication: Mode of cooperation. Irrelevant if there's no sensing. Otherwise, it's either full
        cooperation (True) or no cooperation (False). Full cooperation means agents can communicate with each other. No
        cooperation means agents replan for themselves only.

        :return: The path that was ultimately taken for each agent.
        """

        self.populate_initial_solution()
        time = 0

        if not communication:
            graphs, constraints = self.create_plan_graphs_and_constraints()

        while True:
            actions_taken = self.execute_next_step(time)
            if self.at_goal_state():
                break
            self.simulate_sensing_and_broadcast(time, actions_taken)
            self.online_CSTU.create_new_plans()
            time += 1
        self.print_final_solution(time)

    def execute_next_step(self, time):
        """
        Executes the next action for all agents that are at a vertex and updates the current state accordingly. Also
        inserts into the transitioning agents dict the ones that just performed an action that isn't waiting (and
        removes them from the list of agents that are at a vertex)
        :return:
        """
        actions_taken = {}  # Agents that are at a vertex and are making an action (including wait)

        # Iterate over agents that were on the move and randomly decide if they already arrived to their destination
        if time in self.arrival_times:
            for arrival in self.arrival_times[time]:
                self.online_CSTU.current_state['at_vertex'][arrival[0]] = arrival[1]
                self.online_CSTU.current_state['in_transition'].pop(arrival[0], None)

        for agent in list(self.online_CSTU.current_state['at_vertex'].keys()):  # Agents that are currently in a vertex
            action = self.get_next_action(agent, time)
            actual_time = random.randint(max(time + 1, action[2][0]),
                                         action[2][1])  # Randomly choose real traversal time
            actions_taken[agent] = action
            self.final_solution.paths[agent].path.append((action[2], action[1]))
            if action[0] != action[1]:  # Not a wait action
                self.online_CSTU.current_state['at_vertex'].pop(agent, None)
                self.online_CSTU.current_state['in_transition'][agent] = action
                if actual_time not in self.arrival_times:
                    self.arrival_times[actual_time] = []
                self.arrival_times[actual_time].append((agent, action[1]))
        # for agent_action in self.arrival_times[time]:  # Agents that finished traversing
        #    self.online_CSTU.current_state['at_vertex'][agent_action[0]] = agent_action[1]

        return actions_taken

    def simulate_sensing_and_broadcast(self, time, actions_taken):

        for agent, action in actions_taken.items():  # Iterate over agents that performed a move action
            if self.sensing_prob > random.random():  # Agent can sense their arrival time
                actual_time = random.randint(action[2][0], action[2][1])  # Randomly choose real traversal time
                self.arrival_times[actual_time].append(agent, action[1])

        self.online_CSTU.update_current_state(time)

        pass

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

    def get_next_action(self, agent, time):
        """
        Extract from the current plan the next action that will be taken.
        :param time: The current time tick in execution. Useful when agents reach their goal and wait.
        :param agent: The agent for the needed action
        :return: The action done including possible completion times.
        """
        start_location = self.online_CSTU.current_plan.paths[agent].path[0][1]
        if len(self.online_CSTU.current_plan.paths[agent].path) >= 2:
            destination = self.online_CSTU.current_plan.paths[agent].path[1]
            self.online_CSTU.current_plan.paths[agent].path.pop(0)
            return start_location, destination[1], destination[0]
        else:  # Reached the end of the path.
            return start_location, start_location, (time + 1, time + 1)

    def create_plan_graphs_and_constraints(self):
        """
        Create a personal graph for each agent based on its initial solution. This is because when an agent replans
        without cooperation, it has to follow the original path it had. This ensures no collisions occur.
        :return: Possible graphs for each agent and constraints.
        """

        agent_graphs = {}  # Maps between an agent and their valid path
        constraints = {}  # Maps between an agent and constraints RELEVANT TO THEM!!

        self.online_CSTU.initial_plan.create_movement_tuples()
        for agent, plan in self.online_CSTU.initial_plan.paths.items():
            new_agent_graph, traversed_locations = self.generate_tu_problem_graph(agent)

            agent_graphs[agent] = new_agent_graph

        return agent_graphs, constraints

    def generate_tu_problem_graph(self, agent):
        """
        Converts an agent's found path to a graph.
        :param agent: The agent we're building a graph for
        :return: a TimeUncertaintyProblem object. The edges for it will be the same edges traversed by the agent.
        """

        agent_path_graph = TimeUncertaintyProblem()
        prev_move = self.online_CSTU.initial_plan.tuple_solution[agent][0]
        self.online_CSTU.initial_plan.tuple_solution[agent].pop(0)
        agent_path_graph.edges_and_weights = {}
        traversed_locations = {prev_move[1][0]: {(agent, (0, 0))}}

        for move in self.online_CSTU.initial_plan.tuple_solution[agent]:
            weight = (move[0][0] - prev_move[0][0], move[0][1] - prev_move[0][1])
            edge = move[1]
            if edge not in traversed_locations:
                traversed_locations[edge] = set()
            traversed_locations[edge].add((agent, (move[0][0], move[0][1]-1)))

            if move[1][0] not in traversed_locations:
                traversed_locations[move[1][0]] = set()
            traversed_locations[move[1][0]].add((agent, move[0]))

            if edge[0] not in agent_path_graph.edges_and_weights:
                agent_path_graph.edges_and_weights[edge[0]] = set()

            agent_path_graph.edges_and_weights[edge[0]].add((edge[1], weight))

            if edge[1] not in agent_path_graph.edges_and_weights:
                agent_path_graph.edges_and_weights[edge[1]] = set()

            agent_path_graph.edges_and_weights[edge[1]].add((edge[0], weight))

            prev_move = move

        return agent_path_graph, traversed_locations

    def print_final_solution(self, time):

        print('Initial Path:')
        print(f'Total time: {self.online_CSTU.initial_plan.cost}')

        print('Final Path Taken:')
        print(f'Total time: {time}')
        print(f'Total Cost: {self.final_solution.cost}')

    def populate_initial_solution(self):
        self.online_CSTU.find_initial_path()
        self.final_solution.cost = self.online_CSTU.initial_plan.cost

        for agent, plan in self.online_CSTU.initial_plan.paths.items():
            self.final_solution.paths[agent].cost = plan.cost

    def comp_weights(self):
        """
        Create fixed weights for the graph.
        :return:
        """

        for vertex_1, edges in self.tu_problem.edges_and_weights.items():
            self.fixed_weights[vertex_1] = []
            for vertex_2 in edges:
                self.fixed_weights[vertex_1].append((vertex_2[0], random.randint(vertex_2[1][0], vertex_2[1][1])))
