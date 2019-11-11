"""
An online variant of cbstu. During plan execution, the agents receive information about their current time and other
agent's current state. This will allow to re-plan with additional information, hopefully leading to higher quality
results in terms of Sum of Costs.
"""
import copy
import time

from pathfinding.planners.cbstu import CBSTUPlanner
from pathfinding.planners.constraint_A_star import ConstraintAstar
from pathfinding.planners.utils.map_reader import TimeUncertaintyProblem


class OnlineCBSTU:

    def __init__(self, tu_problem, full_sensing=False):
        """
        Initialize the planner.
        :param tu_problem: a time-uncertainty problem. This is an object that must be capable of updating the current
        state of the world.
        """
        self.tu_problem = tu_problem
        self.offline_cbstu_planner = CBSTUPlanner(tu_problem)
        self.initial_plan = None
        self.current_plan = None
        self.current_state = None
        self.full_sensing = full_sensing

    def find_initial_path(self, min_best_case=False, soc=True, time_limit=300, initial_sol=None):
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
            start = time.time()
            self.initial_plan = self.offline_cbstu_planner.find_solution(min_best_case, time_limit, soc)
            total_time = time.time() - start
            self.initial_plan.time_to_solve = total_time
            self.current_plan = copy.deepcopy(self.initial_plan)
        self.current_state = {'time': 0,
                              'cost': 0,  # Agents that are at a vertex
                              'at_vertex': copy.deepcopy(self.tu_problem.start_positions),
                              'in_transition': {}}  # Agents that are transitioning.

    def update_current_state(self, curr_time, sensed_agents):
        """
        Extract new info from the current state, such as where each agent is and what the current time is. Both provide
        useful information. Only called when there is communication amongst agents. For each sensing agent, we can
        already update all the future potential presences to reflect the new data. If we are using full sensing, we can
        even update the information for the agents that haven't currently sensed.

        :param curr_time: Current time in the simulation
        :param sensed_agents: A dictionary of agents that have sensed
        :return: The offline planner now has an up to date view of the current state.
        """
        self.current_state['time'] = curr_time

        traversing_agents = set(self.tu_problem.start_positions.keys()) - set(sensed_agents.keys())

        for agent in self.tu_problem.start_positions:
            curr_presence = self.current_plan.paths[agent].path[0]
            if curr_presence[0][1] - curr_presence[0][0] == 0:  # There was no uncertainty to begin with.
                continue

            if agent in sensed_agents:  # We can update the future path based on this sense
                time_inc = curr_time - curr_presence[0][0]
                time_dec = curr_presence[0][1] - curr_time
                for idx, presence in enumerate(self.current_plan.paths[agent].path):
                    new_presence = (presence[0][0] + time_inc, presence[0][1] - time_dec), presence[1]
                    self.current_plan.paths[agent].path[idx] = new_presence

            # If we have full sense, we can learn about traversing agents as well
            if agent in traversing_agents and self.full_sensing:
                time_diff = curr_time + 1 - curr_presence[0][0]
                presence = self.current_plan.paths[agent].path[0]
                new_presence = (presence[0][0] + time_diff, presence[0][1]), presence[1]
                prev_node = self.current_state['in_transition'][agent][0]
                # At the earliest, the agent will arrive only next time step, so we can update the current plan.
                self.current_state['in_transition'][agent] = prev_node, new_presence[1], new_presence[0]

                for idx, presence in enumerate(self.current_plan.paths[agent].path):
                    new_presence = (presence[0][0] + time_diff, presence[0][1]), presence[1]
                    self.current_plan.paths[agent].path[idx] = new_presence

    def create_new_centralized_plan(self, curr_time, sensing_agents):
        """
        Replan for the agents that updated their location. Must first create a large set of constraints for all agents
        that are currently replanning.

        :return: New plans for the agents that are at a vertex.
        """

        unsensed_agents = set(self.tu_problem.start_positions.keys()) - set(sensing_agents.keys())
        if self.current_plan.cost[1] - self.current_plan.cost[0] > 0:
            new_cons = set()
            for unsensing_agent in unsensed_agents:  # Agents that haven't sensed
                for move in self.current_plan.paths[unsensing_agent].path:
                    for planning_agent, presence in sensing_agents.items():
                        new_cons.add((planning_agent, move[1], move[0]))

            self.offline_cbstu_planner.startPositions = sensing_agents
            new_plans = self.offline_cbstu_planner.find_solution(existing_cons=new_cons, curr_time=(curr_time, curr_time))
            for agent, path in new_plans.paths.items():
                self.current_plan.paths[agent] = path
            self.current_plan.add_stationary_moves({agent: self.current_plan.paths[agent] for agent in sensing_agents})

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

    def plan_distributed(self, graphs, constraints, sensing_agents, curr_time):
        """
        Lets agents replan after sensing, but without communication. Each agent plans for itself while taking into
        account possible constraints.
        :param curr_time: current time
        :param graphs: the path that each agent can plan in.
        :param constraints: A set of constraints for all agents
        :param sensing_agents: Agents that performed a sensing action
        :return: Creates new plans for each agent that sensed by updating the current plan.
        """

        if self.current_plan.cost[1] - self.current_plan.cost[0] > 0:
            for agent, loc in sensing_agents.items():
                agent_constraints = self.offline_cbstu_planner.final_constraints | constraints[agent]
                planner = ConstraintAstar(graphs[agent])
                new_plan = planner.compute_agent_path(agent_constraints, agent, loc, graphs[agent].goal_positions[agent],
                                                      set(), False, curr_time=(curr_time, curr_time))
                self.current_plan.paths[agent] = new_plan

            self.current_plan.add_stationary_moves({agent: self.current_plan.paths[agent] for agent in sensing_agents})

    def create_plan_graphs_and_constraints(self):
        """
        Create a personal graph for each agent based on its initial solution and a personal set of constraints.
        This is because when an agent replans without cooperation, it has to follow the original path it had. This
        ensures no collisions occur.

        :return: Possible graphs for each agent and constraints.
        """

        agent_graphs = {}  # Maps between an agent and their valid path
        constraints = {}  # Maps between an agent and constraints RELEVANT TO THEM!!
        traversed_locations = {}  # All locations that have been traversed in the solution.
        # Create the graph for each agent
        for agent, plan in self.initial_plan.paths.items():
            new_agent_graph, traversed_locations = self.generate_tu_problem_graph(agent, traversed_locations)
            new_agent_graph.fill_heuristic_table()
            agent_graphs[agent] = new_agent_graph
            constraints[agent] = set()
        # Update their respective constraints
        for agent, tu_plan in self.initial_plan.paths.items():
            for move in tu_plan.path:
                for presence in traversed_locations[move[1]]:
                    if presence[0] != agent:
                        constraints[agent].add((agent, move[1], presence[1]))

        return agent_graphs, constraints

    def generate_tu_problem_graph(self, agent, traversed_locs):
        """
        Converts an agent's found path to a graph.
        :param traversed_locs:
        :param agent: The agent we're building a graph for
        :return: a TimeUncertaintyProblem object. The edges for it will be the same edges traversed by the agent.
        """

        agent_path_graph = TimeUncertaintyProblem()
        agent_path_graph.start_positions[agent] = self.initial_plan.paths[agent].path[0][1]
        agent_path_graph.goal_positions[agent] = self.initial_plan.paths[agent].path[-1][1]
        prev_loc = self.initial_plan.paths[agent].path[0]
        agent_path_graph.edges_and_weights = {}
        self.safe_add_to_dict(prev_loc[1], (agent, (0, 0)), traversed_locs)  # add the vertex

        for loc in self.initial_plan.paths[agent].path[1:]:
            edge_weight = (loc[0][0] - prev_loc[0][0], loc[0][1] - prev_loc[0][1])

            # add the vertex to edge dictionary. Must add the edge in both directions
            self.safe_add_to_dict(prev_loc[1], (loc[1], edge_weight), agent_path_graph.edges_and_weights)
            self.safe_add_to_dict(loc[1], (prev_loc[1], edge_weight), agent_path_graph.edges_and_weights)

            # Add the edge and node to traversed locations
            self.safe_add_to_dict(loc[1], (agent, loc[0]), traversed_locs)

            if edge_weight != (1, 1):  # We don't add edges that were traversed instantly.
                self.safe_add_to_dict((prev_loc[1], loc[1]), (agent, (prev_loc[0][0] + 1, loc[0][1] - 1)),
                                      traversed_locs)

            prev_loc = loc

        return agent_path_graph, traversed_locs

    @staticmethod
    def safe_add_to_dict(key, value, dict_to_insert):
        """
        Adds an item to a given dictionary safely. First check if it's there, and if not creates an empty set and adds
        it to it.
        """
        if key not in dict_to_insert:
            dict_to_insert[key] = set()
        dict_to_insert[key].add(value)
