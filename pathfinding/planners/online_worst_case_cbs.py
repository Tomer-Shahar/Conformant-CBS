"""
A trivial online planner that utilizes CBS to solve graphs with time uncertainty. A TU problem is given as input, and a
solution is found for the case where each edge has the maximal time possible. During execution, after traversing an edge
and sensing, an agent waits the appropriate number of time steps in order to synchronize with the initial plan.
Note that this algorithm does not require any communication, but it does require full sensing.
"""

from pathfinding.planners.cbstu import CBSTUPlanner
import copy


class OnlinePessimisticCBS:

    def __init__(self, tu_problem):
        self.tu_problem = tu_problem
        self.convert_to_worst_case()
        self.init_sol = None
        self.curr_sol = None
        self.current_state = None
        self.traversing_agents = set()  # Used to monitor which agents arrived at a node for the first time

    def convert_to_worst_case(self):
        for vertex, edges in self.tu_problem.edges_and_weights:
            new_edges = []
            for edge in edges:
                new_edges.push = edge[0], (edge[1][1], edge[1][1])

            self.tu_problem[vertex] = new_edges

    def find_initial_path(self, min_best_case=False, soc=True, time_limit=60, initial_sol=None):

        cbstu_planner = CBSTUPlanner(self.tu_problem)
        self.init_sol = cbstu_planner.find_solution(min_best_case, time_limit, soc)
        self.curr_sol = copy.deepcopy(self.init_sol)
        self.current_state = {'time': 0,
                              'cost': 0,                    # Agents that are at a vertex
                              'at_vertex': copy.deepcopy(self.tu_problem.start_positions),
                              'in_transition': {}}  # Agents that are transitioning.
        return self.init_sol

    def plan_distributed(self, graphs, constraints, sensing_agents, time):

        """
        Receives all the agents that finished traversing and adds the necessary wait actions to the current plan.
        :param time: The current time in the simulator
        :param sensing_agents: The agents that sensed (i.e. the ones that are at a vertex)
        :return: updates the current plan.
        """

        for agent, loc in sensing_agents:
            curr_move = self.curr_sol.tuple_solution[agent][0]
            if curr_move[1][0] != curr_move[1][1]:  # It's a move action
                time_to_wait = curr_move[1] - time
                for i in range(time_to_wait):
                    wait_action = ()
                    new_plan = self.curr_sol.paths[agent].path.push()


