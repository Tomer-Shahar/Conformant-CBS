"""
This class represents the solutions that are stored in each constraint node. They consist of conformant plans for each
agent, the total cost range of the solution and the length (i.e the max length between the different paths).
"""
import math
from pathfinding.conformant_plan import ConformantPlan

STAY_STILL_COST = 1


class ConformantSolution:

    def __init__(self):
        self.cost = math.inf, math.inf
        self.length = math.inf
        self.paths = {}
        self.tuple_solution = None

    def copy_solution(self, other_sol):

        if other_sol.tuple_solution:
            self.tuple_solution = {}

        for agent, plan in other_sol.paths.items():
            self.paths[agent] = plan
            if other_sol.tuple_solution:
                self.tuple_solution[agent] = other_sol.tuple_solution[agent]

    def compute_solution_cost(self, sum_of_costs=True):
        """
        Computes the cost for a given solution. Can return either the SIC or simply the maximum time
        of any path in the solution.
        """

        if sum_of_costs:
            min_cost = 0
            max_cost = 0
            for agent, plan in self.paths.items():
                if plan.path is None:
                    return math.inf, math.inf

                min_cost += plan.cost[0]
                max_cost += plan.cost[1]
            self.cost = min_cost, max_cost
        else:
            self.cost = self.get_max_of_min_path_time(), self.get_max_path_time()

    def get_max_of_min_path_time(self):
        """
        Returns the maximum between the minimum costs of the different paths in the solution.
        """
        max_min_time = self.paths[1].cost[0]
        for agent, plan in self.paths.items():
            if plan.cost[0] > max_min_time:
                max_min_time = plan.cost[0]
        return max_min_time

    def get_max_path_time(self):
        max_time = self.paths[1].cost[1]
        for agent, plan in self.paths.items():
            if plan.cost[1] > max_time:
                max_time = plan.cost[1]
        return max_time

    def create_movement_tuples(self):
        """
        converts each path in solution to a tuple of ( (t1,t2), (u,v) ) where (u,v) is an edge and t1 is when the agent
        began the movement across it and t2 is when the agent completed the movement.

        For easier comparison, 'u' and 'v' will be sorted.
        """
        self.tuple_solution = {}

        for agent, plan in self.paths.items():
            new_path = []
            for move in range(0, len(plan.path) - 1):
                start_vertex = min(plan.path[move][1], plan.path[move + 1][1])
                start_time = plan.path[move][0][0]
                next_vertex = max(plan.path[move][1], plan.path[move + 1][1])
                finish_time = plan.path[move + 1][0][1]
                new_path.append(((start_time, finish_time), (start_vertex, next_vertex)))

            self.tuple_solution[agent] = new_path

    def add_stationary_moves(self):
        """
        Appends to each path the time steps where the agent waits at his goal. We will normalize all the minimum times
        where the agents arrive at their goals, i.e if one agent arrived between (5,10) and another agent between
        (1,3), we will add stationary moves for the second agent so that it only finishes the plan when the time is
        (5, 7) -> the minimum arrival times are the same. This is because there is no point to a solution where, for
        example, only one agent arrives earlier than the rest, since even in an optimal case it'd have to wait.
        Note that staying at the goal once arriving does not add to the cost, in the case that the agent doesn't have
        to move anymore.
        """
        max_min_time = self.get_max_of_min_path_time()

        for agent, plan in self.paths.items():  # Iterate through all plans, fix each one if need be.
            new_path = plan.path
            last_move = plan.path[-1]
            path_min_time = last_move[0][0]
            path_max_time = last_move[0][1]
            if path_min_time < max_min_time:  # The agent is gonna stay at the end at the same position.
                for time_step in range(1, max_min_time - path_min_time + 1):
                    stationary_move = ((path_min_time + time_step * STAY_STILL_COST,
                                        path_max_time + time_step * STAY_STILL_COST), last_move[1])
                    new_path.append(stationary_move)
                # plan.cost = new_path[-1][0]
            self.paths[agent] = ConformantPlan(agent, new_path, plan.cost)

        self.length = len(self.paths[1].path)


"""
    def fill_in_solution(self):
        
        Converts a solution that contains only the vertices of the paths to a solution where the paths is filled with
        the movements where the agent is travelling an edge. i.e if a path is [(0,time=0),(4,time=3)], we know that the
        agent was on the edge (0,4) at time 1 and 2 -> [(0,time=0),((0,4),time=1),((0,4),time=2),(4,time=3)]
    
        max_time = self.get_max_path_time()

        for agent, plan in self.paths.items():
            filled_path = []
            for i in range(0, len(plan.path) - 1):
                curr_node = plan.path[i]
                next_node = plan.path[i + 1]
                if curr_node[0] + 1 == next_node[0]:  # i.e they are sequential
                    filled_path.append(curr_node)
                else:
                    filled_path.append(curr_node)
                    for time_interval in range(curr_node[0] + 1, next_node[0]):
                        edge_move = (time_interval, (curr_node[1], next_node[1]))
                        filled_path.append(edge_move)
            last_move = plan.path[-1]
            filled_path.append(last_move)
            if last_move[0] < max_time:  # The agent is gonna stay at the end at the same position.
                curr_time = len(filled_path)
                for time_interval in range(curr_time, max_time + 1):
                    stationary_move = (time_interval, last_move[1])
                    filled_path.append(stationary_move)
            new_path = ConformantPlan(agent, filled_path, max_time)
            self.paths[agent] = new_path
"""

