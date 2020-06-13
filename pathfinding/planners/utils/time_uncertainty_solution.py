"""
This class represents the solutions that are stored in each constraint node. They consist of conformant plans for each
agent, the total cost range of the solution and the length (i.e the max length between the different paths).
"""
import math
import json
import os
from pathfinding.planners.utils.time_uncertainty_plan import TimeUncertaintyPlan
from collections import defaultdict

STAY_STILL_COST = 1


class TimeUncertaintySolution:

    def __init__(self):
        self.cost = math.inf, math.inf
        self.paths = {}
        self.tuple_solution = {}
        self.nodes_generated = -1
        self.constraints = defaultdict(list)
        self.time_to_solve = -1
        self.sic = -1, -1

    @staticmethod
    def empty_solution(nodes_generated=-1):
        empty_sol = TimeUncertaintySolution()
        empty_sol.nodes_generated = nodes_generated
        return empty_sol

    def copy_solution(self, other_sol):

        if other_sol.tuple_solution:
            self.tuple_solution = {}
        self.time_to_solve = other_sol.time_to_solve

        for agent, plan in other_sol.paths.items():
            self.paths[agent] = plan
            if other_sol.tuple_solution:
                self.tuple_solution[agent] = other_sol.tuple_solution[agent]

        self.cost = other_sol.cost

    def compute_solution_cost(self, sum_of_costs=True):
        """
        Computes the cost for a given solution. Can return either the SIC or simply the maximum time
        of any path in the solution.
        """

        if sum_of_costs:
            min_cost = 0
            max_cost = 0
            for agent, plan in self.paths.items():
                if not plan.path:
                    return math.inf, math.inf
                plan.compute_cost()
                min_cost += plan.cost[0]
                max_cost += plan.cost[1]
            self.cost = min_cost, max_cost
        else:
            self.cost = self.get_max_of_min_path_time(), self.get_max_path_time()

    def get_max_of_min_path_time(self):
        """
        Returns the maximum between the minimum costs of the different paths in the solution.
        """

        for agent, path in self.paths.items():
            max_min_time = path.cost[0]
            break

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

    def create_movement_tuples(self, agents=None):
        """
        converts each path in solution to a tuple of ( (t1,t2), (u,v) ) where (u,v) is an edge and t1 is when the agent
        began the movement across it and t2 is when the agent completed the movement.

        For easier comparison, 'u' and 'v' will be sorted. We still maintain 'f' or 'b' to signify what was the original
        direction. This is necessary in a few very specific conflicts.
        """
        agents = self.paths.keys() if not agents else agents
        for agent in agents:
            path = self.paths[agent].path
            new_path = []
            for move in range(0, len(path) - 1):
                start_vertex = min(path[move][1], path[move + 1][1])
                start_time = path[move][0][0]
                next_vertex = max(path[move][1], path[move + 1][1])
                finish_time = path[move + 1][0][1]
                if start_vertex == path[move][1]:
                    direction = 'f'  # The original beginning vertex was 'start_vertex'
                else:
                    direction = 'b'  # The original beginning vertex was 'next_vertex'

                new_path.append(((start_time, finish_time), (start_vertex, next_vertex), direction))

            self.tuple_solution[agent] = new_path

    def add_stationary_moves(self, agents_to_update=None):
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
        new_moves = set()
        if agents_to_update is None:
            agents_to_update = self.paths
        for agent, plan in agents_to_update.items():  # Iterate through all plans, fix each one if need be.
            try:
                last_move = plan.path[-1]
            except IndexError:
                print(plan.path)
            path_min_time = last_move[0][0]

            if path_min_time < max_min_time:  # The agent is gonna stay at the end at the same position.
                #if self.paths[agent].path[-1] =
                self.paths[agent].path.append(((path_min_time + 1, max_min_time), last_move[1]))
                new_moves.add((agent, (path_min_time + 1, max_min_time), last_move[1]))
        return new_moves

    def save(self, agent_num, uncertainty, map_type, agent_seed, map_seed, min_best_case, use_pc, use_bp, folder):
        solution_path = os.path.join(folder, map_type, str(agent_num) + ' agents')
        if not os.path.exists(solution_path):
            os.makedirs(solution_path)
        objective = 'min best case' if min_best_case else 'min worst case'
        file_name = f'map seed {map_seed}_{agent_num} agents_agent seed {agent_seed}_{uncertainty} uncertainty_' \
            f'{objective}_using pc {use_pc}_using bypass {use_bp}, {map_type}.sol'
        path = os.path.join(solution_path, file_name)

        with open(path, 'w+') as sol_file:
            json_sol = {'paths': {}, 'constraints': None, 'time_to_solve': self.time_to_solve, 'sic': self.sic,
                        'nodes_generated': self.nodes_generated}
            for agent, path in self.paths.items():
                json_sol['paths'][agent] = path.path
            json_sol['constraints'] = list(self.constraints.items())
            json.dump(json_sol, sol_file)

    @staticmethod
    def load(agent_num, uncertainty, map_type, agent_seed, map_seed, min_best_case, use_pc, use_bp, folder):
        """
        Loads a previously computed solution.
        """
        objective = 'min best case' if min_best_case else 'min worst case'
        file_name = f'map seed {map_seed}_{agent_num} agents_agent seed {agent_seed}_{uncertainty} uncertainty_' \
            f'{objective}_using pc {use_pc}_using bypass {use_bp}, {map_type}.sol'
        path = os.path.join(folder, map_type, f'{agent_num} agents', file_name)

        if not os.path.exists(path):
            return None
        try:
            with open(path, 'r') as sol_file:
                json_sol = json.load(sol_file)
                tu_sol = TimeUncertaintySolution()
                for agent, path in json_sol['paths'].items():
                    tuple_path = []
                    for presence in path:
                        tuple_path.append((tuple(presence[0]), tuple(presence[1])))
                    tu_plan = TimeUncertaintyPlan(int(agent), tuple_path, math.inf)
                    tu_sol.paths[int(agent)] = tu_plan
                for con in json_sol['constraints']:
                    if type(con[0][0]) == int:  # it's a vertex constraint
                        loc = tuple(con[0])
                    elif type(con[0][0]) == list:  # it's an edge constraint
                        loc = tuple(con[0][0]), tuple(con[0][1])
                    else:
                        raise TypeError
                    tuple_con = con[1][0][0], tuple(con[1][0][1])
                    tu_sol.constraints[loc].append(tuple_con)

                tu_sol.time_to_solve = json_sol['time_to_solve']
                tu_sol.nodes_generated = json_sol['nodes_generated']
                tu_sol.compute_solution_cost()
                tu_sol.create_movement_tuples()
                return tu_sol
        except:
            os.remove(path)  # Delete the messed up file
            return None

