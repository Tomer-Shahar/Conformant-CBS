"""
A simple implementation of a prioritized planner that can deal with time uncertainty.
"""

import time
import math

from pathfinding.planners.utils.time_error import OutOfTimeError
from pathfinding.planners.utils.time_uncertainty_solution import TimeUncertainSolution
from pathfinding.planners.constraint_A_star import *


class PrioritizedPlanner:

    def __init__(self, tu_problem):
        self.tu_problem = tu_problem
        self.planner = ConstraintAstar(tu_problem)
        self.start_time = 0
        self.curr_time = (0, 0)
        self.curr_cons = set()

    def find_solution(self, min_best_case=False, time_limit=60, soc=True, existing_cons=None, curr_time=(0, 0)):
        """
        Finds a solution for a mapf-tu problem using Silver's (2005) prioritized planner.
        """
        if existing_cons:
            self.curr_cons = existing_cons
        self.start_time = time.time()
        self.curr_time = curr_time
        final_solution = TimeUncertainSolution()
        for agent, start in self.tu_problem.start_positions.items():
            time_passed = time.time() - self.start_time
            if time_passed > time_limit:
                return OutOfTimeError()
            begin_time = time.time()
            agent_path = self.planner.compute_agent_path(self.curr_cons, agent, start,
                                                         self.tu_problem.goal_positions[agent], set(), min_best_case,
                                                         time_limit=time_limit - time_passed, curr_time=self.curr_time,
                                                         suboptimal=True)
            print(f'Time to compute agent {agent}\'s path: {time.time() - begin_time}')
            if not agent_path.path:
                return TimeUncertainSolution.empty_solution()
            final_solution.paths[agent] = agent_path
            final_solution.add_stationary_moves()
            final_solution.create_movement_tuples()
            self.extract_cons(final_solution, agent)

        final_solution.compute_solution_cost(sum_of_costs=soc)
        return final_solution

    def extract_cons(self, solution, agent):
        """
        Convert an agent's path into constraints for future agents.
        constraint = (agent, vertex, time)
        """
        agent_to_constrain = {x for x in self.tu_problem.start_positions.keys() if x > agent}
        for move in solution.paths[agent].path:
            for con_agent in agent_to_constrain:
                self.curr_cons.add((con_agent, move[1], move[0]))

        for move in solution.tuple_solution[agent]:
            for con_agent in agent_to_constrain:
                if move[0][1] - move[0][0] == 1:  # instantaneous travel
                    t = move[0][1], move[0][1]
                    self.curr_cons.add((con_agent, move[1], t))
                else:
                    for tick in range(move[0][0]+1, move[0][1]):
                        self.curr_cons.add((con_agent, move[1], (tick, tick)))


