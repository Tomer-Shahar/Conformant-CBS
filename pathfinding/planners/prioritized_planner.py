"""
A simple implementation of a prioritized planner that can deal with time uncertainty.
"""


from pathfinding.planners.utils.time_uncertainty_solution import TimeUncertaintySolution
from pathfinding.planners.constraint_A_star import *
from collections import defaultdict


class PrioritizedPlanner:

    def __init__(self, tu_problem):
        self.tu_problem = tu_problem
        self.planner = ConstraintAstar(tu_problem)
        self.start_time = 0
        self.curr_time = (0, 0)
        self.curr_cons = defaultdict(list)

    def find_solution(self, min_best_case=False, time_limit=60, soc=True, existing_cons=None, curr_time=(0, 0),
                      to_print=False):
        """
        Finds a solution for a mapf-tu problem using Silver's (2005) prioritized planner.
        """
        if existing_cons:
            self.curr_cons = existing_cons
        self.start_time = time.time()
        self.curr_time = curr_time
        final_solution = TimeUncertaintySolution()
        for agent, start in self.tu_problem.start_positions.items():
            time_passed = time.time() - self.start_time
            if time_passed > time_limit:
                return OutOfTimeError()
            begin_time = time.time()
            agent_path = self.planner.compute_agent_path(self.curr_cons, agent, start,
                                                         self.tu_problem.goal_positions[agent], set(), min_best_case,
                                                         time_limit=time_limit - time_passed, curr_time=self.curr_time,
                                                         suboptimal=True)
            if to_print:
                print(f'Time to compute agent {agent}\'s path: {time.time() - begin_time}')
            if not agent_path.path:
                return TimeUncertaintySolution.empty_solution()
            final_solution.paths[agent] = agent_path
            final_solution.add_stationary_moves()
            final_solution.create_movement_tuples()
            self.extract_cons(final_solution, agent)

        final_solution.time_to_solve = time.time() - self.start_time
        final_solution.compute_solution_cost(sum_of_costs=soc)
        return final_solution

    def extract_cons(self, solution, agent):
        """
        Convert an agent's path into constraints for future agents where a constraint = (vertex, time). No need for
        agent since this constraint applies to all future agents.
        :param solution: Agent's solution
        :param agent: the agent
        :return: Updates self.curr_cons
        """

        for move in solution.paths[agent].path:
            self.curr_cons[move[1]].append(move[0])

        for move in solution.tuple_solution[agent]:
            if move[0][1] - move[0][0] == 1:  # instantaneous travel
                t = move[0][1], move[0][1]
                self.curr_cons[move[1]].append(t)
            else:
                for tick in range(move[0][0]+1, move[0][1]):
                    self.curr_cons[move[1]].append((tick, tick))


