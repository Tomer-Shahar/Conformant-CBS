import copy

from pathfinding.planners.operator_decomposition_a_star import *
from pathfinding.planners.prioritized_planner import PrioritizedPlanner
from pathfinding.simulator import *
import profile
import random


def print_solution(solution):
    print()
    for agent, plan in solution.paths.items():
        print("Path for agent_" + str(agent) + ": ")
        for movement in plan.path:
            print(movement[1], end="")
            if plan.path[-1] != movement:
                print(" --> ", end="")
        print()
    print("Solution cost is between " + str(solution.cost[0]) + " and " + str(solution.cost[1]))
    print("Solution length is  " + str(solution.length) + "\n")


def run_map(problem, sic_heuristic=False, print_sol=True, time_limit=180, use_cbs=True, use_cat=True):
    for agent in range(1, len(problem.start_positions) + 1):
        print("Agent " + str(agent) + ": " + str(problem.start_positions[agent]) + ", to " +
              str(problem.goal_positions[agent]))
    print("Finding solution.")
    if use_cbs:
        if sic_heuristic:
            print("Using sum of costs measurement.")
        else:
            print("Finding shortest global time measurement.")
        if use_cat:
            print("Using Conflict Avoidance Table")
        else:
            print("Not using Conflict Avoidance Table")
        ccbs_planner = CBSTUPlanner(problem)
        solution = ccbs_planner.find_solution(soc=sic_heuristic, time_lim=time_limit, use_cat=use_cat)
        if print_sol:
            print_solution(solution)
    else:
        oda_solver = ODAStar(problem)
        solution = oda_solver.find_solution(time_limit=time_limit)

def run_test_map():
    print("----------- Small custom map ---------------")
    test_map = TimeUncertaintyProblem('../maps/test_map.map')
    test_map.generate_problem_instance(uncertainty=0)
    test_map.start_positions[1] = (0, 0)
    test_map.start_positions[2] = (17, 0)
    test_map.goal_positions[1] = (19, 0)
    test_map.goal_positions[2] = (17, 0)
    test_map.fill_heuristic_table()
    test_map.print_map()
    run_map(test_map, sic_heuristic=True, print_sol=True)
    run_map(test_map, sic_heuristic=False, print_sol=True)


map_file = '..\..\maps\\ost003d.map'
blank_problem = TimeUncertaintyProblem(map_file)
random.seed(1001)
blank_problem.generate_problem_instance(0)
blank_problem.generate_agents(35)
blank_problem.fill_heuristic_table(min_best_case=False)

cbstu_planner = CBSTUPlanner(blank_problem)
profile.run('sol = cbstu_planner.find_solution(min_best_case=True, time_lim=120, use_pc=True)', sort=1)

print('Done')

