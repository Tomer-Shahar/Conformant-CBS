import copy

from pathfinding.planners.utils.map_reader import TimeUncertaintyProblem
from pathfinding.planners.cbstu import *
from pathfinding.planners.operator_decomposition_a_star import *
from pathfinding.simulator import *
import os
import profile
import random
import json

seed = 1234541
random.seed(seed)
print(f'The seed is {seed}')


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
        solution = ccbs_planner.find_solution(soc=sic_heuristic, time_limit=time_limit, use_cat=use_cat)
        if print_sol:
            print_solution(solution)
    else:
        oda_solver = ODAStar(problem)
        solution = oda_solver.create_solution(time_limit=time_limit)


def print_map(conf_problem):
    if os.name == 'nt':
        print('The map being run:')
        grid = copy.deepcopy(conf_problem.map)

        for agent, position in conf_problem.start_positions.items():
            grid[position[0]][position[1]] = chr(ord('A') - 1 + agent) + ' '

        for agent, position in conf_problem.goal_positions.items():
            grid[position[0]][position[1]] = str(agent) + ' '

        for row in grid:
            for cell in row:
                if cell == 1:
                    print("■ ", end="")
                elif cell == 0:
                    print("□ ", end="")
                else:
                    print(cell, end="")
            print("")
        print("--------------------------------------------")
    else:
        pass


def run_test_map():
    print("----------- Small custom map ---------------")
    test_map = TimeUncertaintyProblem('../maps/test_map.map')
    test_map.generate_problem_instance(uncertainty=0)
    test_map.start_positions[1] = (0, 0)
    test_map.start_positions[2] = (17, 0)
    test_map.goal_positions[1] = (19, 0)
    test_map.goal_positions[2] = (17, 0)
    test_map.fill_heuristic_table()
    print_map(test_map)
    run_map(test_map, sic_heuristic=True, print_sol=True)
    run_map(test_map, sic_heuristic=False, print_sol=True)

"""
def save_solution(solution, tu_problem, uncertainty, soc, min_best_case):
    start = [v for v in tu_problem.start_positions.items()]
    goals = [v for v in tu_problem.goal_positions.items()]
    path = f'.\\previous_solutions\\sol_soc={soc}_mbc={min_best_case}_start={start}_goals={goals}_' \
        f'uncertainty={uncertainty}.sol'

    with open(path, 'w+') as sol_file:
        json_sol = {'paths': {}, 'constraints': None}
        for agent, path in solution.paths.items():
            json_sol['paths'][agent] = path.path
        json_sol['constraints'] = list(solution.constraints)
        json.dump(json_sol, sol_file)


def check_if_solution_exists(tu_problem, uncertainty, soc, min_best_case):
    start = [v for v in tu_problem.start_positions.items()]
    goals = [v for v in tu_problem.goal_positions.items()]
    path = f'.\\previous_solutions\\sol_soc={soc}_mbc={min_best_case}_start={start}_goals={goals}_' \
        f'uncertainty={uncertainty}.sol'
    if not os.path.exists(path):
        return None

    with open(path, 'r') as sol_file:
        json_sol = json.load(sol_file)
        tu_sol = TimeUncertainSolution()
        for agent, path in json_sol['paths'].items():
            tuple_path = []
            for presence in path:
                tuple_path.append((tuple(presence[0]), tuple(presence[1])))
            tu_plan = TimeUncertainPlan(int(agent), tuple_path, math.inf)
            tu_sol.paths[int(agent)] = tu_plan
        for con in json_sol['constraints']:
            tuple_con = con[0], tuple(con[1]), tuple(con[2])
            tu_sol.constraints.add(tuple_con)

        tu_sol.compute_solution_cost()
        tu_sol.create_movement_tuples()
        return tu_sol
"""
# profile.run('run_test_map()', sort=1)
total_start = time.time()

# print("-------------- Large moving-ai maps -------------------")
# complex_map = TimeUncertaintyProblem('../maps/ost003d.map')
# print("Parsing (round, slightly narrow) map..")
# complex_map.generate_problem_instance(uncertainty=0)
# complex_map.generate_agents(10)
# print("Filling heuristics table..")
# complex_map.fill_heuristic_table()
# profile.run('run_map(complex_map, print_sol=False, time_limit=60, use_cbs=True, use_cat=False)', sort=1)


tu_problem = TimeUncertaintyProblem('./test_map.map')
uncertainty = 2
tu_problem.generate_problem_instance(uncertainty=uncertainty)
tu_problem.start_positions[1] = (0, 0)
tu_problem.start_positions[2] = (17, 0)
tu_problem.goal_positions[1] = (19, 0)
tu_problem.goal_positions[2] = (17, 0)
tu_problem.fill_heuristic_table()

sim = MAPFSimulator(tu_problem, sensing_prob=0.1)
soc = True
min_best_case = False
comm = False
planner = CBSTUPlanner(tu_problem)
#sol = planner.find_solution()
#sol.save_solution(tu_problem, uncertainty, soc, min_best_case, '.\\previous_solutions')
sol = TimeUncertainSolution.load_solution(tu_problem, uncertainty, soc, min_best_case, '.\\previous_solutions')
sim.begin_execution(min_best_case=min_best_case, soc=soc, time_limit=1000, communication=comm, initial_sol=sol)
final_path = sim.final_solution
print('Done')

