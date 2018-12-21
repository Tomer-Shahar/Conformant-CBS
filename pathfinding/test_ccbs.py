from pathfinding.map_reader import ConformantProblem
from pathfinding.conformant_cbs import *
import os
import profile


def print_solution(solution):
    print("")
    for agent, plan in solution[0].items():
        print("Path for agent_" + str(agent) + ": ")
        for movement in plan.path:
            print(movement[1], end="")
            if plan.path[-1] != movement:
                print(" --> ", end="")
        print()
    print("Solution cost is between " + str(solution[1][0]) + " and " + str(solution[1][1]))
    print("Solution length is  " + str(solution[2]) + "\n")


def run_map(ccbs_map, sic_heuristic=False, print_sol=True, time_limit=180):
    for agent in range(1, len(ccbs_map.start_positions) + 1):
        print("Agent " + str(agent) + ": " + str(ccbs_map.start_positions[agent]) + ", to " +
              str(ccbs_map.goal_positions[agent]))
    print("Finding solution.")
    if sic_heuristic:
        print("Using sum of costs measurement.")
    else:
        print("Finding shortest global time measurement.")
    ccbs_planner = ConformantCbsPlanner(ccbs_map)
    start = time.time()
    solution = ccbs_planner.find_solution(sum_of_costs=sic_heuristic, time_limit=time_limit)
    total = (time.time() - start)
    print("Solution found. Time Elapsed: " + str(total) + " seconds")
    if print_sol:
        print_solution(solution)


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
    test_map = ConformantProblem('../maps/test_map.map')
    test_map.generate_problem_instance(2, (1, 1), (1, 1))
    test_map.start_positions[1] = (0, 0)
    test_map.start_positions[2] = (17, 0)
    test_map.goal_positions[1] = (19, 0)
    test_map.goal_positions[2] = (17, 0)
    test_map.fill_heuristic_table()
    print_map(test_map)
    run_map(test_map, sic_heuristic=True)
    run_map(test_map, sic_heuristic=False)


profile.run('run_test_map()', sort=1)

print("\n----------- Larger random map: 25x25 ---------------")
random_map = ConformantProblem.generate_rectangle_map(12, 12, (1, 1), (1, 1), agent_num=6, is_eight_connected=False)
print_map(random_map)
profile.run('run_map(random_map, sic_heuristic=True, time_limit=10)', sort=2)

total_start = time.time()


print("\n----------- Extra Larger random map: 49x49 ---------------")
random_map = ConformantProblem.generate_rectangle_map(24, 24, (1, 1), (1, 1), agent_num=4, is_eight_connected=False)
print_map(random_map)
run_map(random_map, sic_heuristic=True, print_sol=False)
profile.run('run_map(random_map, sic_heuristic=True, print_sol=False,time_limit=300)', sort=1)


print("-------------- Large moving-ai map -------------------")
complex_map = ConformantProblem('../maps/Archipelago.map')
print("Parsing map..")
complex_map.generate_problem_instance(agent_num=10)
print("Filling heuristics table..")
complex_map.fill_heuristic_table()
profile.run('run_map(complex_map, print_sol=False)', sort=1)

print("Test finished!")
print("Total time elapsed: " + str(time.time() - total_start))




