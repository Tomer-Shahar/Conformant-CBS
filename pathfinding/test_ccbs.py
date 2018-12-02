from pathfinding.map_reader import ConformantProblem
from pathfinding.conformant_cbs import *
import os


def print_solution(solution, conformant_map):
    print("")
    for agent, path in solution[0].items():
        print("Path for agent_" + str(agent) + ": ")
        for movement in path[1]:
            print(movement[1], end="")
            if path[1][-1] != movement:
                print(" --> ", end="")
        print()
    print("Solution cost is between " + str(solution[1][0]) + " and " + str(solution[1][1]))
    print("Solution length is  " + str(solution[2]) + "\n")


def run_map(ccbs_map, sic_heuristic=False):
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
    solution = ccbs_planner.find_solution(sum_of_costs=sic_heuristic, time_limit=10000000)
    total = (time.time() - start)
    print("Solution found. Time Elapsed: " + str(total) + " seconds")
    print_solution(solution, ccbs_map)


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


total_start = time.time()
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

print("\n----------- Larger random map: 12x12 ---------------")
random_map = ConformantProblem.generate_rectangle_map(12, 12, (1, 1), (1, 1), agent_num=3, is_eight_connected=False)
print_map(random_map)
run_map(random_map, sic_heuristic=True)

print("\n----------- Extra Larger random map: 24x24 ---------------")
random_map = ConformantProblem.generate_rectangle_map(24, 24, (1, 1), (1, 1), agent_num=4, is_eight_connected=False)
print_map(random_map)
run_map(random_map, sic_heuristic=True)

print("-------------- Large moving-ai map -------------------")

complex_map = ConformantProblem('../maps/Archipelago.map')
print("Parsing map..")
complex_map.generate_problem_instance(agent_num=5)
print("Filling heuristic table")
complex_map.fill_heuristic_table()

run_map(complex_map)

print("Test finished!")
print("Total time elapsed: " + str(time.time() - total_start))
