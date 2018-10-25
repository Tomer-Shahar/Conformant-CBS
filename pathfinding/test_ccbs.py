from pathfinding.map_reader import conformant_problem
from pathfinding.conformant_cbs import *


def print_solution(solution, map):
    print("")
    for agent, path in solution[0].items():
        print("Path for agent_" + str(agent) + ": ")
        for movement in path[1]:
            print(map.vertex_id_to_coordinate(movement[1]), end="")
            if path[1][-1] != movement:
                print(" --> ", end="")
        print()
    print("Solution cost is between " + str(solution[1][0]) + " and " + str(solution[1][1]))
    print("Solution length is  " + str(solution[2]) + "\n")


def run_map(ccbs_map, sic_heuristic=False):
    for agent in range(1, len(ccbs_map.start_positions) + 1):
        print("Agent " + str(agent) + ": " + str(
            ccbs_map.vertex_id_to_coordinate(ccbs_map.start_positions[agent])) + ", to " + str(
            ccbs_map.vertex_id_to_coordinate(ccbs_map.goal_positions[agent])))
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


total_start = time.time()
print("----------- Small custom map ---------------")
test_map = conformant_problem('../maps/test_map.map')

test_map.generate_problem_instance(2, (1, 1), (1, 1))
test_map.start_positions[1] = test_map.coordinate_to_vertex_id((0, 0))
test_map.start_positions[2] = test_map.coordinate_to_vertex_id((17, 0))
test_map.goal_positions[1] = test_map.coordinate_to_vertex_id((19, 0))
test_map.goal_positions[2] = test_map.coordinate_to_vertex_id((17, 0))
test_map.fill_heuristic_table()
run_map(test_map)

test_map.start_positions[1] = test_map.coordinate_to_vertex_id((0, 0))
test_map.start_positions[2] = test_map.coordinate_to_vertex_id((17, 0))
test_map.goal_positions[1] = test_map.coordinate_to_vertex_id((19, 0))
test_map.goal_positions[2] = test_map.coordinate_to_vertex_id((17, 0))
test_map.fill_heuristic_table()
run_map(test_map, sic_heuristic=True)

test_map.start_positions[1] = test_map.coordinate_to_vertex_id((0, 0))
test_map.start_positions[2] = test_map.coordinate_to_vertex_id((17, 0))
test_map.goal_positions[1] = test_map.coordinate_to_vertex_id((19, 0))
test_map.goal_positions[2] = test_map.coordinate_to_vertex_id((17, 0))
test_map.fill_heuristic_table()

run_map(test_map)

print("\n----------- Larger random map ---------------")
rando_map = conformant_problem.generate_rectangle_map(12, 12, (1, 1), (1, 1), agent_num=3, is_eight_connected=False)
run_map(rando_map)

print("------------------- Large moving-ai map ----------------------")

complex_map = conformant_problem('../maps/Archipelago.map')
print("Parsing map..")
complex_map.generate_problem_instance(agent_num=2)
print("Filling heuristic table")
complex_map.fill_heuristic_table()

run_map(complex_map)

print("Test finished!")
print("Total time elapsed: " + str(time.time()-total_start))
