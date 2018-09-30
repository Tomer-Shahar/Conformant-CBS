from pathfinding.map_reader import ccbsMap
from pathfinding.conformant_cbs import *

def print_solution(solution,map):
    print("")
    for agent,path in solution[0].items():
        print("Path for agent_"+str(agent)+": ")
        for movement in path[1]:
            print(map.vertex_id_to_coordinate(movement[1]), end="")
            if path[1][-1] != movement:
                print(" --> ", end="")
        print()
    print("Solution cost is between " + str(solution[1][0]) + " and " + str(solution[1][1]))

num_of_agent = 3
rando_map = ccbsMap.generate_rectangle_map(8, 8, (1,1), (1,1), num_of_agent, False)

for agent in range(1,num_of_agent+1):
    print("Agent " + str(agent) + ": " + str(rando_map.vertex_id_to_coordinate(rando_map.start_positions[agent])) + ", to " + str(rando_map.vertex_id_to_coordinate(rando_map.goal_positions[agent])))
ccbs_planner = ConformantCbsPlanner(rando_map)
solution = ccbs_planner.find_solution(time_limit=1000 * 30)

if solution:
    print_solution(solution, rando_map)

"""------------------- Large moving-ai map ----------------------"""

complex_map = ccbsMap('../maps/Archipelago.map')
start = time.time()

print("Parsing map..")
complex_map.parse_file(agent_num=1)

ccbs_planner = ConformantCbsPlanner(complex_map)
for agent in range(1, len(complex_map.start_positions) + 1):
    print("Agent " + str(agent) + ": " + str(complex_map.vertex_id_to_coordinate(complex_map.start_positions[agent])) + ", to " + str(complex_map.vertex_id_to_coordinate(complex_map.goal_positions[agent])))

print("Finding solution..")
start = time.time()
solution = ccbs_planner.find_solution(time_limit=10000000)
total = (time.time() - start)

print("Solution found. Time Elapsed: " + str(total) + " seconds")
print_solution(solution, complex_map)

"""
num_of_agent = 3
rando_map = ccbsMap.generate_rectangle_map(10, 10, (1,1), (1,1), num_of_agent)

for agent in range(1,num_of_agent+1):
    print("Agent " + str(agent) + ": " + str(rando_map.vertex_id_to_coordinate(rando_map.start_positions[agent])) + ", to " + str(rando_map.vertex_id_to_coordinate(rando_map.goal_positions[agent])))
ccbs_planner = ConformantCbsPlanner(rando_map)
solution = ccbs_planner.find_solution(time_limit=1000 * 30)

if solution:
    print_solution(solution, rando_map)

"""

print("Test finished!")