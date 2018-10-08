"""
Unit tests for Conformant-CBS
"""
from pathfinding.map_reader import ccbsMap
from pathfinding.conformant_cbs import *
import unittest

class TestCcbsPlanner(unittest.TestCase):

    def test_CCBS_without_uncertainty_path_21(self):
        test_map = ccbsMap()
        test_map.map = [ [0 for i in range(20)] for j in range(20)]
        test_map.width = 20
        test_map.height = 20
        for i in range (1,18):
            for j in range(1,2):
                test_map.map[i][j] = 1
        test_map.generate_edges_and_weights((1, 1), (1, 1))
        test_map.start_positions = { 1:0, 2:17 * 20}
        test_map.goal_positions = {1:19 * test_map.width + 0, 2:17 * test_map.width + 0}
        for agent in range(1, len(test_map.start_positions)):
            print("Agent " + str(agent) + ": " + str(
                test_map.vertex_id_to_coordinate(test_map.start_positions[agent])) + ", to " + str(
                test_map.vertex_id_to_coordinate(test_map.goal_positions[agent])))
        ccbs_planner = ConformantCbsPlanner(test_map)
        solution = ccbs_planner.find_solution(time_limit=10000000)

        self.print_solution(solution, test_map)

        self.assertTrue(solution[1] == 21)

    def test_CCBS_without_uncertainty_path_24(self):
        test_map = ccbsMap()
        test_map.map = [ [0 for i in range(20)] for j in range(20)]
        test_map.width = 20
        test_map.height = 20
        for i in range (1,18):
            for j in range(1,2):
                test_map.map[i][j] = 1
        test_map.generate_edges_and_weights((1, 1), (1, 1))
        test_map.start_positions = { 1:0, 2:19 * test_map.width}
        test_map.goal_positions = {1:19 * test_map.width + 0, 2:0}
        for agent in range(1, len(test_map.start_positions)):
            print("Agent " + str(agent) + ": " + str(
                test_map.vertex_id_to_coordinate(test_map.start_positions[agent])) + ", to " + str(
                test_map.vertex_id_to_coordinate(test_map.goal_positions[agent])))
        ccbs_planner = ConformantCbsPlanner(test_map)
        solution = ccbs_planner.find_solution(time_limit=10000000)

        self.print_solution(solution, test_map)

        self.assertTrue(solution[1] == 23)


    def print_solution(self, solution, map):
        print("")
        for agent,path in solution[0].items():
            print("Path for agent "+str(agent)+": ")
            for movement in path[1]:
                print(map.vertex_id_to_coordinate(movement[1]), end="")
                if path[1][-1] != movement:
                    print(" --> ", end="")
            print('\n')
        print("Solution cost: " + str(solution[1]))


