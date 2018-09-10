"""
Unit tests for Conformant-CBS
"""
from pathfinding.map_reader import ccbsMap
from pathfinding.conformant_cbs import *
import unittest

class TestCcbsPlanner(unittest.TestCase):

    def test_CCBS_without_uncertainty(self):
        test_map = ccbsMap()
        test_map.map = [ [0 for i in range(20)] for j in range(20)]
        test_map.width = 20
        test_map.height = 20
        for i in range (1,18):
            for j in range(1,2):
                test_map.map[i][j] = 1
        test_map.generate_edges_and_timeSteps((1,1), (1,1))
        test_map.start_positions = { 1:0, 2:17 * 20}
        test_map.goal_positions = {1:19 * test_map.width + 0, 2:17 * test_map.width + 0}
        for agent in range(1, len(test_map.start_positions)+1):
            print("Agent " + str(agent) + ": " + str(
                test_map.vertex_id_to_coordinate(test_map.start_positions[agent])) + ", to " + str(
                test_map.vertex_id_to_coordinate(test_map.goal_positions[agent])))
        ccbs_planner = ConformantCbsPlanner(test_map)
        solution = ccbs_planner.find_solution(time_limit=10000000)
        self.assertTrue(solution[1] == 21)



