"""
Unit tests for Conformant-CBS
"""

from pathfinding.map_reader import conformant_problem
from pathfinding.conformant_cbs import *
from pathfinding.path_finder import *
import unittest
import random


class TestMapReader(unittest.TestCase):

    def setUp(self):
        self.conf_problem = conformant_problem()
        self.conf_problem.map = [[0, 1, 1, 0],
                                 [0, 1, 0, 0],
                                 [0, 0, 0, 0]]
        self.conf_problem.width = 4
        self.conf_problem.height = 3
        """
        .XX.     0 1 2  3
        .X..     4 5 6  7
        ....     8 9 10 11
        """

    def tearDown(self):
        pass

    def test_4_connected_edge_generation(self):
        """Tests if all necessary edges are generated (4-connected)"""

        self.conf_problem.generate_edges_and_weights()
        edges = self.conf_problem.edges_and_weights

        self.assertTrue(len(edges) == 9)  # There should be 9 vertices

        self.assertTrue(0 in edges)
        self.assertTrue(edges[0] == [(4, 1, 1)])

        self.assertTrue(3 in edges)
        self.assertTrue(edges[3] == [(7, 1, 1)])

        self.assertTrue(4 in edges)
        self.assertTrue(edges[4] == [(0, 1, 1), (8, 1, 1)])

        self.assertTrue(6 in edges)
        self.assertTrue(edges[6] == [(7, 1, 1), (10, 1, 1)])

        self.assertTrue(7 in edges)
        self.assertTrue(edges[7] == [(3, 1, 1), (6, 1, 1), (11, 1, 1)])

        self.assertTrue(8 in edges)
        self.assertTrue(edges[8] == [(4, 1, 1), (9, 1, 1)])

        self.assertTrue(9 in edges)
        self.assertTrue(edges[9] == [(8, 1, 1), (10, 1, 1)])

        self.assertTrue(10 in edges)
        self.assertTrue(edges[10] == [(6, 1, 1), (9, 1, 1), (11, 1, 1)])

        self.assertTrue(11 in edges)
        self.assertTrue(edges[11] == [(7, 1, 1), (10, 1, 1)])

    def test_8_connected_edge_generation(self):
        """Tests if all necessary edges are generated (8-connected)"""

        self.conf_problem.generate_edges_and_weights(is_eight_connected=True)
        edges = self.conf_problem.edges_and_weights

        self.assertTrue(len(edges) == 9)  # There should be 9 vertices

        self.assertTrue(0 in edges)
        self.assertTrue(edges[0] == [(4, 1, 1)])

        self.assertTrue(3 in edges)
        self.assertTrue(edges[3] == [(6, 1, 1), (7, 1, 1)])

        self.assertTrue(4 in edges)
        self.assertTrue(edges[4] == [(0, 1, 1), (8, 1, 1), (9, 1, 1)])

        self.assertTrue(6 in edges)
        self.assertTrue(edges[6] == [(3, 1, 1), (7, 1, 1), (9, 1, 1), (10, 1, 1), (11, 1, 1)])

        self.assertTrue(7 in edges)
        self.assertTrue(edges[7] == [(3, 1, 1), (6, 1, 1), (10, 1, 1), (11, 1, 1)])

        self.assertTrue(8 in edges)
        self.assertTrue(edges[8] == [(4, 1, 1), (9, 1, 1)])

        self.assertTrue(9 in edges)
        self.assertTrue(edges[9] == [(4, 1, 1), (6, 1, 1), (8, 1, 1), (10, 1, 1)])

        self.assertTrue(10 in edges)
        self.assertTrue(edges[10] == [(6, 1, 1), (7, 1, 1), (9, 1, 1), (11, 1, 1)])

        self.assertTrue(11 in edges)
        self.assertTrue(edges[11] == [(6, 1, 1), (7, 1, 1), (10, 1, 1)])

    def test_edge_weights(self):
        """Tests if the weights of the edges are in the range given."""

        min_a = random.randint(1, 3)
        min_b = random.randint(4, 6)
        min_range = min_a, min_b

        max_a = random.randint(7, 10)
        max_b = random.randint(11, 15)
        max_range = max_a, max_b

        self.conf_problem.generate_edges_and_weights(min_range, max_range)

        for vertex, edgeList in self.conf_problem.edges_and_weights.items():
            for edge in edgeList:
                self.assertTrue(min_a <= edge[1] <= min_b)
                self.assertTrue(max_a <= edge[2] <= max_b)

    def test_agent_generation(self):
        """Tests that the generate_agents function works properly"""

        self.conf_problem.generate_agents(3)
        self.assertTrue(len(self.conf_problem.start_positions) == 3)
        goalSet = set()
        for agent, goal_pos in self.conf_problem.goal_positions.items():
            goalSet.add(goal_pos)

        for agent, pos in self.conf_problem.start_positions.items():
            self.assertTrue(pos not in goalSet)

    def test_heuristic_table(self):
        """Tests that the heuristic table is properly filled on a 4-connected and 8-connected map"""

        self.conf_problem.goal_positions = {1: 0, 2: 8}
        self.conf_problem.generate_edges_and_weights()
        self.conf_problem.fill_heuristic_table()
        table = self.conf_problem.heuristic_table

        self.assertTrue(len(table[0]) == 9)
        self.assertTrue(len(table[8]) == 9)

        self.assertTrue(table[0][0] == 0)
        self.assertTrue(table[0][3] == 7)
        self.assertTrue(table[0][4] == 1)
        self.assertTrue(table[0][6] == 5)
        self.assertTrue(table[0][7] == 6)
        self.assertTrue(table[0][8] == 2)
        self.assertTrue(table[0][9] == 3)
        self.assertTrue(table[0][10] == 4)
        self.assertTrue(table[0][11] == 5)

        self.assertTrue(table[8][0] == 2)
        self.assertTrue(table[8][3] == 5)
        self.assertTrue(table[8][4] == 1)
        self.assertTrue(table[8][6] == 3)
        self.assertTrue(table[8][7] == 4)
        self.assertTrue(table[8][8] == 0)
        self.assertTrue(table[8][9] == 1)
        self.assertTrue(table[8][10] == 2)
        self.assertTrue(table[8][11] == 3)

        self.conf_problem.generate_edges_and_weights(is_eight_connected=True)
        self.conf_problem.fill_heuristic_table()
        table = self.conf_problem.heuristic_table

        self.assertTrue(len(table[0]) == 9)
        self.assertTrue(len(table[8]) == 9)

        self.assertTrue(table[0][0] == 0)
        self.assertTrue(table[0][3] == 4)
        self.assertTrue(table[0][4] == 1)
        self.assertTrue(table[0][6] == 3)
        self.assertTrue(table[0][7] == 4)
        self.assertTrue(table[0][8] == 2)
        self.assertTrue(table[0][9] == 2)
        self.assertTrue(table[0][10] == 3)
        self.assertTrue(table[0][11] == 4)

        self.assertTrue(table[8][0] == 2)
        self.assertTrue(table[8][3] == 3)
        self.assertTrue(table[8][4] == 1)
        self.assertTrue(table[8][6] == 2)
        self.assertTrue(table[8][7] == 3)
        self.assertTrue(table[8][8] == 0)
        self.assertTrue(table[8][9] == 1)
        self.assertTrue(table[8][10] == 2)
        self.assertTrue(table[8][11] == 3)

class TestCcbsPlanner(unittest.TestCase):
    """
    Class for testing the conformant_cbs class
    """

    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_vertex_conflict_extraction(self):
        """
        Tests whether the extract vertex conflict function works properly. Basically, two agents (1 and 2) are moving
        towards vertex 3 from vertices 1 and 2 respectively. The overlap time is 4-6.
        """
        interval_1 = (3, 6)
        interval_2 = (4, 10)
        v_1 = 1
        v_2 = 2
        v = 3
        agent_1 = 1
        agent_2 = 2
        planner = ConformantCbsPlanner(conformant_problem.generate_rectangle_map(5, 5, (1, 1), (2, 2), 2, False))

        planner.edges_and_weights[1] = [(3, 3, 6)]
        planner.edges_and_weights[2] = [(3, 4, 10)]
        con_sets = planner.extract_vertex_conflict(interval_1, interval_2, v, agent_1, agent_2, v_1, v_2)

        agent_1_cons = {(1, 3, 0), (1, 3, 1), (1, 3, 2), (1, 3, 3)}
        agent_2_cons = {(2, 3, -4), (2, 3, -3), (2, 3, -2), (2, 3, -1), (2, 3, 0), (2, 3, 1), (2, 3, 2)}

        self.assertTrue(con_sets[0] == agent_1_cons and con_sets[1] == agent_2_cons)

