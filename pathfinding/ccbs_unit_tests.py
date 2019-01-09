"""
Unit tests for Conformant-CBS
"""

from pathfinding.map_reader import ConformantProblem
from pathfinding.conformant_solution import ConformantSolution
from pathfinding.conformant_cbs import *
from pathfinding.constraint_A_star import *
import unittest
import random


class TestMapReader(unittest.TestCase):

    def setUp(self):
        self.conf_problem = ConformantProblem()
        self.conf_problem.map = [[0, 1, 1, 0],
                                 [0, 1, 0, 0],
                                 [0, 0, 0, 0]]
        self.conf_problem.width = 4
        self.conf_problem.height = 3
        """
        .XX.     
        .X..     
        ....     
        """

    def tearDown(self):
        pass

    def test_4_connected_edge_generation(self):
        """Tests if all necessary edges are generated (4-connected)"""

        self.conf_problem.generate_edges_and_weights()
        edges = self.conf_problem.edges_and_weights

        self.assertTrue(len(edges) == 9)  # There should be 9 vertices
        self.assertTrue(edges[(0, 0)] == [((1, 0), (1, 1))])
        self.assertTrue(edges[(0, 3)] == [((1, 3), (1, 1))])

        self.assertTrue(edges[(1, 0)] == [((0, 0), (1, 1)), ((2, 0), (1, 1))])
        self.assertTrue(edges[(1, 2)] == [((1, 3), (1, 1)), ((2, 2), (1, 1))])
        self.assertTrue(edges[(1, 3)] == [((0, 3), (1, 1)), ((1, 2), (1, 1)), ((2, 3), (1, 1))])

        self.assertTrue(edges[(2, 0)] == [((1, 0), (1, 1)), ((2, 1), (1, 1))])
        self.assertTrue(edges[(2, 1)] == [((2, 0), (1, 1)), ((2, 2), (1, 1))])
        self.assertTrue(edges[(2, 2)] == [((1, 2), (1, 1)), ((2, 1), (1, 1)), ((2, 3), (1, 1))])
        self.assertTrue(edges[(2, 3)] == [((1, 3), (1, 1)), ((2, 2), (1, 1))])

    def test_8_connected_edge_generation(self):
        """Tests if all necessary edges are generated (8-connected)"""

        self.conf_problem.generate_edges_and_weights(is_eight_connected=True)
        edges = self.conf_problem.edges_and_weights

        self.assertTrue(len(edges) == 9)  # There should be 9 vertices

        self.assertTrue(edges[(0, 0)] == [((1, 0), (1, 1))])
        self.assertTrue(edges[(0, 3)] == [((1, 2), (1, 1)), ((1, 3), (1, 1))])

        self.assertTrue(edges[(1, 0)] == [((0, 0), (1, 1)), ((2, 0), (1, 1)), ((2, 1), (1, 1))])
        self.assertTrue(edges[(1, 2)] == [((0, 3), (1, 1)), ((1, 3), (1, 1)), ((2, 1), (1, 1)), ((2, 2), (1, 1)), ((2, 3), (1, 1))])
        self.assertTrue(edges[(1, 3)] == [((0, 3), (1, 1)), ((1, 2), (1, 1)), ((2, 2), (1, 1)), ((2, 3), (1, 1))])

        self.assertTrue(edges[(2, 0)] == [((1, 0), (1, 1)), ((2, 1), (1, 1))])
        self.assertTrue(edges[(2, 1)] == [((1, 0), (1, 1)), ((1, 2), (1, 1)), ((2, 0), (1, 1)), ((2, 2), (1, 1))])
        self.assertTrue(edges[(2, 2)] == [((1, 2), (1, 1)), ((1, 3), (1, 1)), ((2, 1), (1, 1)), ((2, 3), (1, 1))])
        self.assertTrue(edges[(2, 3)] == [((1, 2), (1, 1)), ((1, 3), (1, 1)), ((2, 2), (1, 1))])

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
                self.assertTrue(min_a <= edge[1][0] <= min_b)
                self.assertTrue(max_a <= edge[1][1] <= max_b)

    def test_agent_generation(self):
        """Tests that the generate_agents function works properly"""

        self.conf_problem.generate_agents(3)
        self.assertTrue(len(self.conf_problem.start_positions) == 3)
        goal_set = set()
        for agent, goal_pos in self.conf_problem.goal_positions.items():
            goal_set.add(goal_pos)

        for agent, pos in self.conf_problem.start_positions.items():
            self.assertTrue(pos not in goal_set)

    def test_4_connected_heuristic_table(self):
        """Tests that the heuristic table is properly filled on a 4-connected and 8-connected map"""

        self.conf_problem.goal_positions = {1: (0, 0), 2: (2, 0)}
        self.conf_problem.generate_edges_and_weights(is_eight_connected=False)
        self.conf_problem.fill_heuristic_table()
        table = self.conf_problem.heuristic_table

        self.assertTrue(len(table[(0, 0)]) == 9)
        self.assertTrue(len(table[(2, 0)]) == 9)

        self.assertTrue(table[(0, 0)][(0, 0)] == 0)
        self.assertTrue(table[(0, 0)][(0, 3)] == 7)
        self.assertTrue(table[(0, 0)][(1, 0)] == 1)
        self.assertTrue(table[(0, 0)][(1, 2)] == 5)
        self.assertTrue(table[(0, 0)][(1, 3)] == 6)
        self.assertTrue(table[(0, 0)][(2, 0)] == 2)
        self.assertTrue(table[(0, 0)][(2, 1)] == 3)
        self.assertTrue(table[(0, 0)][(2, 2)] == 4)
        self.assertTrue(table[(0, 0)][(2, 3)] == 5)

        self.assertTrue(table[(2, 0)][(0, 0)] == 2)
        self.assertTrue(table[(2, 0)][(0, 3)] == 5)
        self.assertTrue(table[(2, 0)][(1, 0)] == 1)
        self.assertTrue(table[(2, 0)][(1, 2)] == 3)
        self.assertTrue(table[(2, 0)][(1, 3)] == 4)
        self.assertTrue(table[(2, 0)][(2, 0)] == 0)
        self.assertTrue(table[(2, 0)][(2, 1)] == 1)
        self.assertTrue(table[(2, 0)][(2, 2)] == 2)
        self.assertTrue(table[(2, 0)][(2, 3)] == 3)

    def test_8_connected_heuristic_table(self):
        """Tests that the heuristic table is properly filled on a 4-connected and 8-connected map"""

        self.conf_problem.goal_positions = {1: (0, 0), 2: (2, 0)}

        self.conf_problem.generate_edges_and_weights(is_eight_connected=True)
        self.conf_problem.fill_heuristic_table()
        table = self.conf_problem.heuristic_table

        self.assertTrue(len(table[(0, 0)]) == 9)
        self.assertTrue(len(table[(2, 0)]) == 9)

        self.assertTrue(table[(0, 0)][(0, 0)] == 0)
        self.assertTrue(table[(0, 0)][(0, 3)] == 4)
        self.assertTrue(table[(0, 0)][(1, 0)] == 1)
        self.assertTrue(table[(0, 0)][(1, 2)] == 3)
        self.assertTrue(table[(0, 0)][(1, 3)] == 4)
        self.assertTrue(table[(0, 0)][(2, 0)] == 2)
        self.assertTrue(table[(0, 0)][(2, 1)] == 2)
        self.assertTrue(table[(0, 0)][(2, 2)] == 3)
        self.assertTrue(table[(0, 0)][(2, 3)] == 4)

        self.assertTrue(table[(2, 0)][(0, 0)] == 2)
        self.assertTrue(table[(2, 0)][(0, 3)] == 3)
        self.assertTrue(table[(2, 0)][(1, 0)] == 1)
        self.assertTrue(table[(2, 0)][(1, 2)] == 2)
        self.assertTrue(table[(2, 0)][(1, 3)] == 3)
        self.assertTrue(table[(2, 0)][(2, 0)] == 0)
        self.assertTrue(table[(2, 0)][(2, 1)] == 1)
        self.assertTrue(table[(2, 0)][(2, 2)] == 2)
        self.assertTrue(table[(2, 0)][(2, 3)] == 3)


class TestConformantSolution(unittest.TestCase):

    def setUp(self):
        self.conformant_sol = ConformantSolution()
        self.conformant_sol.paths[1] = ConformantPlan(agent_id=1,
                                                      path=[((0, 0), (0, 1)),
                                                            ((1, 3), (0, 2)),
                                                            ((2, 6), (0, 3)),
                                                            ((3, 9), (0, 4)),
                                                            ((4, 12), (1, 4)),
                                                            ((5, 15), (1, 5))]
                                                      , cost=(5, 15))

        self.conformant_sol.paths[2] = ConformantPlan(agent_id=1,
                                                      path=[((0, 0), (3, 1)),
                                                            ((6, 8), (3, 2))]
                                                      , cost=(6, 8))

        self.conformant_sol.length = 5

    def tearDown(self):
        pass

    def test_get_max_min_time(self):
        max_min_time = self.conformant_sol.get_max_of_min_path_time()
        self.assertEqual(max_min_time, 6)

    def test_get_max_time(self):
        max_time = self.conformant_sol.get_max_path_time()

        self.assertEqual(max_time, 15)

    def test_stationary_movement_addition(self):
        self.conformant_sol.add_stationary_moves()

        expected_filled_solution = {
            1: [((0, 0), (0, 1)),
                ((1, 3), (0, 2)),
                ((2, 6), (0, 3)),
                ((3, 9), (0, 4)),
                ((4, 12), (1, 4)),
                ((5, 15), (1, 5)),
                ((6, 16), (1, 5))],
            2: self.conformant_sol.paths[2].path
        }

        self.assertEqual(self.conformant_sol.paths[1].path, expected_filled_solution[1])
        self.assertEqual(self.conformant_sol.paths[2].path, expected_filled_solution[2])

    def test_compute_solution_cost(self):
        """
        tests the compute solution cost function for both sum of cost plans and make span plans.
        """

        self.conformant_sol.compute_solution_cost(sum_of_costs=True)
        self.assertEqual(self.conformant_sol.cost, (11, 23))

        self.conformant_sol.compute_solution_cost(sum_of_costs=False)
        self.assertEqual(self.conformant_sol.cost, (6, 15))

    def test_movement_tuples(self):
        """
        Tests the tuple creation function which converts the solution's plan into a list of edges traversed.:
        """
        self.conformant_sol.add_stationary_moves()
        self.conformant_sol.create_movement_tuples()

        expected_movement = {
            1: [((0, 3), ((0, 1), (0, 2))),
                    ((1, 6), ((0, 2), (0, 3))),
                    ((2, 9), ((0, 3), (0, 4))),
                    ((3, 12), ((0, 4), (1, 4))),
                    ((4, 15), ((1, 4), (1, 5))),
                    ((5, 16), ((1, 5), (1, 5)))],

            2: [((0, 8), ((3, 1), (3, 2)))]
        }

        self.assertEqual(self.conformant_sol.tuple_solution, expected_movement)


class TestLowLevelSolver(unittest.TestCase):
    """ Class for testing the low level solver. """

    def setUp(self):
        pass

    def test_large_map_with_goal_constraint(self):
        """
        Tests a large map where the agent begins at the goal, but has a constraint much later on.
        Ideally, the solution should be found quick since the agent can just stay there and move one step when need be
        """
        large_map = ConformantProblem()
        large_map.map = [[0 for i in range(50)] for j in range(50)]
        large_map.width = 50
        large_map.height = 50
        large_map.generate_edges_and_weights()
        start_goal = (25, 25)
        large_map.start_positions = {1: start_goal}
        large_map.goal_positions = {1: start_goal}
        solver = ConstraintAstar(large_map)
        con_time = 100
        goal_con = {(1, start_goal, con_time)}
        large_map.fill_heuristic_table()
        time_lim = 2
        plan = solver.compute_agent_path(goal_con, 1, start_goal, start_goal, time_limit=time_lim)

        self.assertEqual(plan[2], (con_time+1, con_time+1))


class TestCcbsPlanner(unittest.TestCase):
    """
    Class for testing the conformant_cbs class
    """

    def setUp(self):
        self.conf_problem = ConformantProblem()
        self.conf_problem.map = [[0 for i in range(20)] for j in range(20)]
        for x in range(1, 18):
            for y in range(1, 2):
                self.conf_problem.map[x][y] = 1
        self.conf_problem.width = 20
        self.conf_problem.height = 20
        self.conf_problem.generate_edges_and_weights()
        self.path_finder = ConstraintAstar(self.conf_problem)
        self.CCBS_planner = ConformantCbsPlanner(self.conf_problem)

    def tearDown(self):
        pass

    def test_vertex_conflict_extraction(self):
        """
        Tests whether the extract vertex conflict function works properly. Basically, two agents (1 and 2) are moving
        towards vertex 3 from vertices 1 and 2 respectively. The overlap time is 4-6.
        """
        interval_1 = (3, 6)
        interval_2 = (4, 10)
        v_1 = (0, 2)
        v_2 = (0, 4)
        v = (0, 3)
        agent_1 = 1
        agent_2 = 2
        self.CCBS_planner.edges_and_weights[v_1] = [(v, 3, 6)]
        self.CCBS_planner.edges_and_weights[v_2] = [(v, 4, 10)]
        con_sets = self.CCBS_planner.extract_vertex_conflict_constraints(
            interval_1, interval_2, v, agent_1, agent_2)

        agent_1_cons = {(agent_1, v, 6)}
        agent_2_cons = {(agent_2, v, 6)}

        self.assertTrue(con_sets[0] == agent_1_cons and con_sets[1] == agent_2_cons)

    def test_edge_swap_conflict_extraction_with_time_range(self):
        """
        Tests whether the extract edge conflict function works properly. Basically, two agents (1 and 2) are travelling
        and will eventually swap paths. The conflicted edge has a time range of 5-10 to pass it.
        Agent 1 will finish traversing the edge between 12 and 20, and agent 2 will finish between 8 and 18.
        Agent 1 will occupy the edge between 8 and 19, while agent 2 will occupy it between 4 and 17.
        The conflict interval will be (8, 17) --> constrain time tick 17.
        """

        agent_1 = 1
        agent_2 = 2
        interval_1 = (12, 20)
        interval_2 = (8, 18)
        edge = ((0, 5), (0, 6))
        self.CCBS_planner.edges_and_weights = {edge[0]: [(edge[1], (5, 10))]}
        agent_1_cons = set()
        agent_2_cons = set()

        for i in range(8, 14):
            agent_1_cons.add((agent_1, edge, i))
            agent_2_cons.add((agent_2, edge, i))

        con_sets = self.CCBS_planner.extract_edge_conflict(agent_1, agent_2, interval_1, interval_2, edge)

        self.assertTrue(con_sets[0] == agent_1_cons and con_sets[1] == agent_2_cons)

    def test_edge_swap_conflict_extraction_without_time_range(self):
        """
        Tests whether the extract edge conflict function works properly. Basically, two agents (1 and 2) are travelling
        and will eventually swap paths. The conflicted edge has a traversal time of 1.
        """
        agent_1 = 1
        agent_2 = 2
        interval_1 = (17, 18)
        interval_2 = (17, 18)
        v_1 = (17, 0)
        v_2 = (18, 0)
        edge = (v_1, v_2)
        self.CCBS_planner.edges_and_weights = {v_1: {(v_2, (1, 1))}}
        agent_1_cons = {(agent_1, edge, 18)}  # ToDo: Should the time be 17 or 18?
        agent_2_cons = {(agent_2, edge, 18)}

        con_sets = self.CCBS_planner.extract_edge_conflict(agent_1, agent_2, interval_1, interval_2, edge)

        self.assertTrue(con_sets[0] == agent_1_cons and con_sets[1] == agent_2_cons)

    def test_illegal_moves(self):
        """
        Tests if illegal actions are correctly detected such as moving over a constrained edge.
        """
        agent = 1
        v_1 = (17, 0)
        v_2 = (18, 0)
        edge = (v_1, v_2)
        self.CCBS_planner.edges_and_weights = {v_1: {(v_2, (1, 1)), ((16, 0), (1, 1))}}
        constraints = {(1, edge, 18)}
        self.single_agent_node = SingleAgentNode(v_1, (16, 0), (17, 17), self.conf_problem, (19, 0))
        successor = (v_2, (18, 18))
        self.assertFalse(self.single_agent_node.legal_move(agent, successor, constraints))

    def test_simple_4_connected_two_agent_map(self):

        """ Tests a simple 20x20 map with 2 agents and non-weighted edges"""
        self.conf_problem.generate_edges_and_weights()
        self.conf_problem.start_positions[1] = (0, 0)
        self.conf_problem.start_positions[2] = (17, 0)
        self.conf_problem.goal_positions[1] = (19, 0)
        self.conf_problem.goal_positions[2] = (17, 0)
        self.conf_problem.fill_heuristic_table()

        ccbs_planner = ConformantCbsPlanner(self.conf_problem)
        solution = ccbs_planner.find_solution(min_best_case=True, time_limit=2000, sum_of_costs=True)
        self.assertEqual(solution.cost, (23, 23))  # Only agent 1 moves
        self.assertEqual(solution.length, 24)

        solution = ccbs_planner.find_solution(min_best_case=True, time_limit=2000, sum_of_costs=False)
        self.assertEqual(solution.cost, (20, 20))  # Both agents move simultaneously
        solution.compute_solution_cost(sum_of_costs=True)
        self.assertEqual(solution.cost, (39, 39))
        self.assertEqual(solution.length, 21)

