"""
Unit tests for Conformant-CBS
"""

from pathfinding.planners.utils.map_reader import TimeUncertaintyProblem
from pathfinding.planners.utils.time_uncertainty_solution import TimeUncertainSolution
from pathfinding.planners.cbstu import *
from pathfinding.planners.constraint_A_star import *
from pathfinding.planners.operator_decomposition_a_star import *
from pathfinding.planners.utils.time_error import *
from pathfinding.simulator import MAPFSimulator
from pathfinding.planners.online_cbstu import OnlineCBSTU

import unittest
import random


class TestMapReader(unittest.TestCase):

    def setUp(self):
        self.conf_problem = TimeUncertaintyProblem()
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

        blank_map = TimeUncertaintyProblem()
        blank_map.map = [[0 for i in range(50)] for j in range(50)]
        blank_map.height = 50
        blank_map.width = 50
        blank_map.generate_edges_and_weights(uncertainty=1)

        distribution = {(1, 1): 0, (1, 2): 0, (2, 2): 0}
        for vertex, edgeList in blank_map.edges_and_weights.items():
            for edge in edgeList:
                distribution[edge[1]] += 1
        sum_val = sum(distribution.values())
        distribution[(1, 1)] = distribution[(1, 1)] / sum_val
        distribution[(1, 2)] = distribution[(1, 2)] / sum_val
        distribution[(2, 2)] = distribution[(2, 2)] / sum_val
        print(distribution)

        self.assertTrue(0.3 <= distribution[(1, 1)] <= 0.35)
        self.assertTrue(0.3 <= distribution[(1, 2)] <= 0.35)
        self.assertTrue(0.3 <= distribution[(2, 2)] <= 0.35)

    def test_edge_weights_distribution(self):
        """Tests if the weights of the edges are in the range given."""

        uncertainty = 1

        self.conf_problem.generate_edges_and_weights(uncertainty)

        for vertex, edgeList in self.conf_problem.edges_and_weights.items():
            for edge in edgeList:
                self.assertTrue(1 <= edge[1][0] <= edge[1][1])
                self.assertTrue(edge[1][0] <= edge[1][1] <= 1+uncertainty)

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

    def test_edge_weight_generation(self):
        """
        Tests that the function that generates edge weights always produces the same weight for edge (u,v) as edge
        (v,u).
        """
        blank_map = TimeUncertaintyProblem()
        blank_map.map = [[0 for i in range(50)] for j in range(50)]
        blank_map.width = 50
        blank_map.height = 50
        blank_map.generate_edges_and_weights(uncertainty=10)

        edge_weights_dict = {}
        for vertex, edges in blank_map.edges_and_weights.items():
            for next_vertex in edges:
                next_node = next_vertex[0]
                edge_weight = next_vertex[1]
                normal_edge = self.normalize_edge(vertex, next_node)
                if normal_edge in edge_weights_dict:
                    self.assertEqual(edge_weights_dict[normal_edge], edge_weight)
                else:
                    edge_weights_dict[normal_edge] = edge_weight

    @staticmethod
    def normalize_edge(start, end):
        return min(start, end), max(start, end)


class TestConformantSolution(unittest.TestCase):

    def setUp(self):
        self.conformant_sol = TimeUncertainSolution()
        self.conformant_sol.paths[1] = TimeUncertainPlan(agent_id=1,
                                                         path=[((0, 0), (0, 1)),
                                                            ((1, 3), (0, 2)),
                                                            ((2, 6), (0, 3)),
                                                            ((3, 9), (0, 4)),
                                                            ((4, 12), (1, 4)),
                                                            ((5, 15), (1, 5))]
                                                         , cost=(5, 15))

        self.conformant_sol.paths[2] = TimeUncertainPlan(agent_id=1,
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
        large_map = TimeUncertaintyProblem()
        large_map.map = [[0 for i in range(50)] for j in range(50)]
        large_map.width = 50
        large_map.height = 50
        large_map.generate_edges_and_weights()
        start_goal = (25, 25)
        large_map.start_positions = {1: start_goal}
        large_map.goal_positions = {1: start_goal}
        solver = ConstraintAstar(large_map)
        con_time = 50, 50
        goal_con = {(1, start_goal, con_time)}
        large_map.fill_heuristic_table()
        time_lim = 10
        plan = solver.compute_agent_path(goal_con, 1, start_goal, start_goal, {1: set()}, time_limit=time_lim)

        self.assertEqual(plan.cost, (con_time[0] + 1, con_time[1] + 1))


class TestCcbsPlanner(unittest.TestCase):
    """
    Class for testing the conformant_cbs class
    """

    def setUp(self):
        self.conf_problem = TimeUncertaintyProblem()
        self.conf_problem.map = [[0 for i in range(20)] for j in range(20)]
        for x in range(1, 18):
            for y in range(1, 2):
                self.conf_problem.map[x][y] = 1
        self.conf_problem.width = 20
        self.conf_problem.height = 20
        self.conf_problem.generate_edges_and_weights()
        self.path_finder = ConstraintAstar(self.conf_problem)
        self.CCBS_planner = CBSTUPlanner(self.conf_problem)

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

        agent_1_cons = {(agent_1, v, (6, 6))}
        agent_2_cons = {(agent_2, v, (6, 6))}

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

        agent_1_cons.add((agent_1, edge, (17, 17)))
        agent_2_cons.add((agent_2, edge, (17, 17)))

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
        agent_1_cons = {(agent_1, edge, (18, 18))}  # ToDo: Should the time be 17 or 18?
        agent_2_cons = {(agent_2, edge, (18, 18))}

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
        constraints = {(1, edge, (18, 18))}
        self.single_agent_node = SingleAgentNode(v_1, (16, 0), (17, 17), self.conf_problem, (19, 0), 0)
        successor = (v_2, (18, 18))
        self.assertFalse(self.single_agent_node.legal_move(agent, successor[0], successor[1], constraints))

    def test_simple_4_connected_two_agent_map(self):

        """ Tests a simple 20x20 map with 2 agents and non-weighted edges"""
        self.conf_problem.generate_edges_and_weights()
        self.conf_problem.start_positions[1] = (0, 0)
        self.conf_problem.start_positions[2] = (17, 0)
        self.conf_problem.goal_positions[1] = (19, 0)
        self.conf_problem.goal_positions[2] = (17, 0)
        self.conf_problem.fill_heuristic_table()

        ccbs_planner = CBSTUPlanner(self.conf_problem)
        solution = ccbs_planner.find_solution(min_best_case=True, time_limit=2000, sum_of_costs=False)
        self.assertEqual(solution.cost, (20, 20))  # Both agents move simultaneously

        solution.compute_solution_cost(sum_of_costs=True)
        self.assertEqual(solution.cost, (39, 39))
        self.assertEqual(solution.length, 21)

        solution = ccbs_planner.find_solution(min_best_case=True, time_limit=2000, sum_of_costs=True)
        self.assertEqual(solution.cost, (23, 23))  # Only agent 1 moves
        self.assertEqual(solution.length, 24)



    def test_more_complex_4_connected_map(self):
        complex_conf_prob = TimeUncertaintyProblem()
        complex_conf_prob.edges_and_weights = {
            (0, 0): [((0, 1), (3, 5))],
            (0, 1): [((0, 0), (3, 5)), ((0, 2), (1, 3)), ((1, 1), (1, 2))],
            (0, 2): [((0, 1), (1, 3))],
            (1, 1): [((0, 1), (1, 2)), ((2, 1), (1, 2))],
            (2, 0): [((2, 1), (2, 4))],
            (2, 1): [((2, 0), (2, 4)), ((2, 2), (1, 5)), ((1, 1), (1, 2))],
            (2, 2): [((2, 1), (1, 5))]
        }
        complex_conf_prob.start_positions[1] = (0, 0)
        complex_conf_prob.start_positions[2] = (0, 2)
        complex_conf_prob.goal_positions[1] = (2, 1)
        complex_conf_prob.goal_positions[2] = (1, 1)
        complex_conf_prob.fill_heuristic_table()

        ccbs_planner = CBSTUPlanner(complex_conf_prob)
        solution = ccbs_planner.find_solution(min_best_case=True, time_limit=10, sum_of_costs=True)
        self.assertEqual(solution.cost, (13, 20))

    def test_blank_map(self):
        seed = 12345678
        random.seed(seed)
        map_file = '.\small_blank_map.map'
        blank_problem = TimeUncertaintyProblem(map_file)
        blank_problem.generate_problem_instance(1)
        seed = 2025421785
        random.seed(seed)
        blank_problem.generate_agents(2)
        blank_problem.fill_heuristic_table()
        ccbs_solver = CBSTUPlanner(blank_problem)
        sol = ccbs_solver.find_solution(True, 1000, True)
        self.assertEqual(sol.cost, (16, 24))

        seed = 12345678
        random.seed(seed)
        blank_problem.generate_problem_instance(uncertainty=2)
        seed = 1432200775
        random.seed(seed)
        blank_problem.generate_agents(agent_num=3)
        blank_problem.fill_heuristic_table()

        ccbs_solver = CBSTUPlanner(blank_problem)
        sol = ccbs_solver.find_solution(True, 1000, True)
        self.assertEqual(sol.cost, (28, 44))

    def test_edge_conflict_detection(self):

        edge_example = TimeUncertaintyProblem()
        edge_example.edges_and_weights = {
            (0, 0): [((0, 1), (5, 5))],
            (0, 1): [((0, 0), (5, 5)), ((0, 2), (3, 3))],
            (0, 2): [((0, 1), (3, 3)), ((0, 3), (7, 7)), ((1, 2), (5, 5))],
            (0, 3): [((0, 2), (7, 7))],
            (1, 2): [((0, 2), (5, 5))]
        }

        edge_example.start_positions[1] = (0, 0)
        edge_example.start_positions[2] = (0, 3)
        edge_example.goal_positions[1] = (0, 3)
        edge_example.goal_positions[2] = (0, 0)

        edge_example.fill_heuristic_table()
        solver = CBSTUPlanner(edge_example)

        sol = solver.find_solution(time_limit=100)

        self.assertEqual(sol.cost, (40, 40))

    def test_optimality(self):
        blank_map = TimeUncertaintyProblem('.\\small_blank_map.map')
        random.seed(12345678)
        blank_map.generate_problem_instance(uncertainty=1)
        agent_seed = 1315457633
        random.seed(agent_seed)
        blank_map.generate_agents(agent_num=7)
        blank_map.fill_heuristic_table()
        cbs_planner = CBSTUPlanner(blank_map)
        oda_solver = ODAStar(blank_map)
        cbs_sol = cbs_planner.find_solution(time_limit=2)
        oda_sol = oda_solver.create_solution()
        self.assertEqual(cbs_sol.cost[0], 45)
        self.assertEqual(oda_sol[1][0], 45)


class TestODAPlanner(unittest.TestCase):
    """
    Class for testing the conformant_cbs class
    """

    def setUp(self):
        self.conf_problem = TimeUncertaintyProblem()
        self.conf_problem.map = [[0 for i in range(20)] for j in range(20)]
        for x in range(1, 18):
            for y in range(1, 2):
                self.conf_problem.map[x][y] = 1
        self.conf_problem.width = 3
        self.conf_problem.height = 20
        self.conf_problem.generate_edges_and_weights()

        self.mini_conf_problem = TimeUncertaintyProblem()
        self.mini_conf_problem.map = [[0, 0],
                                      [0, 0]]
        self.mini_conf_problem.width = 2
        self.mini_conf_problem.height = 2
        self.mini_conf_problem.edges_and_weights = {
            (0, 0): [((0, 1), (1, 1)), ((1, 0), (1, 1))],
            (0, 1): [((0, 0), (1, 1)), ((1, 1), (3, 3))],
            (1, 0): [((0, 0), (1, 1)), ((1, 1), (3, 3))],
            (1, 1): [((1, 0), (3, 3)), ((0, 1), (3, 3))]}

    def tearDown(self):
        pass

    def test_simple_4_connected_two_agent_map(self):

        self.path_finder = ODAStar(self.mini_conf_problem)

        self.mini_conf_problem.start_positions[1] = (0, 0)
        self.mini_conf_problem.start_positions[2] = (0, 1)
        self.mini_conf_problem.start_positions[3] = (1, 1)
        self.mini_conf_problem.goal_positions[1] = (1, 1)
        self.mini_conf_problem.goal_positions[2] = (1, 0)
        self.mini_conf_problem.goal_positions[3] = (0, 0)
        self.mini_conf_problem.fill_heuristic_table()

        solution = self.path_finder.create_solution(time_limit=1, objective='min_best_case', sic=True)
        self.assertEqual(solution[1], (10, 10))

        """ Tests a simple 20x20 map with 2 agents and non-weighted edges"""
        self.path_finder = ODAStar(self.conf_problem)
        self.conf_problem.start_positions[1] = (0, 0)
        self.conf_problem.start_positions[2] = (17, 0)
        self.conf_problem.goal_positions[1] = (19, 0)
        self.conf_problem.goal_positions[2] = (17, 0)
        self.conf_problem.fill_heuristic_table()

        solution = self.path_finder.create_solution(time_limit=5, objective='min_best_case', sic=True)
        self.assertEqual(solution[1], (23, 23))  # Only agent 1 moves
        # self.assertEqual(solution.length, 24)

        solution = self.path_finder.create_solution(time_limit=5, objective='min_best_case', sic=False)
        self.assertEqual(solution[1], (39, 39))  # Both agents move simultaneously

    def test_more_complex_4_connected_map(self):
        complex_conf_prob = TimeUncertaintyProblem()
        complex_conf_prob.edges_and_weights = {
            (0, 0): [((0, 1), (3, 5))],
            (0, 1): [((0, 0), (3, 5)), ((0, 2), (1, 3)), ((1, 1), (1, 2))],
            (0, 2): [((0, 1), (1, 3))],
            (1, 1): [((0, 1), (1, 2)), ((2, 1), (1, 2))],
            (2, 0): [((2, 1), (2, 4))],
            (2, 1): [((2, 0), (2, 4)), ((2, 2), (1, 5)), ((1, 1), (1, 2))],
            (2, 2): [((2, 1), (1, 5))]
        }
        complex_conf_prob.start_positions[1] = (0, 0)
        complex_conf_prob.start_positions[2] = (0, 2)
        complex_conf_prob.goal_positions[1] = (2, 1)
        complex_conf_prob.goal_positions[2] = (1, 1)
        complex_conf_prob.fill_heuristic_table()

        self.path_finder = ODAStar(complex_conf_prob)
        solution = self.path_finder.create_solution(time_limit=5, objective='min_best_case', sic=True)
        self.assertEqual(solution[1], (13, 20))

    def test_collision_detection(self):

        t_map = TimeUncertaintyProblem()
        t_map.edges_and_weights = {
            (0, 0): [((0, 1), (10, 10))],
            (0, 1): [((0, 0), (10, 10)), ((0, 2), (1, 1)), ((1, 1), (20, 20))],
            (0, 2): [((0, 1), (1, 1))],
            (1, 1): [((0, 1), (20, 20))]
        }

        t_map.start_positions[1] = (0, 0)
        t_map.start_positions[2] = (0, 1)
        t_map.goal_positions[1] = (0, 2)
        t_map.goal_positions[2] = (0, 1)

        t_map.fill_heuristic_table()
        oda_solver = ODAStar(t_map)
        sol = oda_solver.create_solution(1, min_time_policy=True)

        self.assertEqual(sol[1], (51, 51))

    def test_blank_map(self):
        seed = 12345678
        random.seed(seed)
        map_file = './small_blank_map.map'
        blank_problem = TimeUncertaintyProblem(map_file)
        blank_problem.generate_problem_instance(1)
        seed = 2025421785
        random.seed(seed)
        blank_problem.generate_agents(2)
        blank_problem.fill_heuristic_table()
        oda_star_solver = ODAStar(blank_problem)
        sol = oda_star_solver.create_solution(5, min_time_policy=False)  # Serial mode
        self.assertEqual(sol[1], (16, 22))

        sol = oda_star_solver.create_solution(5, min_time_policy=True)  # Queue mode
        self.assertEqual(sol[1], (16, 22))

        seed = 12345678
        random.seed(seed)
        blank_problem.generate_problem_instance(uncertainty=2)
        seed = 1432200775
        random.seed(seed)
        blank_problem.generate_agents(agent_num=3)
        blank_problem.fill_heuristic_table()

        oda_star_solver = ODAStar(blank_problem)

        queue_sol = oda_star_solver.create_solution(time_limit=5, min_time_policy=True)  # Queue mode
        self.assertEqual(queue_sol[1], (28, 46))

    def test_exam_problem(self):

        mini_conf_problem = TimeUncertaintyProblem()
        mini_conf_problem.map = [[0, 0, 0],
                                 [0, 0, 0]]
        mini_conf_problem.width = 3
        mini_conf_problem.height = 2
        mini_conf_problem.edges_and_weights = {
            (0, 0): [((0, 1), (1, 1)), ((1, 0), (1, 1))],
            (0, 1): [((0, 0), (1, 1)), ((1, 1), (1, 1)), ((0, 2), (1, 1))],
            (0, 2): [((0, 1), (1, 1)), ((1, 2), (1, 1))],
            (1, 0): [((0, 0), (1, 1)), ((1, 1), (1, 1))],
            (1, 1): [((1, 0), (1, 1)), ((0, 1), (1, 1)), ((1, 2), (1, 1))],
            (1, 2): [((1, 1), (1, 1)), ((0, 2), (1, 1))]}

        mini_conf_problem.start_positions[1] = (0, 0)
        mini_conf_problem.start_positions[2] = (0, 2)
        mini_conf_problem.goal_positions[1] = (0, 2)
        mini_conf_problem.goal_positions[2] = (0, 0)
        mini_conf_problem.fill_heuristic_table()

        path_finder = ODAStar(mini_conf_problem)
        solution = path_finder.create_solution(time_limit=1000, objective='min_best_case', sic=True)
        self.assertEqual(solution[1], (6, 6))

    def test_incomplete_problem(self):
        mini_conf_problem = TimeUncertaintyProblem()
        mini_conf_problem.map = [[0, 0],
                                 [0, 0]]
        mini_conf_problem.width = 2
        mini_conf_problem.height = 2
        mini_conf_problem.edges_and_weights = {
            (0, 0): [((0, 1), (1, 1)), ((1, 1), (1, 1))],
            (0, 1): [((0, 0), (1, 1)), ((1, 1), (1, 1))],
            (1, 0): [((1, 1), (1, 2))],
            (1, 1): [((1, 0), (1, 1)), ((0, 0), (1, 1)), ((0, 1), (1, 1))]}

        mini_conf_problem.start_positions[1] = (0, 0)
        mini_conf_problem.start_positions[2] = (0, 1)
        mini_conf_problem.start_positions[3] = (1, 0)
        mini_conf_problem.goal_positions[1] = (0, 1)
        mini_conf_problem.goal_positions[2] = (1, 1)
        mini_conf_problem.goal_positions[3] = (0, 0)
        mini_conf_problem.fill_heuristic_table()

        path_finder = ODAStar(mini_conf_problem)
        with self.assertRaises(OutOfTimeError) as err:
            path_finder.create_solution(time_limit=3, objective='min_best_case', sic=True)


class TestOnlineCBSTU(unittest.TestCase):
    """
    Tests the online cbstu and simulator.
    """

    def test_simulator_no_broadcasting_always_sensing_simple(self):
        """
        Tests the online planner with a simple example where agents cannot communicate.
        """

        simple_tu = TimeUncertaintyProblem()
        simple_tu.edges_and_weights = {
            (0, 1): [((1, 1), (1, 1))],
            (0, 2): [((1, 2), (2, 9))],
            (1, 0): [((1, 1), (1, 5))],
            (1, 1): [((1, 0), (1, 5)), ((0, 1), (1, 1)), ((1, 2), (1, 1)), ((2, 1), (1, 1))],
            (1, 2): [((1, 1), (1, 1)), ((0, 2), (2, 9))],
            (2, 1): [((1, 1), (1, 1))]
        }
        simple_tu.start_positions = {1: (1, 0), 2: (0, 2)}
        simple_tu.goal_positions = {1: (0, 1), 2: (2, 1)}
        simple_tu.fill_heuristic_table()

        offline_cbstu_planner = CBSTUPlanner(simple_tu)
        sol = offline_cbstu_planner.find_solution()
        online_cbstu_planner = OnlineCBSTU(simple_tu)
        online_cbstu_planner.find_initial_path()
        self.assertEqual(sol.cost, (9, 20))
        self.assertEqual(online_cbstu_planner.initial_plan.cost, sol.cost)

        sim = MAPFSimulator(simple_tu, sensing_prob=1)
        sim.create_initial_solution()
        graphs, constraints = sim.create_plan_graphs_and_constraints()
        sensing_agent = {2: (1, 2)}
        sim.time = 5
        online_cbstu_planner.plan_distributed(graphs, constraints, sensing_agent, sim.time)
        online_cbstu_planner.current_plan.compute_solution_cost(sum_of_costs=True)
        self.assertEqual(online_cbstu_planner.current_plan.cost, (9, 13))