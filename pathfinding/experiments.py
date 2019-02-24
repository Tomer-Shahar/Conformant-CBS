"""
File for running some experiments.
"""
import os.path
import csv
import sys
from pathfinding.map_reader import *
from pathfinding.conformant_cbs import *
from pathfinding.operator_decomposition_a_star import *


class Experiments:

    def __init__(self, output_folder):
        self.output_folder = output_folder
        if not os.path.exists(self.output_folder):
            os.makedirs(self.output_folder)

        self.num_of_reps = 1
        self.max_agents_num = 2
        self.uncertainty = 0
        self.time_limit = 300
        self.file_prefix = 'default - '

    def run_blank_map(self, rep_num, agent_num):
        results_file = self.file_prefix + 'small_open_map_results.csv'
        map_file = '../maps/small_blank_map.map'
        seed = 12345678
        random.seed(seed)
        blank_problem = ConformantProblem(map_file)
        blank_problem.generate_problem_instance(self.uncertainty)
        print(f"--- STARTED BLANK MAP | SEED: {seed} | UNCERTAINTY: {self.uncertainty} ---")
        self.run_and_log_same_instance_experiment(blank_problem, results_file, agent_num, rep_num, seed)

    def run_maze_map(self, rep_num, agent_num):
        results_file = self.file_prefix + 'maze_results.csv'
        seed = random.randrange(sys.maxsize)
        random.seed(seed)
        maze = ConformantProblem.generate_rectangle_map(12, 12, self.uncertainty)
        print(f"--- STARTED MAZE MAP | SEED: {seed} | UNCERTAINTY: {self.uncertainty} ---")

        self.run_and_log_same_instance_experiment(maze, results_file, agent_num, rep_num, seed)

    def run_large_open_map(self, rep_num, agent_num):
        results_file = self.file_prefix + 'large_map_results.csv'
        map_file = '../maps/ost003d.map'
        seed = 12345678
        random.seed(seed)
        large_map = ConformantProblem(map_file)
        large_map.generate_problem_instance(self.uncertainty)
        print(f"--- STARTED LARGE OPEN MAP | SEED: {seed} | UNCERTAINTY: {self.uncertainty} ---")

        self.run_and_log_same_instance_experiment(large_map, results_file, agent_num, rep_num, seed)

    def run_corridor_map(self, rep_num, agent_num):
        results_file = self.file_prefix + 'narrow_corridor_results.csv'
        map_file = '../maps/brc202d.map'
        seed = 12345678
        random.seed(seed)
        corridor_map = ConformantProblem(map_file)
        corridor_map.generate_problem_instance(self.uncertainty)
        print(f"--- STARTED CORRIDOR MAP | SEED: {seed}--- | UNCERTAINTY: {self.uncertainty} ---")
        self.run_and_log_same_instance_experiment(corridor_map, results_file, agent_num, rep_num, seed)

    def run_circular_map(self, rep_num, agent_num):
        results_file = self.file_prefix + 'round_map_results.csv'
        map_file = '../maps/ost003d.map'
        seed = 12345678
        random.seed(seed)
        circular_map = ConformantProblem(map_file)
        circular_map.generate_problem_instance(self.uncertainty)
        print(f"--- STARTED CIRCULAR MAP | SEED: {seed}--- | UNCERTAINTY: {self.uncertainty} ---")
        self.run_and_log_same_instance_experiment(circular_map, results_file, agent_num, rep_num, seed)

    def run_and_log_same_instance_experiment(self, conf_problem, results_file, agent_num, rep_num, map_seed):
        with open(os.path.join(self.output_folder, results_file), 'w') as map_result_file:
            map_result_file.write('Experiment Number,Map Seed,Number of Agents, Agents Seed, Uncertainty,Timeout,'
                                  'CBS Time,ODA Queue Time, CCBS Min Cost, CCBS Max Cost, ODA Min Cost, ODA Max Cost,'
                                  ' CCBS nodes expanded, ODA nodes expanded\n')

        ccbs_total_time = 0
        oda_queue_total_time = 0

        ccbs_success = 0
        oda_queue_success = 0

        ccbs_time = 0
        oda_queue_time = 0

        ccbs_cost = [0, 0]
        oda_queue_cost = [0, 0]

        for i in range(rep_num):
            agent_seed = random.randrange(sys.maxsize)
            random.seed(agent_seed)
            conf_problem.generate_agents(agent_num)
            try:
                conf_problem.fill_heuristic_table()
            except MemoryError:
                print("Memory Error when filling heuristic table..")
                i -= 1
                conf_problem.heuristic_table = None
                continue
            ccbs_planner = ConformantCbsPlanner(conf_problem)
            oda_planner = ODAStar(conf_problem)
            ccbs_sol = None
            oda_star_queue_sol = None

            print(f'Started run #{i + 1}, agent seed: {agent_seed}, number of agents: {agent_num}')

            try:
                start_time = time.time()
                ccbs_sol = ccbs_planner.find_solution(min_best_case=True, time_limit=self.time_limit, sum_of_costs=True)
                if ccbs_sol:
                    ccbs_time = time.time() - start_time
                    ccbs_success += 1
                    ccbs_total_time += ccbs_time
                    ccbs_cost[0] += ccbs_sol.cost[0]
                    ccbs_cost[1] += ccbs_sol.cost[1]
                else:
                    i -= 1
                    continue
            except OutOfTimeError:
                ccbs_time = -1

            try:
                start_time = time.time()
                oda_star_queue_sol = oda_planner.create_solution(self.time_limit, objective='min_best_case', sic=True,
                                                                 min_time_policy=True)
                if oda_star_queue_sol:
                    oda_queue_time = time.time() - start_time
                    oda_queue_success += 1
                    oda_queue_total_time += oda_queue_time
                    oda_queue_cost[0] += oda_star_queue_sol[1][0]
                    oda_queue_cost[1] += oda_star_queue_sol[1][1]
            except OutOfTimeError:
                oda_queue_time = -1
            except MemoryError:
                print("ODA Memory Error :-(")
                oda_queue_time = -2

            if ccbs_sol and oda_star_queue_sol and (ccbs_sol.cost[0] != oda_star_queue_sol[1][0]):
                print("ccbs cost: " + str(ccbs_sol.cost))
                print("oda cost: " + str(oda_star_queue_sol[1]))
                #self.write_solution(ccbs_sol, oda_star_queue_sol)
                i -= 1
                continue
                #raise RuntimeError

            if ccbs_sol:
                ccbs_min = ccbs_sol.cost[0]
                ccbs_max = ccbs_sol.cost[1]
                ccbs_nodes_expanded = ccbs_sol.nodes_expanded
            else:
                ccbs_min = -1
                ccbs_max = -1
                ccbs_nodes_expanded = -1

            if oda_star_queue_sol:
                oda_min = oda_star_queue_sol[1][0]
                oda_max = oda_star_queue_sol[1][1]
                oda_nodes = oda_star_queue_sol[2]
            else:
                oda_min = -1
                oda_max = -1
                oda_nodes = -1
            with open(os.path.join(self.output_folder, results_file), 'a') as map_result_file:
                results = f'{i +1},{map_seed},{agent_num}, {agent_seed}, {self.uncertainty},{self.time_limit},' \
                    f'{ccbs_time}, {oda_queue_time}, {ccbs_min}, {ccbs_max}, {oda_min}, {oda_max},' \
                    f' {ccbs_nodes_expanded}, {oda_nodes}\n'
                map_result_file.write(results)

        # Write final results.
        if ccbs_success > 0:
            ccbs_cost[0] /= ccbs_success   # We'll only divide by successful runs.
            ccbs_cost[1] /= ccbs_success

        if oda_queue_success > 0:
            oda_queue_cost[0] /= oda_queue_success
            oda_queue_cost[1] /= oda_queue_success

        ccbs_success /= rep_num
        oda_queue_success /= rep_num

        ccbs_total_time /= rep_num
        oda_queue_total_time /= rep_num

        with open(os.path.join(self.output_folder, results_file), 'a') as map_result_file:
            header = '\nAlgorithm, Average Run Time, Success Rate, Average Cost, Ratio \n'
            map_result_file.write(header)
            ccbs_ratio, oda_queue_ratio = self.calc_ratio(ccbs_total_time, oda_queue_total_time)
            results = f'CCBS, {ccbs_total_time}, {ccbs_success*100}%, {ccbs_cost[0]} to {ccbs_cost[1]}, {ccbs_ratio}\n'
            results += f'ODA-queue, {oda_queue_total_time}, {oda_queue_success*100}%, {oda_queue_cost[0]} to {oda_queue_cost[1]}, {oda_queue_ratio}\n'
            map_result_file.write(results)

    @staticmethod
    def write_solution(cbs_sol, oda_sol):
        error_file = f'.\\{len(cbs_sol.paths)} Agents.csv'
        with open(error_file, 'w') as file:
            file.write('CBS Solution\n')
            done = False
            i=0
            while not done:
                for agent, path in cbs_sol.paths.items():
                    pass
    @staticmethod
    def calc_ratio(ccbs_total_time, oda_queue_total_time):
        min_time = min(ccbs_total_time, oda_queue_total_time)

        ccbs_ratio = -1
        oda_queue_ratio = -1

        if ccbs_total_time != 0 and min_time != 0:
            ccbs_ratio = ccbs_total_time / min_time

        if oda_queue_total_time != 0 and min_time != 0:
            oda_queue_ratio = oda_queue_total_time / min_time

        return ccbs_ratio, oda_queue_ratio

    def run_experiments_on_same_instance(self, num_of_agents, uncertainty, time_limit, rep_num):
        self.uncertainty = uncertainty
        self.max_agents_num = num_of_agents
        self.time_limit = time_limit

        self.file_prefix = f'{num_of_agents} agents -  {self.uncertainty} uncertainty - '

        self.run_blank_map(rep_num, num_of_agents)
        self.run_circular_map(rep_num, num_of_agents)
        self.run_corridor_map(rep_num, num_of_agents)


if os.name == 'nt':
    exp = Experiments('.\\..\\experiments')
elif os.name == 'posix':
    exp = Experiments('./../experiments')
for uncertainty_val in range(0, 5, 1):
    if uncertainty_val == 3:
        continue
    for agent_num in range(6, 7):
        exp.run_experiments_on_same_instance(num_of_agents=agent_num, uncertainty=uncertainty_val, time_limit=60, rep_num=30)

print("Finished Experiments")