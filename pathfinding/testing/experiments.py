"""
File for running some experiments.
"""
import os.path
import sys
from pathfinding.planners.operator_decomposition_a_star import *
from pathfinding.simulator import *


class Experiments:

    def __init__(self, output_folder):
        self.output_folder = output_folder
        if not os.path.exists(self.output_folder):
            os.makedirs(self.output_folder)

        self.num_of_reps = 1
        self.agents_num = 2
        self.uncertainty = 0
        self.time_limit = 300
        self.reps = 10
        self.file_prefix = 'default - '

    def run_blank_map(self, rep_num, agent_num):
        results_file = self.file_prefix + 'small_open_map_results.csv'
        map_file = '../maps/small_blank_map.map'
        seed = 12345678
        random.seed(seed)
        blank_problem = TimeUncertaintyProblem(map_file)
        blank_problem.generate_problem_instance(self.uncertainty)
        print(f"--- STARTED BLANK MAP | SEED: {seed} | UNCERTAINTY: {self.uncertainty} ---")
        self.run_and_log_same_instance_experiment(blank_problem, results_file, agent_num, rep_num, seed)

    def run_maze_map(self, rep_num, agent_num):
        results_file = self.file_prefix + 'maze_results.csv'
        seed = random.randrange(sys.maxsize)
        random.seed(seed)
        maze = TimeUncertaintyProblem.generate_rectangle_map(12, 12, self.uncertainty)
        print(f"--- STARTED MAZE MAP | SEED: {seed} | UNCERTAINTY: {self.uncertainty} ---")

        self.run_and_log_same_instance_experiment(maze, results_file, agent_num, rep_num, seed)

    def run_large_open_map(self, rep_num, agent_num):
        results_file = self.file_prefix + 'large_map_results.csv'
        map_file = '../maps/ost003d.map'
        seed = 12345678
        random.seed(seed)
        large_map = TimeUncertaintyProblem(map_file)
        large_map.generate_problem_instance(self.uncertainty)
        print(f"--- STARTED LARGE OPEN MAP | SEED: {seed} | UNCERTAINTY: {self.uncertainty} ---")

        self.run_and_log_same_instance_experiment(large_map, results_file, agent_num, rep_num, seed)

    def run_corridor_map(self, rep_num, agent_num, use_cat=True):
        results_file = self.file_prefix + 'narrow_corridor_results.csv'
        map_file = '../maps/brc202d.map'
        seed = 12345678
        random.seed(seed)
        corridor_map = TimeUncertaintyProblem(map_file)
        corridor_map.generate_problem_instance(self.uncertainty)
        print(f"--- STARTED CORRIDOR MAP | SEED: {seed}--- | UNCERTAINTY: {self.uncertainty} ---")
        self.run_and_log_same_instance_experiment(corridor_map, results_file, agent_num, rep_num, seed, use_cat)

    def run_circular_map(self, rep_num, agent_num):
        results_file = self.file_prefix + 'round_map_results.csv'
        map_file = '../maps/ost003d.map'
        seed = 12345678
        random.seed(seed)
        circular_map = TimeUncertaintyProblem(map_file)
        circular_map.generate_problem_instance(self.uncertainty)
        print(f"--- STARTED CIRCULAR MAP | SEED: {seed}--- | UNCERTAINTY: {self.uncertainty} ---")
        self.run_and_log_same_instance_experiment(circular_map, results_file, agent_num, rep_num, seed)

    def run_and_log_same_instance_experiment(self, conf_problem, results_file, agent_num, rep_num, map_seed,
                                             use_cat=True):
        with open(os.path.join(self.output_folder, results_file), 'w') as map_result_file:
            map_result_file.write('Experiment Number,Map Seed,Number of Agents, Agents Seed, Uncertainty,Timeout,'
                                  'CBS Time,ODA Queue Time, CCBS Min Cost, CCBS Max Cost, ODA Min Cost, ODA Max Cost,'
                                  ' CCBS nodes expanded, ODA nodes expanded\n')

        ccbs_total_time = 0
        oda_queue_total_time = 0

        ccbs_success = 0
        oda_queue_success = 0

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
            ccbs_planner = CBSTUPlanner(conf_problem)
            oda_planner = ODAStar(conf_problem)
            ccbs_sol = None
            oda_star_queue_sol = None

            print(f'Started run #{i + 1}, agent seed: {agent_seed}, number of agents: {agent_num}')

            try:
                start_time = time.time()
                ccbs_sol = ccbs_planner.find_solution(min_best_case=True, time_limit=self.time_limit, soc=True,
                                                      use_cat=use_cat)
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
                oda_star_queue_sol = oda_planner.create_solution(1, objective='min_best_case', sic=True,
                                                                 min_time_policy=True)  # ToDO: CHANGE BACK TIME LIMIT
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
                # self.write_solution(ccbs_sol, oda_star_queue_sol)
                i -= 1
                continue
                # raise RuntimeError

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
                results = f'{i + 1},{map_seed},{agent_num}, {agent_seed}, {self.uncertainty},{self.time_limit},' \
                    f'{ccbs_time}, {oda_queue_time}, {ccbs_min}, {ccbs_max}, {oda_min}, {oda_max},' \
                    f' {ccbs_nodes_expanded}, {oda_nodes}\n'
                map_result_file.write(results)

        # Write final results.
        if ccbs_success > 0:
            ccbs_cost[0] /= ccbs_success  # We'll only divide by successful runs.
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
            results = f'CCBS, {ccbs_total_time}, {ccbs_success * 100}%, {ccbs_cost[0]} to {ccbs_cost[1]}, {ccbs_ratio}\n'
            results += f'ODA-queue, {oda_queue_total_time}, {oda_queue_success * 100}%, {oda_queue_cost[0]} to' \
                f' {oda_queue_cost[1]}, {oda_queue_ratio}\n'
            map_result_file.write(results)

    @staticmethod
    def write_solution(cbs_sol, oda_sol):
        error_file = f'.\\{len(cbs_sol.paths)} Agents.csv'
        with open(error_file, 'w') as file:
            file.write('CBS Solution\n')
            done = False
            i = 0
            while not done:
                for agent, path in cbs_sol.paths.items():
                    pass

    def run_experiments_on_same_instance(self, num_of_agents, uncertainty, time_limit, rep_num, use_cat):
        self.uncertainty = uncertainty
        self.agents_num = num_of_agents
        self.time_limit = time_limit

        self.file_prefix = f'{num_of_agents} agents -  {self.uncertainty} uncertainty - '

        self.run_blank_map(rep_num, num_of_agents)
        self.run_circular_map(rep_num, num_of_agents)
        self.run_corridor_map(rep_num, num_of_agents, use_cat)

    def run_online_experiments(self, agent_num, sensing_prob, comm, reps, time_limit, uncertainty):

        self.uncertainty = uncertainty
        self.agents_num = agent_num
        self.time_limit = time_limit
        self.reps = reps
        self.file_prefix = \
            f'{agent_num} agents - {self.uncertainty} uncertainty - {sensing_prob} sensing - comm {comm} - '
        results_file = self.file_prefix + 'small_open_map_results.csv'
        map_file = '..\\..\\maps\\small_blank_map.map'
        print(f"- STARTED ONLINE BLANK MAP | UNCERTAINTY: {self.uncertainty} | SENSE: {sensing_prob} | COMM: {comm} -")

        map_seed = 96372106
        random.seed(map_seed)
        tu_problem = TimeUncertaintyProblem(map_file)
        tu_problem.generate_problem_instance(self.uncertainty)
        self.run_and_log_online_experiments(tu_problem, map_seed, results_file, sensing_prob, comm)

    def run_and_log_online_experiments(self, tu_problem, map_seed, results_file, sensing_prob, comm):
        results_path = os.path.join(self.output_folder, results_file)
        with open(results_path, 'w') as result_file:
            result_file.write('Experiment Number,'
                              'Map Seed,'
                              'Number of Agents,'
                              'Agents Seed,'
                              'Uncertainty,'
                              'Timeout,'
                              'octu Time,'
                              'initial Min Cost,'
                              'initial Max Cost,'
                              'initial uncertainty,'
                              'octu Min Cost,'
                              'octu Max Cost,'
                              'octu uncertainty,'
                              'Sensing Probability,'
                              'Communication\n')

        success = 0
        initial_agent_seed = 10637296
        random.seed(initial_agent_seed)

        for i in range(self.reps):
            agent_seed = initial_agent_seed + i
            random.seed(agent_seed)
            print(f'Started run #{i + 1}, agent seed: {agent_seed}, number of agents: {self.agents_num}')
            tu_problem.generate_agents(self.agents_num)
            try:
                tu_problem.fill_heuristic_table()
            except MemoryError:
                print("Memory Error when filling heuristic table..")
                i -= 1
                tu_problem.heuristic_table = None
                continue
            sim = MAPFSimulator(tu_problem, sensing_prob)

            try:
                start_time = time.time()
                online_sol = sim.begin_execution(time_limit=self.time_limit, communication=comm)
                init_cost = sim.online_CSTU.initial_plan.cost
                octu_time = time.time() - start_time
                success += 1
                octu_cost = online_sol.cost
                init_tu = init_cost[1] - init_cost[0]
                octu_tu = octu_cost[1] - octu_cost[0]
            except OutOfTimeError:
                octu_cost = -1, -1
                init_cost = -1, -1
                octu_time = -1
                init_tu = -1
                octu_tu = -1

            with open(os.path.join(self.output_folder, results_file), 'a') as map_result_file:
                results = f'{i + 1},' \
                    f'{map_seed},' \
                    f'{self.agents_num},' \
                    f'{agent_seed},' \
                    f'{self.uncertainty},' \
                    f'{self.time_limit},' \
                    f'{octu_time},' \
                    f'{init_cost[0]},' \
                    f'{init_cost[1]},' \
                    f'{init_tu},' \
                    f'{octu_cost[0]},' \
                    f'{octu_cost[1]},' \
                    f'{octu_tu},' \
                    f'{sensing_prob},' \
                    f'{comm}\n'
                map_result_file.write(results)


exp = Experiments('..\\..\\experiments\\Online Runs')
if os.name == 'posix':
    exp = Experiments('../../experiments/Online Runs')

comm = False
for agent_num in range(7, 8):
    for tu in range(2, 3):
        if tu == 3:
            continue
        for sense in range(0 , 101, 25):
            sense_prob = sense / 100
            exp.run_online_experiments(agent_num=agent_num, uncertainty=tu, time_limit=60, reps=50,
                                       sensing_prob=sense_prob, comm=comm)

print("Finished Experiments")
