"""
File for running some experiments.
"""
import os.path
import gc
import sys
from pathfinding.planners.operator_decomposition_a_star import *
from pathfinding.simulator import *
from shutil import copyfile


class Experiments:

    def __init__(self, output_folder):
        self.output_folder = output_folder
        if not os.path.exists(self.output_folder):
            os.makedirs(self.output_folder)

        self.num_of_reps = 1
        self.min_best_case = False
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
                ccbs_sol = ccbs_planner.find_solution(min_best_case=True, time_lim=self.time_limit, soc=True,
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
                oda_star_queue_sol = oda_planner.find_solution(1, objective='min_best_case', sic=True,
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
            results = f'CCBS, {ccbs_total_time},' \
                f' {ccbs_success * 100}%, {ccbs_cost[0]} to {ccbs_cost[1]}, {ccbs_ratio}\n'
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

    def run_online_experiments(self, agent_num, sensing_prob, commy, reps, time_limit, uncertainty, dist):

        self.uncertainty = uncertainty
        self.agents_num = agent_num
        self.time_limit = time_limit
        self.reps = reps
        if self.min_best_case:
            objective = 'min best case'
        else:
            objective = 'min worst case'
        self.file_prefix = \
            f'{agent_num} agents - {self.uncertainty} uncertainty - {sensing_prob} sensing - comm {commy} -' \
            f' {objective} - distribution - {dist}'

        self.run_online_small_map(sensing_prob, commy, dist)
        #self.run_online_circular_map(sensing_prob, commy, dist)
        #self.run_online_warehouse_map(sensing_prob, commy, dist)
        # self.run_online_maze_map(sensing_prob, commy, dist)

    def run_online_small_map(self, sensing_prob, commy, distribution):
        map_type = 'small_open_map'
        results_file = self.file_prefix + f' - {map_type}_results.csv'
        map_file = '..\\..\\maps\\small_blank_map.map'
        if os.name == 'posix':
            map_file = './small_blank_map.map'
        print(
            f"- STARTED ONLINE BLANK MAP | {self.agents_num} AGENTS | UNCERTAINTY: {self.uncertainty} | SENSE: {sensing_prob} | COMM: {commy} | "
            f"DISTRIBUTION: {distribution} | MIN BEST TIME: {self.min_best_case}")

        map_seed = 96372106
        random.seed(map_seed)
        tu_problem = TimeUncertaintyProblem(map_file)
        tu_problem.generate_problem_instance(self.uncertainty)
        self.run_and_log_online_experiments(tu_problem, 'small_open_map', map_seed, results_file, sensing_prob, commy,
                                            distribution)

    def run_online_warehouse_map(self, sensing_prob, commy, distribution):
        map_type = 'warehouse_map'
        results_file = self.file_prefix + f' - {map_type}_results.csv'
        map_file = '..\\..\\maps\\kiva.map'
        if os.name == 'posix':
            map_file = './kiva.map'
        print(
            f"- STARTED ONLINE WAREHOUSE MAP | {self.agents_num} AGENTS | UNCERTAINTY: {self.uncertainty} | SENSE: {sensing_prob} | COMM: {commy} | "
            f"DISTRIBUTION: {distribution} | MIN BEST TIME: {self.min_best_case}")

        map_seed = 96372106
        random.seed(map_seed)
        tu_problem = TimeUncertaintyProblem(map_file)
        tu_problem.generate_problem_instance(self.uncertainty)
        self.run_and_log_online_experiments(tu_problem, 'warehouse_map', map_seed, results_file, sensing_prob, commy,
                                            distribution)

    def run_online_circular_map(self, sensing_prob, commy, distribution):
        map_type = 'circular_map'
        results_file = self.file_prefix + f' - {map_type}_results.csv'
        map_file = '..\\..\\maps\\ost003d.map'
        if os.name == 'posix':
            map_file = '../../maps/ost003d.map'
        print(
            f"- STARTED ONLINE CIRCULAR MAP | {self.agents_num} AGENTS | UNCERTAINTY: {self.uncertainty} | SENSE: {sensing_prob} | COMM: {commy} | "
            f"DISTRIBUTION: {distribution} | MIN BEST TIME: {self.min_best_case}")

        map_seed = 96372106
        random.seed(map_seed)
        tu_problem = TimeUncertaintyProblem(map_file)
        tu_problem.generate_problem_instance(self.uncertainty)
        self.run_and_log_online_experiments(tu_problem, map_type, map_seed, results_file, sensing_prob, commy,
                                            distribution)

    def run_online_maze_map(self, sensing_prob, commy, distribution):
        map_type = 'maze_map'
        results_file = self.file_prefix + f' - {map_type}_results.csv'
        map_file = '..\\..\\maps\\maze512-2-0.map'
        if os.name == 'posix':
            map_file = '../../maps/maze512-2-0.map'
        print(
            f"- STARTED ONLINE MAZE MAP | {self.agents_num} AGENTS | UNCERTAINTY: {self.uncertainty} | SENSE: {sensing_prob} | COMM: {commy} | "
            f"DISTRIBUTION: {distribution} | MIN BEST TIME: {self.min_best_case}")

        map_seed = 96372106
        random.seed(map_seed)
        # tu_problem = TimeUncertaintyProblem.generate_rectangle_map(height=20, width=20, uncertainty=self.uncertainty)
        tu_problem = TimeUncertaintyProblem(map_file)
        tu_problem.generate_problem_instance(self.uncertainty)
        self.run_and_log_online_experiments(tu_problem, map_type, map_seed, results_file, sensing_prob, commy,
                                            distribution)

    def run_and_log_online_experiments(self, tu_problem, map_type, map_seed, results_file, sensing_prob, communication,
                                       dist, to_print=True):
        final_folder = os.path.join(self.output_folder, map_type, f'{self.agents_num} agents')
        if not os.path.exists(final_folder):
            os.makedirs(final_folder)
        final_results_path = os.path.join(final_folder, results_file)
        temp_path = os.path.join(self.output_folder, 'IN PROGRESS - ' + results_file)
        with open(temp_path, 'w') as temp_file:
            temp_file.write('Experiment Number,'
                            'Map Seed,'
                            'Number of Agents,'
                            'Agents Seed,'
                            'Uncertainty,'
                            'Timeout,'
                            'initial time,'
                            'octu Time,'
                            'initial Min Cost,'
                            'initial Max Cost,'
                            'initial uncertainty,'
                            'initial true cost,'
                            'octu Min Cost,'
                            'octu Max Cost,'
                            'octu uncertainty,'
                            'final true cost,'
                            'Sensing Probability,'
                            'Distribution,'
                            'Objective,'
                            'Communication,'
                            'Min SIC,'
                            'Max SIC,'
                            'True SIC\n'
                            )

        success = 0
        initial_agent_seed = 10637299  # 10637296
        random.seed(initial_agent_seed)
        if os.name == 'posix':
            sol_folder = '../../solutions'
        else:
            sol_folder = '..\\..\\solutions'
        for i in range(self.reps):
            agent_seed = initial_agent_seed + i
            random.seed(agent_seed)
            if to_print:
                print(f'Started run #{i + 1}, agent seed: {agent_seed}, number of agents: {self.agents_num}')
            tu_problem.generate_agents(self.agents_num)
            try:
                tu_problem.fill_heuristic_table(self.min_best_case)
            except MemoryError:
                print("Memory Error when filling heuristic table..")
                i -= 1
                tu_problem.heuristic_table = None
                gc.collect()
                break
            sim = MAPFSimulator(tu_problem, sensing_prob, edge_dist=dist)

            try:
                loaded_sol = TimeUncertaintySolution.load(self.agents_num, self.uncertainty, map_type, agent_seed
                                                          , map_seed, self.min_best_case, sol_folder)
                if loaded_sol and loaded_sol.time_to_solve != -1 and loaded_sol.sic == (-1, -1):
                    loaded_sol = None

                if loaded_sol and loaded_sol.time_to_solve == -1:  # It was a timed out solution
                    raise OutOfTimeError
                start_time = time.time()
                online_sol = sim.begin_execution(self.min_best_case, time_limit=self.time_limit,
                                                 communication=communication, initial_sol=loaded_sol)
                octu_time = time.time() - start_time
                if not loaded_sol:
                    octu_time -= sim.online_planner.initial_plan.time_to_solve

                success += 1
                online_sol.create_movement_tuples()

                octu_cost = online_sol.cost
                octu_tu = octu_cost[1] - octu_cost[0]
                final_true_cost = sim.calc_solution_true_cost(online_sol)

                init_sol = sim.online_planner.initial_plan
                init_time = init_sol.time_to_solve
                init_cost = init_sol.cost
                init_tu = init_cost[1] - init_cost[0]
                init_true_cost = sim.calc_solution_true_cost(init_sol)

                min_sic = init_sol.sic[0]
                max_sic = init_sol.sic[1]
                root_sol = sim.online_planner.offline_cbstu_planner.create_root(self.min_best_case).sol
                true_sic = sim.calc_solution_true_cost(root_sol)

                if not loaded_sol:
                    init_sol.save(self.agents_num, self.uncertainty, map_type, agent_seed, map_seed, self.min_best_case,
                                  sol_folder)

            except OutOfTimeError:
                empty_sol = TimeUncertaintySolution()
                empty_sol.save(self.agents_num, self.uncertainty, map_type, agent_seed, map_seed, self.min_best_case,
                               sol_folder)
                octu_cost = -1, -1
                init_cost = -1, -1
                octu_time = -1
                init_tu = -1
                octu_tu = -1
                init_true_cost = -1
                final_true_cost = -1
                min_sic = -1
                max_sic = -1
                true_sic = -1
                if loaded_sol:
                    init_time = loaded_sol.time_to_solve
                elif sim.online_planner.initial_plan:
                    init_time = sim.online_planner.initial_plan.time_to_solve
                else:
                    init_time = -1
            except MemoryError:
                break

            with open(temp_path, 'a') as temp_map_result_file:
                if self.min_best_case:
                    objective = 'Min Best Case'
                else:
                    objective = 'Min Worst Case'
                results = f'{i + 1},' \
                    f'{map_seed},' \
                    f'{self.agents_num},' \
                    f'{agent_seed},' \
                    f'{self.uncertainty},' \
                    f'{self.time_limit},' \
                    f'{init_time},' \
                    f'{octu_time},' \
                    f'{init_cost[0]},' \
                    f'{init_cost[1]},' \
                    f'{init_tu},' \
                    f'{init_true_cost},' \
                    f'{octu_cost[0]},' \
                    f'{octu_cost[1]},' \
                    f'{octu_tu},' \
                    f'{final_true_cost},' \
                    f'{sensing_prob},' \
                    f'{dist},' \
                    f'{objective},' \
                    f'{communication},' \
                    f'{min_sic},' \
                    f'{max_sic}' \
                    f'{true_sic}\n'
                temp_map_result_file.write(results)

        copyfile(temp_path, final_results_path)
        os.remove(temp_path)

    def run_online_combinations(self, agent_num, time_uncer, sense_prob, reps=50, do_max=True, do_min=True, do_uni=True,
                                use_comm=True, no_comm=True, min_best_case=False, time_lim=60):

        sense_prob /= 100
        self.min_best_case = min_best_case
        if do_max:
            if use_comm:
                self.run_online_experiments(agent_num=agent_num, uncertainty=time_uncer, time_limit=time_lim, reps=reps,
                                            sensing_prob=sense_prob, commy=True, dist='max')
            if no_comm:
                self.run_online_experiments(agent_num=agent_num, uncertainty=time_uncer, time_limit=time_lim, reps=reps,
                                            sensing_prob=sense_prob, commy=False, dist='max')
        if do_min:
            if use_comm:
                self.run_online_experiments(agent_num=agent_num, uncertainty=time_uncer, time_limit=time_lim, reps=reps,
                                            sensing_prob=sense_prob, commy=True, dist='min')
            if no_comm:
                self.run_online_experiments(agent_num=agent_num, uncertainty=time_uncer, time_limit=time_lim, reps=reps,
                                            sensing_prob=sense_prob, commy=False, dist='min')

        if do_uni:
            if use_comm:
                self.run_online_experiments(agent_num=agent_num, uncertainty=time_uncer, time_limit=time_lim, reps=reps,
                                            sensing_prob=sense_prob, commy=True, dist='uniform')
            if no_comm:
                self.run_online_experiments(agent_num=agent_num, uncertainty=time_uncer, time_limit=time_lim, reps=reps,
                                            sensing_prob=sense_prob, commy=False, dist='uniform')


exp = Experiments('..\\..\\experiments\\Online Runs')
if os.name == 'posix':
    exp = Experiments('../../experiments/Online Runs')

for tu in range(0, 1):
    for number_of_agents in range(20, 30, 2):
        if tu == 3:
            continue
        for sense in range(0, 1, 50):
            exp.run_online_combinations(number_of_agents, tu, sense, reps=30, do_min=False, do_uni=False, do_max=True,
                                        use_comm=False, no_comm=True, min_best_case=False, time_lim=120)

print("Finished Experiments")
