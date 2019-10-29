import os
import csv

raw_data_file = 'C:\\Users\\Tomer\\PycharmProjects\\Conformant-CBS\\experiments\\All Raw Online Data.csv'
average_results_file = 'C:\\Users\\Tomer\\PycharmProjects\\Conformant-CBS\\experiments\\Average Online Results.csv'
input_folder = 'C:\\Users\\Tomer\\PycharmProjects\\Conformant-CBS\\experiments\\Online Runs'


def get_map_type(file_name):
    map_name = file_name.split('-')[-1].split('.csv')[0]
    if 'maze' in map_name:
        return 'Maze'
    if 'round' in map_name:
        return 'Circular'
    if 'open' in map_name:
        return 'Open Map'
    else:
        return 'Corridor Map'


def append_simulation_file(file):
    exp_results = os.path.join(root, file)
    with open(exp_results, 'r') as exp_file:
        reader = csv.DictReader(exp_file)
        map_type = get_map_type(file)
        num_of_runs = 0
        for row in reader:
            if row['Map Seed'] == '' or row['Experiment Number'] == 'Algorithm':  # Reached end of experiments.
                break
            if int(row['Map Seed']) != 96372106:
                print(f'Wrong map seed, FILE: {file}')
                return
            if 10637296 > int(row['Agents Seed']) or int(row['Agents Seed']) > 10637345:
                print(f'Wrong agent seed, FILE: {file}')
                return
            num_of_runs += 1
            write_simulation_results(map_type, row)


def calc_averages_and_write(map_type,
                            octu_time,
                            initial_min_cost,
                            initial_max_cost,
                            initial_uncertainty,
                            initial_true_cost,
                            final_min_cost,
                            final_max_cost,
                            final_uncertainty,
                            final_true_cost,
                            uncertainty,
                            sensing_probability,
                            num_of_agents,
                            num_of_runs,
                            octu_success,
                            comm,
                            distribution):
    reduction_in_tc = -1

    if octu_success > 0:
        octu_time /= octu_success
        initial_min_cost /= octu_success
        initial_max_cost /= octu_success
        initial_uncertainty /= octu_success
        initial_true_cost /= octu_success
        final_min_cost /= octu_success
        final_max_cost /= octu_success
        final_uncertainty /= octu_success
        final_true_cost /= octu_success
        reduction_in_tc = initial_true_cost - final_true_cost
        octu_success /= num_of_runs

    average_writer.writerow({
        'Map': map_type,
        'Uncertainty': uncertainty,
        'Number of Agents': num_of_agents,
        'With Communication': comm,
        'Sensing Probability': sensing_probability,
        'Runtime (secs)': octu_time,
        'Success': octu_success,
        'Initial Min SOC': initial_min_cost,
        'Initial Max SOC': initial_max_cost,
        'Initial Uncertainty': initial_uncertainty,
        'Initial True Cost': initial_true_cost,
        'Final Min SOC': final_min_cost,
        'Final Max SOC': final_max_cost,
        'Final Uncertainty': final_uncertainty,
        'Final True Cost': final_true_cost,
        'Reduction in True Cost': reduction_in_tc,
        'Distribution': distribution,
        'Number of Runs': num_of_runs
    })


def write_average_results():
    exp_results = os.path.join(root, run_file)
    with open(exp_results, 'r') as exp_file:
        reader = csv.DictReader(exp_file)
        map_type = get_map_type(run_file)

        # Values that must be added together in order to calculate the average
        octu_time = 0
        initial_min_cost = 0
        initial_max_cost = 0
        initial_uncertainty = 0
        initial_true_cost = 0
        final_min_cost = 0
        final_max_cost = 0
        final_uncertainty = 0
        final_true_cost = 0
        uncertainty = -1
        sensing_probability = -1
        num_of_agents = -1
        num_of_runs = 0
        octu_success = 0

        for row in reader:
            if row['Map Seed'] == '':  # Reached end of experiments.
                break
            num_of_runs += 1
            uncertainty = row['Uncertainty']
            distribution = row['Distribution']
            sensing_probability = row['Sensing Probability']
            num_of_agents = row['Number of Agents']
            comm = row['Communication']
            if float(row['octu Time']) >= 0:  # A successful run
                octu_success += 1
                octu_time += float(row['octu Time'])
                initial_min_cost += float(row['initial Min Cost'])
                initial_max_cost += float(row['initial Max Cost'])
                initial_uncertainty += float(row['initial uncertainty'])
                initial_true_cost += float(row['initial true cost'])
                final_min_cost += float(row['octu Min Cost'])
                final_max_cost += float(row['octu Max Cost'])
                final_uncertainty += float(row['octu uncertainty'])
                final_true_cost += float(row['final true cost'])

        # if num_of_runs < 50:  # There's a mistake
        #    print(f'{run_file} ::: {num_of_runs}')

        calc_averages_and_write(map_type,
                                octu_time,
                                initial_min_cost,
                                initial_max_cost,
                                initial_uncertainty,
                                initial_true_cost,
                                final_min_cost,
                                final_max_cost,
                                final_uncertainty,
                                final_true_cost,
                                uncertainty,
                                sensing_probability,
                                num_of_agents,
                                num_of_runs,
                                octu_success,
                                comm,
                                distribution)


def write_simulation_results(map_type, row):
    raw_data_writer.writerow({
        'Map': map_type,
        'Map Seed': row['Map Seed'],
        'Uncertainty': row['Uncertainty'],
        'Number of Agents': row['Number of Agents'],
        'Agent Seed': row['Agents Seed'],
        'With Communication': row['Communication'],
        'Sensing Probability': row['Sensing Probability'],
        'Runtime (secs)': row['octu Time'],
        'Success': 1 if float(row['octu Time']) >= 0 else 0,
        'Initial Min SOC': row['initial Min Cost'],
        'Initial Max SOC': row['initial Max Cost'],
        'Initial Uncertainty': row['initial uncertainty'],
        'Initial True Cost': row['initial true cost'],
        'Final Min SOC': row['octu Min Cost'],
        'Final Max SOC': row['octu Max Cost'],
        'Final Uncertainty': row['octu uncertainty'],
        'Distribution': row['Distribution'],
        'Final True Cost': row['final true cost'],
    })


# Aggregate all of the data into a single large file
with open(raw_data_file, 'w', newline='') as raw_file:
    fields = ['Map', 'Map Seed', 'Uncertainty', 'Number of Agents', 'Agent Seed', 'With Communication',
              'Sensing Probability', 'Runtime (secs)', 'Success', 'Initial Min SOC', 'Initial Max SOC',
              'Initial Uncertainty', 'Initial True Cost', 'Final Min SOC', 'Final Max SOC', 'Final Uncertainty',
              'Distribution', 'Final True Cost']
    raw_data_writer = csv.DictWriter(raw_file, fieldnames=fields, restval='-', extrasaction='ignore')
    raw_data_writer.writeheader()
    num = 0
    for root, dirs, files in os.walk(input_folder):
        for run_file in files:
            if 'IN PROGRESS' in run_file:
                continue
            append_simulation_file(run_file)
            num += 1

# Compute the average results of each setting
with open(average_results_file, 'w', newline='') as avg_file:
    avg_fields = ['Map', 'Uncertainty', 'Number of Agents', 'With Communication', 'Sensing Probability',
                  'Runtime (secs)', 'Success', 'Initial Min SOC', 'Initial Max SOC', 'Initial Uncertainty',
                  'Initial True Cost', 'Final Min SOC', 'Final Max SOC', 'Final Uncertainty', 'Final True Cost',
                  'Reduction in True Cost', 'Distribution', 'Number of Runs']
    average_writer = csv.DictWriter(avg_file, fieldnames=avg_fields)
    average_writer.writeheader()
    for root, dirs, files in os.walk(input_folder):
        for run_file in files:
            write_average_results()

    print(f'Done writing and processing {num} files')
