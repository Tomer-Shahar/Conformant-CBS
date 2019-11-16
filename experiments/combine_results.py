import os
import csv

raw_data_file = 'C:\\Users\\Tomer\\PycharmProjects\\Conformant-CBS\\experiments\\All Raw Online Data.csv'
average_results_file = 'C:\\Users\\Tomer\\PycharmProjects\\Conformant-CBS\\experiments\\Average Online Results.csv'
input_folder = 'C:\\Users\\Tomer\\PycharmProjects\\Conformant-CBS\\experiments\\Online Runs'
sol_folder = 'C:\\Users\\Tomer\\PycharmProjects\\Conformant-CBS\\solutions'


def get_map_type(file_name):
    map_name = file_name.split('-')[-1].split('.csv')[0]
    if 'maze' in map_name:
        return 'Maze'
    if 'circular' in map_name:
        return 'Circular'
    if 'open' in map_name:
        return 'Open Map'
    if 'warehouse' in map_name:
        return 'Warehouse Map'
    else:
        return 'unknown'


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
            dist = file.split('distribution - ')[1].split(' -')[0]
            if dist != row['Distribution']:
                print(run_file)
            num_of_runs += 1
            write_simulation_results(map_type, row, dist=dist)


def calc_averages_and_write(map_type,
                            initial_time,
                            octu_time,
                            initial_min_cost,
                            initial_max_cost,
                            initial_uncertainty,
                            initial_true_cost,
                            final_min_cost,
                            final_max_cost,
                            final_uncertainty,
                            final_true_cost,
                            objective,
                            uncertainty,
                            sensing_probability,
                            num_of_agents,
                            num_of_runs,
                            octu_success,
                            comm,
                            distribution,
                            reduced_TC,
                            increased_TC,
                            same_TC,
                            min_sic,
                            max_sic):
    reduction_in_tc = -1

    if octu_success > 0:
        initial_time /= octu_success
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
        min_sic /= octu_success
        max_sic /= octu_success
        octu_success /= num_of_runs

    average_writer.writerow({
        'Map': map_type,
        'Uncertainty': uncertainty,
        'Number of Agents': num_of_agents,
        'With Communication': comm,
        'Sensing Probability': sensing_probability,
        'Initial Runtime (secs)': initial_time,
        'Online Runtime (secs)': octu_time,
        'Success': octu_success,
        'Initial Min SOC': initial_min_cost,
        'Initial Max SOC': initial_max_cost,
        'Initial Uncertainty': initial_uncertainty,
        'Initial True Cost': initial_true_cost,
        'Final Min SOC': final_min_cost,
        'Final Max SOC': final_max_cost,
        'Final Uncertainty': final_uncertainty,
        'Final True Cost': final_true_cost,
        'Objective': objective,
        'Reduction in True Cost': reduction_in_tc,
        'Distribution': distribution,
        'Number of Runs': num_of_runs,
        'Reduced True Cost': reduced_TC,
        'Increased True Cost': increased_TC,
        'Same True Cost': same_TC,
        'Min SIC': min_sic,
        'Max SIC': max_sic
    })


def write_average_results(run_file):
    exp_results = os.path.join(root, run_file)
    with open(exp_results, 'r') as exp_file:
        reader = csv.DictReader(exp_file)
        map_type = get_map_type(run_file)
        # Values that must be added together in order to calculate the average
        initial_time = 0
        online_time = 0
        initial_min_cost = 0
        initial_max_cost = 0
        initial_uncertainty = 0
        initial_true_cost = 0
        final_min_cost = 0
        final_max_cost = 0
        final_uncertainty = 0
        final_true_cost = 0
        reduced_tc = 0
        increased_tc = 0
        same_tc = 0
        uncertainty = -1
        sensing_probability = -1
        num_of_agents = -1
        num_of_runs = 0
        octu_success = 0
        min_sic = 0
        max_sic = 0

        for row in reader:
            if row['Map Seed'] == '':  # Reached end of experiments.
                break
            num_of_runs += 1
            uncertainty = row['Uncertainty']
            distribution = run_file.split('distribution - ')[1].split(' -')[0]  # row['Distribution']
            sensing_probability = row['Sensing Probability']
            num_of_agents = row['Number of Agents']
            comm = row['Communication']
            try:
                objective = row['Objective']
            except KeyError:
                objective = 'Min Worst Case'

            if float(row['octu Time']) >= 0:  # A successful run
                octu_success += 1
                initial_time += float(row['initial time'])
                online_time += float(row['octu Time'])
                initial_min_cost += float(row['initial Min Cost'])
                initial_max_cost += float(row['initial Max Cost'])
                initial_uncertainty += float(row['initial uncertainty'])
                initial_true_cost += float(row['initial true cost'])
                final_min_cost += float(row['octu Min Cost'])
                final_max_cost += float(row['octu Max Cost'])
                final_uncertainty += float(row['octu uncertainty'])
                final_true_cost += float(row['final true cost'])
                min_sic += float(row['Min SIC'])
                max_sic += float(row['Max SIC'])
                if float(row['final true cost']) < float(row['initial true cost']):
                    reduced_tc += 1
                elif float(row['final true cost']) > float(row['initial true cost']):
                    increased_tc += 1
                else:
                    same_tc += 1

        # if num_of_runs < 50:  # There's a mistake
        #    print(f'{run_file} ::: {num_of_runs}')

        calc_averages_and_write(map_type, initial_time,  online_time, initial_min_cost, initial_max_cost,
                                initial_uncertainty, initial_true_cost, final_min_cost, final_max_cost,
                                final_uncertainty, final_true_cost, objective, uncertainty, sensing_probability,
                                num_of_agents, num_of_runs, octu_success, comm, distribution, reduced_tc, increased_tc,
                                same_tc, min_sic, max_sic)


def write_simulation_results(map_type, row, dist=None):

    tc_change = int(row['initial true cost']) - int(row['final true cost'])
    if tc_change > 0:
        effect = 'Increased True Cost'
    elif tc_change < 0:
        effect = 'Reduced True Cost'
    else:
        effect = 'No Change'

    objective = row['Objective']

    raw_data_writer.writerow({
        'Map': map_type,
        'Map Seed': row['Map Seed'],
        'Uncertainty': row['Uncertainty'],
        'Number of Agents': row['Number of Agents'],
        'Agent Seed': row['Agents Seed'],
        'With Communication': row['Communication'],
        'Sensing Probability': row['Sensing Probability'],
        'Initial Runtime (secs)': row['initial time'],
        'Online Runtime (secs)': row['octu Time'],
        'Success': 1 if float(row['octu Time']) >= 0 else 0,
        'Initial Min SOC': row['initial Min Cost'],
        'Initial Max SOC': row['initial Max Cost'],
        'Initial Uncertainty': row['initial uncertainty'],
        'Initial True Cost': row['initial true cost'],
        'Final Min SOC': row['octu Min Cost'],
        'Final Max SOC': row['octu Max Cost'],
        'Final Uncertainty': row['octu uncertainty'],
        'Distribution': dist,  # row['Distribution'],
        'Final True Cost': row['final true cost'],
        'Objective': objective,
        'Change in True Cost': str(int(row['final true cost']) - int(row['initial true cost'])),
        'Effect on True Cost': effect,
        'True Cost Change': str(int(row['final true cost']) - int(row['initial true cost'])),
        'Min SIC': str(row['Min SIC']),
        'Max SIC': str(row['Max SIC'])
    })


#for root, dirs, files in os.walk(input_folder):
#    for file in files:
#        if 'min best case' in file and 'min worst case' in file:
#            name_arr = file.split(' - min worst case - min worst case - ')
#            new_name = ' - '.join(name_arr)
#            os.rename(os.path.join(root, file), os.path.join(root, new_name))
#            print('yippy')

# Aggregate all of the data into a single large file
with open(raw_data_file, 'w', newline='') as raw_file:
    fields = ['Map', 'Map Seed', 'Uncertainty', 'Number of Agents', 'Agent Seed', 'With Communication',
              'Sensing Probability', 'Initial Runtime (secs)', 'Online Runtime (secs)', 'Success', 'Initial Min SOC',
              'Initial Max SOC', 'Initial Uncertainty', 'Initial True Cost', 'Final Min SOC', 'Final Max SOC',
              'Final Uncertainty', 'Distribution', 'Final True Cost', 'Objective', 'Effect on True Cost',
              'True Cost Change', 'Min SIC', 'Max SIC']
    raw_data_writer = csv.DictWriter(raw_file, fieldnames=fields, restval='-', extrasaction='ignore')
    raw_data_writer.writeheader()
    num = 0
    for root, dirs, files in os.walk(input_folder):
        for run_file in files:
            if 'IN PROGRESS' in run_file:
                continue
            if 'desktop' in run_file:
                continue
            append_simulation_file(run_file)
            num += 1

# Compute the average results of each setting
with open(average_results_file, 'w', newline='') as avg_file:
    avg_fields = ['Map', 'Uncertainty', 'Number of Agents', 'With Communication', 'Sensing Probability',
                  'Initial Runtime (secs)', 'Online Runtime (secs)', 'Success', 'Initial Min SOC', 'Initial Max SOC',
                  'Initial Uncertainty',  'Initial True Cost', 'Final Min SOC', 'Final Max SOC', 'Final Uncertainty',
                  'Final True Cost', 'Objective',  'Reduction in True Cost', 'Distribution', 'Number of Runs',
                  'Reduced True Cost', 'Increased True Cost', 'Same True Cost', 'Min SIC', 'Max SIC']
    average_writer = csv.DictWriter(avg_file, fieldnames=avg_fields)
    average_writer.writeheader()
    for root, dirs, files in os.walk(input_folder):
        for run_file in files:
            if 'IN PROGRESS' in run_file:
                continue
            if 'desktop' in run_file:
                os.remove(os.path.join(root, run_file))
                continue
            write_average_results(run_file)

    print(f'Done writing and processing {num} files')
