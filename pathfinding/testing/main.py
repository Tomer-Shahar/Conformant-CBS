import sys
import os
import json
path = os.getcwd().split(os.path.sep)
path = os.path.sep.join(path[:-1])
sys.path.append(path)
proj_path = os.path.abspath(os.getcwd())
proj_path = proj_path.split('Conformant-CBS')
proj_path = os.path.join(proj_path[0], 'Conformant-CBS', 'pathfinding', 'testing')
previous_settings_file_path = os.path.join(proj_path, 'previous_settings.json')
import pathfinding
from pathfinding.testing.experiments import *

maps_dict = {
    1: 'small_blank_map',
    2: 'circular_map',
    3: 'warehouse_map',
    4: 'maze_map',
    5: 'obstacle_map',
    6: 'obstacle_bottle_neck_map'
}

distribution_dict = {1: 'max', 2: 'uni', 3: 'min'}


def is_valid(arg):
    if len(arg) < 1:
        print('Invalid input.')
        return False

    return True


def get_u():
    legal = False
    while not legal:
        legal = True
        tu = input("Enter desired uncertainty values: ")
        tu = tu.replace(' ', '').split(',')

        if not is_valid(tu):
            legal = False
            continue

        for val in tu:
            if len(val) > 1:
                print('Invalid uncertainty levels')
                legal = False
                break
            if int(val) < 0:
                print('Invalid uncertainty levels')
                legal = False
                break

        tu = [int(v) for v in tu] if legal else []
    return tu


def get_agents():
    legal = False
    while not legal:
        legal = True
        agent_num = input("Enter number of agents: ")
        agent_num = agent_num.replace(' ', '').split(',')
        if not is_valid(agent_num):
            legal = False
            continue

        for val in agent_num:
            if int(val) < 2:
                print('Invalid input.')
                legal = False
                break
        agent_num = [int(v) for v in agent_num] if legal else []

    return agent_num


def get_sense_prob():
    legal = False
    while not legal:
        legal = True
        sp = input("Enter probability for sensing from 0 to 100: ")
        sp = sp.replace(' ', '').split(',')
        sp = [int(v) for v in sp]
        for val in sp:
            if val > 100 or val < 0:
                print('Invalid value entered.')
                legal = False
                continue
    return sp


def get_edge_dist():
    legal = False

    while not legal:
        dist = input("Enter desired edge weight distributions. 1 - max, 2- uni, 3 - min: ")
        dist = dist.replace(' ', '').split(',')
        if len(dist) < 1 or len(dist) > 3:
            print('Invalid input.')
            continue
        dist = [int(v) for v in dist]
        for val in dist:
            if val < 1 or val > 3:
                print('Invalid input.')
                continue
        temp_dist = []
        for val in dist:
            temp_dist.append(distribution_dict[val])
        dist = temp_dist
        legal = True

    return dist


def get_comm_mode():
    legal = False
    while not legal:
        legal = True

        comm = input("Enter whether to use communication: ")
        comm = comm.lower().replace(' ', '').split(',')

        if len(comm) < 1 or len(comm) > 2:
            print('Invalid input.')
            legal = False
            continue

        for val in comm:
            if val != 'false' and val != 'true':
                print('Invalid value entered.')
                legal = False
                continue
        comm = [True if val == 'true' else False for val in comm]

    return comm


def get_mbc():
    legal = False
    while not legal:
        objective = input("Enter planner objectives (True - minimize best case, False - minimize worst case): ")
        objective = objective.lower().replace(' ', '').split(',')
        if len(objective) > 2 or len(objective) < 1:
            print('Invalid input.')
            continue
        for val in objective:
            if val != 'false' and val != 'true':
                print('Invalid value entered.')
                continue
        objective = [True if val == 'true' else False for val in objective]
        legal = True

    return objective


def get_pc():
    legal = False
    while not legal:
        use_pc = input("Enter if to use PC: ")
        use_pc = use_pc.lower().replace(' ', '').split(',')
        if len(use_pc) > 2 or len(use_pc) < 1:
            print('Invalid input.')
            continue
        for val in use_pc:
            if val != 'false' and val != 'true':
                print('Invalid value entered.')
                continue
        use_pc = [True if val == 'true' else False for val in use_pc]
        legal = True

    return use_pc


def get_bp():
    legal = False
    while not legal:
        use_bp = input("Enter if to use BP: ")
        use_bp = use_bp.lower().replace(' ', '').split(',')
        if len(use_bp) > 2 or len(use_bp) < 1:
            print('Invalid input.')
            continue
        for val in use_bp:
            if val != 'false' and val != 'true':
                print('Invalid value entered.')
                continue
        use_bp = [True if val == 'true' else False for val in use_bp]
        legal = True

    return use_bp


def get_maps():

    legal = False
    while not legal:
        domains = input("Enter desired domains: 1 - open 8x8 map, 2 - circular DAO map, 3 - warehouse map,"
                        " 4 - maze map, 5 - obstacle map, 6 - bottle neck map: ")
        domains = domains.replace(' ', '').split(',')
        if len(domains) < 1 or len(domains) > 6:
            print('Invalid input.')
            continue
        domains = [int(v) for v in domains]
        for val in domains:
            if val < 1 or val > 6:
                print('Invalid input.')
                continue
        temp_dist = []
        for val in domains:
            temp_dist.append(maps_dict[val])
        domains = temp_dist
        legal = True

    return domains


def get_reps():
    legal = False
    tu = 50
    while not legal:
        legal = True
        tu = input("Enter desired number of reps: ")

        if not is_valid(tu):
            legal = False
            continue

        tu = int(tu)

    return tu


def save_settings(settings_dict):
    with open(previous_settings_file_path, "w") as settings_file:
        json.dump(settings_dict, settings_file)


def get_previous_settings():
    with open(previous_settings_file_path, "r") as settings_file:
        d = json.load(settings_file)
    return d


def get_new_user_input_settings():
    u = get_u()
    agents = get_agents()
    sense_prob = get_sense_prob()
    edge_dist = get_edge_dist()
    comm_mode = get_comm_mode()
    mbc = get_mbc()
    pc = get_pc()
    bp = get_bp()
    maps = get_maps()
    reps = get_reps()

    settings_dict = {'u': u, 'agents': agents, 'sense_prob': sense_prob, 'edge_dist': edge_dist,
                     'comm_mode': comm_mode, 'mbc': mbc, 'pc': pc, 'bp': bp, 'maps': maps, 'reps': reps}
    return settings_dict


if __name__ == '__main__':
    print('Initiating CBSTU tests..')
    finished = False
    resume_previous_run = False
    d = {}
    while True:
        try:
            resume_previous_run = input('Would you like to use the settings from last run? [Y / N]')
            resume_previous_run = resume_previous_run.lower() == 'y'
            if resume_previous_run:
                d = get_previous_settings()
                # ToDo: Continue from where we left off as well
            else:
                d = get_new_user_input_settings()
            if 'reps' not in d:
                d['reps'] = get_reps()
            print('Your choices were:')
            print(f'Uncertainty: {d["u"]}\nNumber of agents: {d["agents"]}\nSensing Probability: {d["sense_prob"]}\n'
                  f'Edge Distribution: {d["edge_dist"]}\nWith communication: {d["comm_mode"]}\nMinimize best case:'
                  f' {d["mbc"]}\nUse PC: {d["pc"]}\nUse BP: {d["bp"]}\nMaps: {d["maps"]}\nReps: {d["reps"]}\n')
            done = input('Is this correct? [Y / N]: ')
            if done.lower() == 'y':
                break
        except ValueError:
            print('Unexpected error. Please enter input values again.')

    save_settings(d)
    run_experiments(u=d['u'], agents=d['agents'], sense_prob=d['sense_prob'], edge_dist=d['edge_dist'],
                    comm_mode=d['comm_mode'], mbc=d['mbc'], pc=d['pc'], bp=d['bp'], maps=d['maps'], reps=d['reps'])
