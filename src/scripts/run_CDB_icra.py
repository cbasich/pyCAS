import os, sys, time, json, random, copy, pickle, argparse

from copy import deepcopy
from collections import defaultdict
from IPython import embed
from matplotlib import pyplot as plt

import numpy as np
import pandas as pd
import itertools as it

current_file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_file_path, '..'))

import utils
import process_data

from models.CDB_icra import autonomy_model, feedback_model, domain_model, competence_aware_system

OUTPUT_PATH = os.path.join(current_file_path, '..', '..', 'output', 'CDB_icra')
FEEDBACK_PATH = os.path.join(current_file_path, '..', '..', 'domains', 'CDB_icra', 'feedback')
PARAM_PATH = os.path.join(current_file_path, '..', '..', 'domains', 'CDB_icra', 'params')
MAP_PATH = os.path.join(current_file_path, '..', '..', 'domains', 'CDB_icra', 'maps')

global new_feedback

def main(grid_file, N, update=False, interact=False, logging=False, verbose=True, _start=None, _end=None):
    if not os.path.exists( os.path.join(FEEDBACK_PATH, 'cross.data') ):
        init_cross_data()
    if not os.path.exists( os.path.join(FEEDBACK_PATH, 'open.data') ):
        init_open_data()
    if not os.path.exists( os.path.join(FEEDBACK_PATH, 'cross_full.data') ):
        init_full_cross_data()
    if not os.path.exists( os.path.join(FEEDBACK_PATH, 'open_full.data') ):
        init_full_open_data()

    all_level_optimality = []
    try:
        with open(os.path.join(OUTPUT_PATH, grid_file[:-4] + '_alo.txt'), mode = 'r+') as all_level_optimality_file:
            all_level_optimality = [float(x) for x in all_level_optimality_file.readline().split(",")]
    except Exception:
        pass

    global new_feedback
    new_feedback = False

    non_offices = set(['.', '.\n', '\n', 'X', 'X\n', 'C', 'D'])
    offices = []

    with open(os.path.join(MAP_PATH, grid_file), 'r') as f:
        for line in f.readlines():
            chars = line.split(' ')
            offices += [ele for ele in chars if ele not in non_offices]

    if _start == None:
        start = np.random.choice(offices)
    if _end == None:
        end = np.random.choice(offices)
    while end == start:
        end = np.random.choice(offices)

    DM = domain_model.DeliveryBotDomain(grid_file, start)
    DM.set_goal(end)
    AM = autonomy_model.AutonomyModel(DM, [0,1,2,3])
    HM = feedback_model.FeedbackModel(DM, AM, ['+', '-', '/', None], ['cross', 'open'])
    CAS = competence_aware_system.CAS(DM, AM, HM, persistence = 0.9)

    # embed()
    # quit()

    for i in range(N):
        print("Starting episode {} with state {} and goal {}...\n".format(i, start, end))
        if new_feedback:
            DM.helper.build_gams()
            DM.compute_transitions()
            CAS.compute_transitions()
        DM.compute_costs()
        CAS.compute_costs()

        solver = 'FVI'
        print("Solving mdp...")
        print(CAS.solve(solver=solver))

        if logging:
            with open(os.path.join(OUTPUT_PATH, grid_file[:-4] + '_expected_costs.txt'), mode='a+') as expected_cost_file:
                expected_cost_file.write(str(CAS.V[CAS.states.index(CAS.init)]) + "\n")

            alo_value = CAS.check_level_optimality() * 100.0
            all_level_optimality.append(alo_value)

            with open(os.path.join(OUTPUT_PATH, grid_file[:-4] + '_alo.txt'), mode = 'a+') as all_level_optimality_file:
                all_level_optimality_file.write("," + str(alo_value))

        print("Beginning simulation...")
        costs = execute_policy(CAS, 1, i, interact, verbose=verbose)

        if logging:
            with open(os.path.join(OUTPUT_PATH, grid_file[:-4] + '_costs.txt'), mode = 'a+') as cost_file:
                for cost in costs:
                    cost_file.write(str(cost) + ",")
                cost_file.write("\n")

        print("Updating parameters...")
        CAS.update_kappa()
        CAS.save_kappa()

        if update:
            print("Identifying candidates...")
            candidates, init_state_candidate_count = CAS.HM.find_candidates()
            if len(candidates) > 0:
                candidate = candidates[np.random.randint(len(candidates))]
                print(candidate)
                # embed()
                print("Identifying potential discriminators...")
                # try:
                discriminator = CAS.HM.get_most_likely_discriminator(candidate, 3)
                # except Exception:
                #     discriminator = None

                if discriminator == None:
                    print("No discriminator found...")
                else:
                    print("Found discriminator " + str(discriminator) + ".\n")
                    CAS.DM.helper.add_feature(discriminator, candidate)
                    with open(os.path.join(OUTPUT_PATH, 'execution_trace.txt'), mode = 'a+') as f:
                        f.write("Discriminator added: " + str(discriminator) + "\n")
            else:
                print("No candidates...")

            with open(os.path.join(OUTPUT_PATH, 'candidate_count.txt'), mode = 'a+') as f:
                f.write(str(len(candidates)) + ",")

            with open(os.path.join(OUTPUT_PATH, 'init_state_candidate_count.txt'), mode = 'a+') as f:
                f.write(str(init_state_candidate_count) + ",")

        if CAS.check_level_optimality() == 1.0:
            break

        start = np.random.choice(offices)
        end = np.random.choice(offices)
        while start == end:
            end = np.random.choice(offices)

        DM.set_init(start)
        DM.set_goal(end)
        CAS.set_init()
        CAS.set_goals()

    if logging:
        tmp_dic = {}
        visited_level_optimality, feedback_count = process_results(CAS)
        tmp_dic["visited_LO"] = visited_level_optimality
        tmp_dic["feedback_count"] = feedback_count
        with open(os.path.join(OUTPUT_PATH, "competence_graph_info.pkl"), 'wb') as f:
            pickle.dump(tmp_dic, f, protocol=pickle.HIGHEST_PROTOCOL)


def execute_policy(CAS, M, i, interact, verbose=True):
    pi_base = CAS.pi
    transitions_base = CAS.transitions.copy()
    total_returns = []

    execution_trace_file = open(os.path.join(OUTPUT_PATH, 'execution_trace.txt'), mode = 'a+')

    policies = None
    try:
        policies = pickle.load( open(os.path.join(OUTPUT_PATH, 'policies.pkl'), mode='rb'), encoding='bytes')
        policies[max(policies.keys())+1] = {'policy': pi_base, 'state_map': CAS.state_map}
    except Exception:
        policies = {0: {'policy': pi_base, 'state_map': CAS.state_map}}
    pickle.dump(policies, open(os.path.join(OUTPUT_PATH, 'policies.pkl'), mode='wb'), protocol=pickle.HIGHEST_PROTOCOL)

    with open(os.path.join(MAP_PATH, 'map_info.json')) as F:
        map_info = json.load(F)

    full_features = open(os.path.join(PARAM_PATH, 'full_features.txt')).readline().split(",")

    used_open_features, used_cross_features = [], []
    with open( os.path.join(FEEDBACK_PATH, 'open.data'), mode='r+') as f:
        df = pd.read_csv(f)
        used_open_features = list(df.columns)
    with open( os.path.join(FEEDBACK_PATH, 'cross.data'), mode='r+') as f:
        df = pd.read_csv(f)
        used_cross_features = list(df.columns)

    execution_trace_file.write("BEGINNING EPISODE" + str(i) + "\n")
    for j in range(M):
        execution_trace_file.write("BEGINNING TRIAL\n")
        pi = pi_base

        state = CAS.init
        r = 0

        while not state in CAS.goals:
            if state == (None, 3):
                embed()
                quit()
            action = pi[CAS.state_map[state]]
            execution_trace_file.write(str(state) + " | " + str(action) + "\n")
            if verbose:
                print(state, "  ,  ", action)

            r += CAS.costs[CAS.state_map[state]][CAS.actions.index(action)]

            feedback = None
            if action[1] == 1 or action[1] == 2:
                feedback = interfaceWithHuman(state[0], action, map_info[str((state[0][0], state[0][1]))], CAS.DM.timeofday, interact=interact)
                if j == M-1:
                    if action[0] == 'open':
                        f1 = [action[1]] + [CAS.DM.helper.get_state_feature_value(state[0], f) 
                                for f in used_open_features if CAS.DM.helper.get_state_feature_value(state[0], f) != None]
                    elif action[0] == 'cross':
                        f1 = [action[1]] + [CAS.DM.helper.get_state_feature_value(state[0], f) 
                                for f in used_cross_features if CAS.DM.helper.get_state_feature_value(state[0], f) != None]

                    f2 = [action[1]] + [CAS.DM.helper.get_state_feature_value(state[0], f) for f in full_features 
                          if CAS.DM.helper.get_state_feature_value(state[0], f) != None]

                    flagged = (action[1] == 1 and CAS.flags[CAS.states.index(state)][CAS.DM.actions.index(action[0])] == False)

                    updateData(action[0], f1, f2, feedback, flagged)
                    if feedback is not None:
                        execution_trace_file.write("Feedback: " + feedback + "\n")
            if feedback == 'no':
                CAS.remove_transition(state, action)
                CAS.solve()
                pi = CAS.pi
                continue
            elif feedback == 'yes':
                state = (CAS.DM.generate_successor(state[0], action[0]), state[1])
            else:
                state = CAS.generate_successor(state, action)

        total_returns.append(r)
        CAS.set_transitions(transitions_base)

    execution_trace_file.close()

    return total_returns


def interfaceWithHuman(state, action, info, timeofday, interact=True):
    feedback = None

    if interact:
        if action[1] == 1:
            feedback = input('\nCan I take action --' + str(action[0]) + '-- in state ' + str(info) +'?\n\n')
        else:
            feedback = input('\nDo you override --' + str(action[0]) + '-- in state ' + str(info) +'?\n\n')
            if feedback == 'yes':
                feedback = 'no'
            elif feedback == 'no':
                feedback = 'yes'

    else:
        if info['obstacle'] == 'door':
            if info['doortype'] == 'pull':
                feedback = 'no'
            else:
                if ((info['doorsize'] == 'small') 
                 or (info['doorsize'] == 'medium' and info['region'] in ['b1','b2'])):
                    feedback = 'yes'
                else:
                    feedback = 'no'

        elif info['obstacle'] == 'crosswalk':
            if state[3] == 'empty' or (info['visibility'] == 'high' and state[3] == 'light'):
                if timeofday in info['pedestrians']:
                    feedback = 'no'
                else:
                    feedback = 'yes'
            else:
                feedback = 'no'

        if np.random.uniform() <= 0.05:
            feedback = ['yes', 'no'][np.random.randint(2)]

    return feedback


def updateData(action, used_features, full_features, feedback, flagged):
    if feedback is None:
        print("Feedback is NONE")
        pass

    global new_feedback
    new_feedback = True

    data_string = ",".join([str(f) for f in used_features])
    full_data_string = ",".join([str(f) for f in full_features])

    if flagged:
        data_string_copy = "2" + data_string[1:]
        full_data_string_copy = "2" + full_data_string[1:]

    if action == 'cross':
        filepath = os.path.join(FEEDBACK_PATH, 'cross.data')
        with open(filepath, mode='a+') as f:
            f.write("\n" + data_string + "," + str(feedback))
            if flagged:
                f.write("\n" + data_string_copy + "," + str(feedback))

        filepath = os.path.join(FEEDBACK_PATH, 'cross_full.data')
        with open(filepath, mode='a+') as f:
            f.write("\n" + full_data_string + "," + str(feedback))
            if flagged:
                f.write("\n" + full_data_string_copy + "," + str(feedback))
    elif action == 'open':
        filepath = os.path.join(FEEDBACK_PATH, 'open.data')
        with open(filepath, mode='a+') as f:
            f.write("\n" + data_string + "," + str(feedback))
            if flagged:
                f.write("\n" + data_string_copy + "," + str(feedback))

        filepath = os.path.join(FEEDBACK_PATH, 'open_full.data')
        with open(filepath, mode='a+') as f:
            f.write("\n" + full_data_string + "," + str(feedback))
            if flagged:
                f.write("\n" + full_data_string_copy + "," + str(feedback))


def init_cross_data():
    with open( os.path.join(FEEDBACK_PATH, 'cross.data'), 'a+') as f:
        f.write('level,region,traffic,feedback')
        for level in ['1','2']:
            for region in ['r1','r2']:
                for obstacle in ['empty', 'light', 'busy']:
                    for feedback in ['yes','no']:
                        entry = ",".join([level,region,obstacle,feedback])
                        f.write("\n" + entry)


def init_full_cross_data():
    with open( os.path.join(FEEDBACK_PATH, 'cross_full.data'), 'a+') as f:
        f.write('level,region,traffic,visibility,streettype,timeofday,feedback')


def init_open_data():
    with open( os.path.join(FEEDBACK_PATH, 'open.data'), 'a+') as f:
        f.write('level,region,feedback')
        for level in ['1','2']:
            for region in ['b1','b2','b3']:
                for feedback in ['yes','no']:
                    entry = ",".join([level,region,feedback])
                    f.write("\n" + entry)


def init_full_open_data():
    with open( os.path.join(FEEDBACK_PATH, 'open_full.data'), 'a+') as f:
        f.write('level,region,doorsize,doorcolor,doortype,timeofday,feedback')


def process_results(CAS):
    policies = pickle.load( open(os.path.join(OUTPUT_PATH, 'policies.pkl'), mode='rb'), encoding='bytes')
    execution_trace_file = os.path.join(OUTPUT_PATH, 'execution_trace.txt')

    visited_states = process_data.get_visited_states(execution_trace_file)

    all_level_optimality, visited_level_optimality = [], []

    for i in range(len(policies.keys())):
        pi = policies[i]['policy']
        state_map = policies[i]['state_map']

        all_avg, visited_avg = [], []

        for state in visited_states:
            if len(state[0]) < 3 or 'open' in state[0][3]:
                continue
            try:
                action = pi[state_map[state]]
                visited_avg.append(int(CAS.DM.helper.level_optimal(state,action)))
            except Exception:
                continue
        visited_level_optimality.append(np.mean(visited_avg) * 100.0)

    feedback_count = []
    f = open( os.path.join(OUTPUT_PATH, 'execution_trace.txt'), mode='r+')
    count = 0
    for line in f.readlines():
        if 'BEGINNING EPISODE' in line:
            feedback_count.append(count)
        if 'Feedback' in line:
            count += 1
    f.close()

    return visited_level_optimality, feedback_count


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-m', '--map_file', type=str, default='large_campus.txt')
    parser.add_argument('-n', '--num_runs', type=int, default=1)
    parser.add_argument('-u', '--update', type=int, default=1)
    parser.add_argument('-i', '--interact', type=int, default=0)
    parser.add_argument('-l', '--logging', type=int, default=0)
    parser.add_argument('-v', '--verbose', type=int, default=1)
    parser.add_argument('-s', '--start', type=str, default=None)
    parser.add_argument('-e', '--end', type=str, default=None)
    args = parser.parse_args()

    main(args.map_file, args.num_runs, args.update, args.interact, 
         args.logging, args.verbose,args.start, args.end)