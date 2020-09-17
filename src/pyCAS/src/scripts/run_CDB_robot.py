import os, sys, time, json, random, copy, pickle, argparse

from copy import deepcopy
from collections import defaultdict
from IPython import embed
from matplotlib import pyplot as plt

import numpy as np
import itertools as it

current_file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_file_path, '..'))

import utils
import process_data

from models.CDB_robot.competence_aware_system import CAS
from models.CDB_robot import autonomy_model, feedback_model, domain_model

OUTPUT_PATH = os.path.join(current_file_path, '..', '..', 'output', 'CDB_robot')
FEEDBACK_DATA_PATH = os.path.join(current_file_path, '..', '..', 'domains', 'CDB_robot', 'feedback')
PARAM_PATH = os.path.join(current_file_path, '..', '..', 'domains', 'CDB_robot', 'params')
MAP_PATH = os.path.join(current_file_path, '..', '..', 'domains', 'CDB', 'maps')


def main(grid_file, N, update=False, interact=False, logging=False, verbose=True, start=None, end=None):
    if not os.path.exists( os.path.join(FEEDBACK_DATA_PATH, 'open.data') ):
        init_open_data()
    if not os.path.exists( os.path.join(FEEDBACK_DATA_PATH, 'open_full.data') ):
        init_full_open_data()

    all_level_optimality = []
    try:
        with open(os.path.join(OUTPUT_PATH, grid_file[:-4] + '_alo.txt'), mode = 'r+') as all_level_optimality_file:
            all_level_optimality = [float(x) for x in all_level_optimality_file.readline().split(",")]
    except Exception:
        pass

    offices = ['A', 'B']

    for i in range(N):
        end = np.random.choice(offices)
        end = (1, 3)

        print("Building environment for destination {}...".format(end))
        print("Building domain model...")
        DM = domain_model.DeliveryBotDomain(grid_file, start, end)
        print("Building autonomy model...")
        AM = autonomy_model.AutonomyModel(DM, [0, 1, 2, 3])
        print("Building feedback model...")
        HM = feedback_model.FeedbackModel(DM, AM, ['+', '-', '/', None], ['open'])
        print("Building CAS...")
        environment = CAS(DM, AM, HM, persistence=0.75)

        solver = 'FVI'
        print("Solving mdp...")
        print(environment.solve(solver=solver))

        # embed()

        if logging:
            with open(os.path.join(OUTPUT_PATH, grid_file[:-4] + '_expected_costs.txt'), mode='a+') as expected_cost_file:
                expected_cost_file.write(str(environment.V[environment.states.index(environment.init)]) + "\n")

            alo_value = environment.check_level_optimality() * 100.0
            all_level_optimality.append(alo_value)

            with open(os.path.join(OUTPUT_PATH, grid_file[:-4] + '_alo.txt'), mode = 'a+') as all_level_optimality_file:
                all_level_optimality_file.write("," + str(alo_value))

        print("Beginning simulation...")
        costs = execute_policy(environment, 1, i, interact, verbose=verbose)

        if logging:
            with open(os.path.join(OUTPUT_PATH, grid_file[:-4] + '_costs.txt'), mode = 'a+') as cost_file:
                for cost in costs:
                    cost_file.write(str(cost) + ",")
            cost_file.write("\n")

        print("Updating parameters...")
        environment.update_kappa()
        environment.save_kappa()

        if update:
            print("Identifying candidates...")
            candidates, init_state_candidate_count = environment.HM.find_candidates()
            if len(candidates) > 0:
                candidate = candidates[np.random.randint(len(candidates))]
                print(candidate)

                print("Identifying potential discriminators...")
                try:
                    discriminator = environment.HM.get_most_likely_discriminator(candidate)
                except Exception:
                    discriminator = None

                if discriminator == None:
                    print("No discriminator found...")
                else:
                    print("Found discriminator " + str(discriminator) + ".\n")
                    environment.DM.helper.add_feature(discriminator, candidate)
                    with open(os.path.join(OUTPUT_PATH, 'execution_trace.txt'), mode = 'a+') as f:
                        f.write("Discriminator added: " + str(discriminator) + "\n")
            else:
                print("No candidates...")

            with open(os.path.join(OUTPUT_PATH, 'candidate_count.txt'), mode = 'a+') as f:
                f.write(str(len(candidates)) + ",")

            with open(os.path.join(OUTPUT_PATH, 'init_state_candidate_count.txt'), mode = 'a+') as f:
                f.write(str(init_state_candidate_count) + ",")

    if logging:
        tmp_dic = {}
        visited_level_optimality, feedback_count = process_results(environment)
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

    with open(os.path.join(MAP_PATH, 'obstacle_info.json')) as F:
        map_info = json.load(F)

    used_features = open(os.path.join(PARAM_PATH, 'used_features.txt')).readline().split(",")
    full_features = open(os.path.join(PARAM_PATH, 'full_features.txt')).readline().split(",")
    unused_features = [feature for feature in full_features if feature not in used_features]

    execution_trace_file.write("BEGINNING EPISODE" + str(i) + "\n")
    for j in range(M):
        execution_trace_file.write("BEGINNING TRIAL\n")
        pi = pi_base

        state = CAS.init
        r = 0

        while not state in CAS.goals:

            action = pi[CAS.state_map[state]]
            execution_trace_file.write(str(state) + " | " + str(action) + "\n")
            if verbose:
                print(state, "  ,  ", action)

            r += CAS.costs[CAS.state_map[state]][CAS.actions.index(action)]

            feedback = None
            if action[1] == 1 or action[1] == 2:
                feedback = interfaceWithHuman(state[0], action, map_info[str((state[0][0], state[0][1]))], interact=interact)
                if j == M-1:
                    f1 = ['level'+str(action[1])] + [CAS.DM.helper.get_state_feature_value(state[0], f) 
                            for f in used_features if CAS.DM.helper.get_state_feature_value(state[0], f) != None]

                    f2 = [CAS.DM.helper.get_state_feature_value(state[0], f) for f in unused_features 
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


def interfaceWithHuman(state, action, info, interact=True):
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
                feedback = 'yes'

        if np.random.uniform() <= 0.05:
            feedback = ['yes', 'no'][np.random.randint(2)]

    return feedback


def updateData(action, used_features, unused_features, feedback, flagged):
    if feedback is None:
        print("Feedback is NONE")
        pass

    data_string = ",".join([str(f) for f in used_features])
    full_data_string = data_string 
    if len(unused_features) > 0:
        full_data_string = full_data_string + "," + ",".join([str(f) for f in unused_features])

    if flagged:
        data_string_copy = "level2" + data_string[6:]
        full_data_string_copy = "level2" + full_data_string[6:]

    if action == 'open':
        filepath = os.path.join(FEEDBACK_DATA_PATH, 'open.data')
        with open(filepath, mode='a+') as f:
            f.write("\n" + data_string + "," + str(feedback))
            if flagged:
                f.write("\n" + data_string_copy + "," + str(feedback))

        filepath = os.path.join(FEEDBACK_DATA_PATH, 'open_full.data')
        with open(filepath, mode='a+') as f:
            f.write("\n" + full_data_string + "," + str(feedback))
            if flagged:
                f.write("\n" + full_data_string_copy + "," + str(feedback))

def init_open_data():
    with open( os.path.join(FEEDBACK_DATA_PATH, 'open.data'), 'a+') as f:
        f.write('level,x,y,obstacle,feedback')
        for level in ['level1','level2']:
            for x,y in [(1,4), (3,2), (3,4), (5,4)]:
                for obstacle in ['door']:
                    for feedback in ['yes','no']:
                        entry = ",".join([level,str(x),str(y),obstacle,feedback])
                        f.write("\n" + entry)

def init_full_open_data():
    with open( os.path.join(FEEDBACK_DATA_PATH, 'open_full.data'), 'a+') as f:
        f.write('level,x,y,obstacle,doortype,doorcolor,feedback')


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
    parser.add_argument('-m', '--map_file', type=str, default='tiny_map.txt')
    parser.add_argument('-n', '--num_runs', type=int, default=1)
    parser.add_argument('-u', '--update', type=int, default=1)
    parser.add_argument('-i', '--interact', type=int, default=0)
    parser.add_argument('-l', '--logging', type=int, default=0)
    parser.add_argument('-v', '--verbose', type=int, default=1)
    parser.add_argument('-s', '--start', type=str, default='S')
    parser.add_argument('-e', '--end', type=str, default=None)
    args = parser.parse_args()

    main(args.map_file, args.num_runs, args.update, args.interact, 
         args.logging, args.verbose,args.start, args.end)