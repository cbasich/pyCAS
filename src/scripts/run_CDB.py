import os, sys, time, json, random, copy, pickle

from copy import deepcopy
from collections import defaultdict
from IPython import embed

# from matplotlib import cm
from matplotlib import pyplot as plt
# from matplotlib.colors import ListedColormap, LinearSegmentedColormap

import numpy as np
import itertools as it

current_file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_file_path, '..'))

import process_data

from models.CDB.competence_aware_system import CAS
from models.CDB import autonomy_model, feedback_model, domain_model

OUTPUT_PATH = os.path.join(current_file_path, '..', '..', 'output', 'CDB')
FEEDBACK_DATA_PATH = os.path.join(current_file_path, '..', '..', 'domains', 'CDB', 'feedback')
PARAM_PATH = os.path.join(current_file_path, '..', '..', 'domains', 'CDB', 'params')
MAP_PATH = os.path.join(current_file_path, '..', '..', 'domains', 'CDB', 'maps')


def main(grid_file, N, generate):
    if not os.path.exists( os.path.join(FEEDBACK_DATA_PATH, 'cross.data') ):
        init_cross_data()
    if not os.path.exists( os.path.join(FEEDBACK_DATA_PATH, 'open.data') ):
        init_open_data()
    if not os.path.exists( os.path.join(FEEDBACK_DATA_PATH, 'cross_full.data') ):
        init_full_cross_data()
    if not os.path.exists( os.path.join(FEEDBACK_DATA_PATH, 'open_full.data') ):
        init_full_open_data()

    all_level_optimality = []
    try:
        with open(os.path.join(OUTPUT_PATH, grid_file[:-4] + '_alo.txt'), mode = 'r+') as all_level_optimality_file:
            all_level_optimality = [float(x) for x in all_level_optimality_file.readline().split(",")]
    except Exception:
        pass

    offices = ['a','b','d','e','f','g','h','i','j','k','l','m','n','o','p','q','r','s','t']

    for i in range(N):
        # start = offices[np.random.randint(len(offices))]
        # end = offices[np.random.randint(len(offices))]
        # while end == start:
        #     end = offices[np.random.randint(len(offices))]
        # with open(os.path.join(OUTPUT_PATH, 'tasks.txt'), mode='a+') as f:
        #     f.write(state + end + ',')

        start = 'b'
        end = 't'

        print("Building environment...")
        print("Building domain model...")
        DM = domain_model.DeliveryBotDomain(grid_file, start, end)
        print("Building autonomy model...")
        AM = autonomy_model.AutonomyModel(DM, [0, 1, 2, 3])
        print("Building feedback model...")
        HM = feedback_model.FeedbackModel(DM, AM, ['+', '-', '/', None], ['cross', 'open'])
        print("Building CAS...")
        environment = CAS(DM, AM, HM, persistence=0.75)

        solver = 'FVI'
        print("Solving mdp...")
        print(environment.solve(solver=solver))

        with open(os.path.join(OUTPUT_PATH, grid_file[:-4] + '_expected_costs.txt'), mode='a+') as expected_cost_file:
            expected_cost_file.write(str(environment.V[environment.states.index(environment.init)]) + "\n")

        alo_value = environment.check_level_optimality() * 100.0
        all_level_optimality.append(alo_value)

        with open(os.path.join(OUTPUT_PATH, grid_file[:-4] + '_alo.txt'), mode = 'a+') as all_level_optimality_file:
            all_level_optimality_file.write("," + str(alo_value))

        print("Beginning simulation...")
        if solver == 'FVI':
            costs = execute_policy(environment, 10, i)
        elif solver == 'LRTDP':
            cost = execute_LRTDP(environment)

        with open(os.path.join(OUTPUT_PATH, grid_file[:-4] + '_costs.txt'), mode = 'a+') as cost_file:
            for cost in costs:
                cost_file.write(str(cost) + ",")
            cost_file.write("\n")

        print("Updating parameters...")
        environment.update_kappa()
        environment.save_kappa()

        print("Identifying candidates...")
        candidates, init_state_candidate_count = environment.HM.find_candidates()
        if len(candidates) > 0:
            candidate = candidates[np.random.randint(len(candidates))]
            print(candidate)

            print("Identifying potential discriminators...")
            try:
                discriminator = environment.HM.get_most_likely_discriminator(candidate, 1)
            except Exception:
                discriminator = None
            if discriminator == None:
                print("No discriminator found")
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

        visited_level_optimality, feedback_count = process_results(environment)
        graph_results(all_level_optimality, visited_level_optimality, feedback_count)


def execute_LRTDP(CAS):
    cost = 0.0
    s = CAS.states.index(CAS.init)

    while not CAS.states[s] in CAS.goals:
        a, _ = CAS.solver.solve(s)
        print(CAS.states[s], "  ,  ", CAS.actions[a])
        cost += CAS.costs[s][a]

        feedback = None
        state = CAS.states[s]
        action = CAS.actions[a]
        if action[1] in [1,2]:
            feedback = interfaceWithHuman(state[0], action, map_info[str((state[0][0], state[0][1]))])
            updateData(action[0], action[1], CAS.DM.get_region(state[0]), state[0][3], feedback)
            if feedback == '-' or feedback == '/':
                CAS.remove_transition(state, action)

        s = CAS.solver.generate_successor(s, a)

    return cost


def execute_policy(CAS, M, i):
    pi_base = CAS.pi
    transitions_base = CAS.transitions
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
            # print(state, "  ,  ", action)

            r += CAS.costs[CAS.state_map[state]][CAS.actions.index(action)]

            feedback = None
            if action[1] == 1 or action[1] == 2:
                feedback = interfaceWithHuman(state[0], action, map_info[str((state[0][0], state[0][1]))])
                if j == M-1:
                    f1 = [action[1], CAS.DM.helper.get_state_feature_value(state[0], 'region'), state[0][3]]
                    if f1[2] == 'door-closed':
                        f1[2] = 'door'
                    if (('doortype' in used_features and 'door' in state[0][3])
                     or ('visibility' in used_features and state[0][3] in ['empty', 'light', 'busy'])):
                        f1.append(state[0][4])

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
        CAS.transitions = transitions_base

    execution_trace_file.close()

    return total_returns

def interfaceWithHuman(state, action, info, automate=True):
    feedback = None

    if not automate:
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
                if info['doorsize'] == 'small' or (info['doorsize'] == 'medium' and info['region'] == 'b1'):
                    feedback = 'yes'
                else:
                    feedback = 'no'

        elif info['obstacle'] == 'crosswalk':
            if state[3] == 'empty' or (info['visibility'] == 'high' and state[3] == 'light'):
                feedback = 'yes'
            else:
                feedback = 'no'

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
        data_string_copy = "2" + data_string[1:]
        full_data_string_copy = "2" + full_data_string[1:]

    if action == 'cross':
        filepath = os.path.join(FEEDBACK_DATA_PATH, 'cross.data')
        with open(filepath, mode='a+') as f:
            f.write("\n" + data_string + "," + str(feedback))
            if flagged:
                f.write("\n" + data_string_copy + "," + str(feedback))

        filepath = os.path.join(FEEDBACK_DATA_PATH, 'cross_full.data')
        with open(filepath, mode='a+') as f:
            f.write("\n" + full_data_string + "," + str(feedback))
            if flagged:
                f.write("\n" + full_data_string_copy + "," + str(feedback))
    elif action == 'open':
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

def init_cross_data():
    with open( os.path.join(FEEDBACK_DATA_PATH, 'cross.data'), 'a+') as f:
        f.write('level,region,obstacle,feedback')
        for level in ['1','2']:
            for region in ['r1','r2']:
                for obstacle in ['empty', 'light', 'heavy']:
                    for feedback in ['yes','no']:
                        entry = level + "," + region + "," + obstacle + "," + feedback
                        f.write("\n" + entry)

def init_full_cross_data():
    with open( os.path.join(FEEDBACK_DATA_PATH, 'cross_full.data'), 'a+') as f:
        f.write('level,region,obstacle,visibility,streettype,feedback')

def init_open_data():
    with open( os.path.join(FEEDBACK_DATA_PATH, 'open.data'), 'a+') as f:
        f.write('level,region,obstacle,feedback')
        for level in ['1','2']:
            for region in ['b1','b2','b3']:
                for obstacle in ['door']:
                    for feedback in ['yes','no']:
                        entry = level + "," + region + "," + obstacle + "," + feedback
                        f.write("\n" + entry)

def init_full_open_data():
    with open( os.path.join(FEEDBACK_DATA_PATH, 'open_full.data'), 'a+') as f:
        f.write('level,region,obstacle,doortype,doorcolor,feedback')

def process_results(CAS):
    policies = pickle.load( open(os.path.join(OUTPUT_PATH, 'policies.pkl'), mode='rb'), encoding='bytes')
    execution_trace_file = os.path.join(OUTPUT_PATH, 'execution_trace.txt')

    # all_states = CAS.states
    visited_states = process_data.get_visited_states(execution_trace_file)
    # reachable_states = process_data.get_reachable_states(policies, CAS)

    all_level_optimality, visited_level_optimality, reachable_level_optimality = [], [], []

    for i in range(len(policies.keys())):
        pi = policies[i]['policy']
        state_map = policies[i]['state_map']

        all_avg, visited_avg, reachable_avg = [], [], []

        # for state in all_states:
        #     if len(state[0]) < 3 or 'open' in state[0][3]:
        #         continue
        #     try:
        #         action = pi[state_map[state]]
        #     except Exception:
        #         # print("Error check 1")
        #         # embed()
        #         tmp = (state[0][:-1], state[1])
        #         action = pi[state_map[tmp]]
        #     all_avg.append(int(CAS.DM.helper.level_optimal(state,action)))

        for state in visited_states:
            if len(state[0]) < 3 or 'open' in state[0][3]:
                continue
            try:
                action = pi[state_map[state]]
                visited_avg.append(int(CAS.DM.helper.level_optimal(state,action)))
            except Exception:
                continue

        # for state in reachable_states[i]:
        #     if len(state[0]) < 3 or 'open' in state[0][3]:
        #         continue
        #     try:
        #         action = pi[state_map[state]]
        #     except Exception:
        #         # print("Error check 3")
        #         # embed()
        #         tmp = (state[0][:-1], state[1])
        #         action = pi[state_map[tmp]]
        #     reachable_avg.append(int(CAS.DM.helper.level_optimal(state,action)))

        # all_level_optimality.append(np.mean(all_avg) * 100.0)
        visited_level_optimality.append(np.mean(visited_avg) * 100.0)
        # reachable_level_optimality.append(np.mean(reachable_avg))

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

def graph_results(alo, vlo, fc):
    fig, ax1 = plt.subplots()
    ax1.set_xlabel('Episode', fontsize=14)
    ax1.set_ylabel('% Policy at Competence', fontsize=14)
    ax1.set_ylim(top=100)

    ax1.plot(alo, color = 'steelblue', label = 'All States')
    ax1.plot(vlo, color = 'darkkhaki', label = 'Visited States')
    # ax1.plot(rlo, color = 'indianred', label = 'Reachable States')
    ax1.tick_params(axis='y')

    ax2 = ax1.twinx()
    ax2.set_ylabel('Number of Feedback Signals', fontsize=14)
    ax2.plot(fc, color = 'black', label = 'Cumulative Signals')
    ax2.tick_params(axis = 'y')

    fig.tight_layout()
    plt.grid(False)

    lines, labels = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax2.legend(lines + lines2, labels + labels2, loc=4, fontsize=14)

    filepath = os.path.join(OUTPUT_PATH, 'competence_graph.png')
    plt.savefig(filepath)
    plt.clf()

    plt.close(fig)

if __name__ == '__main__':
    # grid_file = sys.argv[1]
    # N = int(sys.argv[2])
    # generate = int(sys.argv[3])
    # main(grid_file, N, generate)
    main('map_1.txt', 200, 0)
