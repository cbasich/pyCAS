import os, sys, time, json, random, copy, pickle

from copy import deepcopy
from collections import defaultdict
from IPython import embed

import numpy as np
import itertools as it

current_file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_file_path, '..'))

OUTPUT_PATH = os.path.join(current_file_path, '..', '..', 'output', 'CDB')
FEEDBACK_DATA_PATH = os.path.join(current_file_path, '..', '..', 'domains', 'CDB', 'feedback')
PARAM_PATH = os.path.join(current_file_path, '..', '..', 'domains', 'CDB', 'params')
MAP_PATH = os.path.join(current_file_path, '..', '..', 'domains', 'CDB', 'maps')

from models.main.competence_aware_system import CAS
from models.CDB import CDB_autonomy_model, CDB_feedback_model, CDB_domain_model


def main(grid_file, N, generate):
    if not os.path.exists( os.path.join(FEEDBACK_DATA_PATH, 'cross.data') ):
        init_cross_data()
    if not os.path.exists( os.path.join(FEEDBACK_DATA_PATH, 'open.data') ):
        init_open_data()
    if not os.path.exists( os.path.join(FEEDBACK_DATA_PATH, 'cross_full.data') ):
        init_full_cross_data()
    if not os.path.exists( os.path.join(FEEDBACK_DATA_PATH, 'open_full.data') ):
        init_full_open_data()

    cost_file = open(os.path.join(OUTPUT_PATH, grid_file[:-4] + '_costs.txt'), 'a+')
    expected_cost_file = open(os.path.join(OUTPUT_PATH, grid_file[:-4] + '_expected_costs.txt'), 'a+')

    offices = ['a','b','d','e','f','g','h','i','j','k','l','m','n','o','p','q','r','s','t'] #,'u','v','w','y','z']

    for i in range(N):
        start = offices[np.random.randint(len(offices))]
        end = offices[np.random.randint(len(offices))]
        while end == start:
            end = offices[np.random.randint(len(offices))]

        # start = 'b'
        # end = 'h'

        print("Building environment...")
        print("Building domain model...")
        DM = CDB_domain_model.DeliveryBotDomain(grid_file, start, end)
        print("Building autonomy model...")
        AM = CDB_autonomy_model.AutonomyModel(DM, [0, 1, 2, 3])
        print("Building feedback model...")
        HM = CDB_feedback_model.FeedbackModel(DM, AM, ['+', '-', '/', None], ['cross', 'open'])
        print("Building CAS...")
        environment = CAS(DM, AM, HM, persistence=0.75)

        print("Solving mdp...")
        solver = 'FVI'
        print(environment.solve(solver=solver))
        if solver == 'FVI':
            expected_cost_file.write(str(environment.V[environment.states.index(environment.init)]) + '\n')

        print("Beginning simulation...")
        if solver == 'FVI':
            cost = execute_policy(environment, 1, i)
        elif solver == 'LRTDP':
            cost = execute_LRTDP(environment)
        print(cost)
        cost_file.write(str(cost) + '\n')

        print("Updating parameters...")
        environment.update_kappa()
        environment.save_kappa()

        print("Identifying candidates...")
        candidates = environment.HM.find_candidates()
        if len(candidates) > 0:
            candidate = candidates[np.random.randint(len(candidates))]

            print("Identifying potential discriminators...")
            discriminator = environment.HM.get_most_likely_discriminator(candidate, 1)
            if discriminator == None:
                print("No discriminator found")
            else:
                environment.DM.helper.add_feature(discriminator, candidate)
                print(discriminator)
        else:
            print("No candidates...")

    expected_cost_file.close()
    cost_file.close()

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
    total_returns = 0

    execution_trace_file = open(os.path.join(OUTPUT_PATH, "execution_trace.txt"), mode = 'a+')

    with open(os.path.join(MAP_PATH, 'map_info.json')) as F:
        map_info = json.load(F)

    used_features = open(os.path.join(PARAM_PATH, 'used_features.txt')).readline().split(',')
    full_features = open(os.path.join(PARAM_PATH, 'full_features.txt')).readline().split(',')
    unused_features = [feature for feature in full_features if feature not in used_features]

    execution_trace_file.write("BEGINNING EPISODE\n")
    for j in range(M):
        execution_trace_file.write("BEGINNING TRIAL\n")
        pi = pi_base

        state = CAS.init
        r = 0

        while not state in CAS.goals:

            action = pi[CAS.state_map[state]]
            execution_trace_file.write(str(state) + " | " + str(action) + "\n")
            print(state, "  ,  ", action)

            r += CAS.costs[CAS.state_map[state]][CAS.actions.index(action)]

            feedback = None
            if action[1] == 1 or action[1] == 2:
                feedback = interfaceWithHuman(state[0], action, map_info[str((state[0][0], state[0][1]))])
                if i == M-1:
                    f1 = [action[1], CAS.DM.helper.get_state_feature_value(state[0], 'region'), state[0][3]]
                    if f1[2] == 'door-closed':
                        f1[2] = 'door'
                    if (('doortype' in used_features and 'door' in state[0][3])
                     or ('visibility' in used_features and state[0][3] in ['empty', 'light', 'heavy'])):
                        f1.append(state[0][4])

                    f2 = [CAS.DM.helper.get_state_feature_value(state[0], f) for f in unused_features 
                          if CAS.DM.helper.get_state_feature_value(state[0], f) != None]

                    updateData(action[0], f1, f2, feedback)
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

        total_returns += r
        CAS.transitions = transitions_base

    execution_trace_file.close()

    return total_returns/M

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
            if info['doortype'] == 'light' or (info['doortype'] == 'medium' and info['region'] == 'b1'):
                feedback = 'yes'
            else:
                feedback = 'no'

        elif info['obstacle'] == 'crosswalk':
            if state[3] == 'light' or (info['visibility'] == 'high' and state[3] == 'light'):
                feedback = 'yes'
            else:
                feedback = 'no'

        if np.random.uniform() <= 0.05:
            feedback = ['yes', 'no'][np.random.randint(2)]

    return feedback

def updateData(action, used_features, unused_features, feedback):
    if feedback is None:
        pass
    data_string = ",".join([str(f) for f in used_features])
    full_data_string = data_string + "," + ",".join([str(f) for f in unused_features])

    if action == 'cross':
        filepath = os.path.join(FEEDBACK_DATA_PATH, 'cross.data')
        with open(filepath, mode='a+') as f:
            f.write('\n' + data_string + ',' + str(feedback))
        if len(unused_features) > 0:
            filepath = os.path.join(FEEDBACK_DATA_PATH, 'cross_full.data')
            with open(filepath, mode='a+') as f:
                f.write('\n' + full_data_string + ',' + str(feedback))
    elif action == 'open':
        filepath = os.path.join(FEEDBACK_DATA_PATH, 'open.data')
        with open(filepath, mode='a+') as f:
            f.write('\n' + data_string + ',' + str(feedback))
        if len(unused_features) > 0:
            filepath = os.path.join(FEEDBACK_DATA_PATH, 'open_full.data')
            with open(filepath, mode='a+') as f:
                f.write('\n' + full_data_string + ',' + str(feedback))

def init_cross_data():
    with open( os.path.join(FEEDBACK_DATA_PATH, 'cross.data'), 'a+') as f:
        f.write('level,region,obstacle,feedback')
        for level in ['1','2']:
            for region in ['r1','r2','r3']:
                for obstacle in ['empty', 'light', 'busy']:
                    for feedback in ['yes','no']:
                        entry = level + ',' + region + ',' + obstacle + ',' + feedback
                        f.write('\n' + entry)

def init_full_cross_data():
    with open( os.path.join(FEEDBACK_DATA_PATH, 'cross_full.data'), 'a+') as f:
        f.write('level,region,obstacle,visibility,feedback')
        # for level in ['1','2']:
        #     for region in ['b1','b2','b3']:
        #         # for obstacle in ['light-closed', 'medium-closed', 'heavy-closed']:
        #         for obstacle in ['door-open', 'door-closed']:
        #             for feedback in ['yes','no']:
        #                 entry = level + ',' + region + ',' + obstacle + ',' + feedback
        #                 f.write('\n' + entry)

def init_open_data():
    with open( os.path.join(FEEDBACK_DATA_PATH, 'open.data'), 'a+') as f:
        f.write('level,region,obstacle,feedback')
        for level in ['1','2']:
            for region in ['b1','b2','b3']:
                # for obstacle in ['light-closed', 'medium-closed', 'heavy-closed']:
                for obstacle in ['door']:
                    for feedback in ['yes','no']:
                        entry = level + ',' + region + ',' + obstacle + ',' + feedback
                        f.write('\n' + entry)

def init_full_open_data():
    with open( os.path.join(FEEDBACK_DATA_PATH, 'open_full.data'), 'a+') as f:
        f.write('level,region,obstacle,doortype,feedback')
        # for level in ['1','2']:
        #     for region in ['b1','b2','b3']:
        #         # for obstacle in ['light-closed', 'medium-closed', 'heavy-closed']:
        #         for obstacle in ['door-open', 'door-closed']:
        #             for feedback in ['yes','no']:
        #                 entry = level + ',' + region + ',' + obstacle + ',' + feedback
        #                 f.write('\n' + entry)

if __name__ == '__main__':
    # grid_file = sys.argv[1]
    # N = int(sys.argv[2])
    # generate = int(sys.argv[3])
    # main(grid_file, N, generate)
    main('map_1.txt', 20, 0)