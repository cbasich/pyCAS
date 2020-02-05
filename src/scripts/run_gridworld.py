import os, sys, time, random, copy, pickle

from collections import defaultdict
from IPython import embed

import numpy as np
import itertools as it

current_file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_file_path, '..'))
sys.path.append(os.path.join(current_file_path, '..', 'models'))

OUTPUT_PATH = os.path.join(current_file_path, '..', '..', 'out', 'gridworlds')
FEEDBACK_DATA_PATH = os.path.join(current_file_path, '..', '..', 'data', 'feedback', 'gridworlds')

from models import CAS, autonomy_model, feedback_model, CDB_domain_model


def main(grid_file, N, generate):
    cost_file = open(os.path.join(OUTPUT_PATH, grid_file[:-4] + '_costs.txt'), 'a+')
    expected_cost_file = open(os.path.join(OUTPUT_PATH, grid_file[:-4] + '_expected_costs.txt'), 'a+')

    offices = ['a','b','c','d','e','f','g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v','w','y','z']

    for i in range(N):
        start = offices[np.random.randint(len(offices))]
        end = offices[np.random.randint(len(offices))]
        while end == start:
            end = offices[np.random.randint(len(offices))]

        DM = CDB_domain_model.DeliveryBotDomain(grid_file, start, end)
        AM = autonomy_model.AM(DM, [0, 1, 2, 3])
        HM = feedback_model.HM(DM, AM, ['+', '-', '/', None], ['cross', 'open'])

        if not os.path.exists( os.path.join(FEEDBACK_DATA_PATH, 'cross') ):
            init_cross_data()
        if not os.path.exists( os.path.join(FEEDBACK_DATA_PATH, 'open') ):
            init_open_data()

        environment = CAS(DM, AM, HM, persistence=0.75)

        print("Solving mdp...")
        environment.solve()

        expected_cost_file.write(str(environment.V[environment.state_map[environment.init]]) + '\n')
        
        print("Beginning simulation...")
        cost = execute_policy(environment, 100, i)
        # cost_file.write(str(cost) + '\n')
        print(cost)

        print("Updating parameters...")
        environment.update_kappa()
        environment.save_kappa()
    
    expected_cost_file.close()
    cost_file.close()

    # if generate:
    #     generate_graph(os.path.join(OUTPUT_PATH, grid_file[:-4] + '_expected_costs.txt'),
                    # os.path.join(OUTPUT_PATH, grid_file[:-4] + '_costs.txt'))


def execute_policy(CAS, M, i):
    pi_base = CAS.pi
    total_returns = 0

    for j in range(M):
        pi = pi_base

        state = CAS.init
        r = 0

        while not state in CAS.goals:

            action = pi[CAS.state_map[state]]

            r += CAS.costs[CAS.state_map[state]][CAS.actions.index(action)]

            feedback = None
            if action[1] == 1 or action[1] == 2:
                feedback = interfaceWithHuman(state, action)
                if i == M-1:
                    updateData(action[0], action[1], CAS.DM.get_region(state), state[3], feedback)

            state = CAS.generate_successor(state, action, feedback)

        returns += r
        CAS.reset()

    return total_returns/M

def interfaceWithHuman(state, action):
    feedback = None
    if action[1] == 1:
        feedback = input('\nCan I take action --' + str(action[1]) + '-- in state ' + str(state) +'?\n\n')
    else:
        feedback = input('\nDo you override --' + str(action[1]) + '-- in state ' + str(state) +'?\n\n')
    return feedback

def updateData(action, level, region, obstacle, feedback):
    if feedback is None:
        pass
    if action == 'cross':
        filepath = sys.path.append(FEEDBACK_DATA_PATH, 'open.data')
        with open(filepath, mode='a+') as f:
            f.write('\n' + str(level) + ',' + str(region) + ',' + str(obstacle) + ',' + str(feedback))
    elif action == 'open':
        filepath = sys.path.append(FEEDBACK_DATA_PATH, 'open.data')
        with open(filepath, mode='a+') as f:
            f.write('\n' + str(level) + ',' + str(region) + ',' + str(obstacle) + ',' + str(feedback))

def init_cross_data():
    with open( os.path.join(FEEDBACK_DATA_PATH, 'cross.data'), 'a+') as f:
        f.write('level,region,obstacle,feedback')
        for level in ['1','2']:
            for region in ['r1','r2','r3']:
                for obstacle in ['empty', 'light', 'busy']:
                    for feedback in ['yes','no']:
                        entry = level + ',' + region + ',' + obstacle + ',' + feedback
                        f.write('\n' + entry)

def init_open_data():
    with open( os.path.join(FEEDBACK_DATA_PATH, 'open.data'), 'a+') as f:
        f.write('level,region,obstacle,feedback')
        for level in ['1','2']:
            for region in ['b1','b2','b3']:
                for obstacle in ['light-closed', 'medium-closed', 'heavy-closed']:
                    for feedback in ['yes','no']:
                        entry = level + ',' + region + ',' + obstacle + ',' + feedback
                        f.write('\n' + entry)

if __name__ == '__main__':
    grid_file = sys.argv[1]
    N = int(sys.argv[2])
    generate = int(sys.argv[3])
    main(grid_file, N, generate)