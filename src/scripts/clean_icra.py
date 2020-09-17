import os, sys

current_file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_file_path, '..'))

def clean_feedback():
    os.remove(os.path.join(current_file_path, '..', '..', 'domains', 'CDB_icra', 'feedback', 'cross.data'))
    os.remove(os.path.join(current_file_path, '..', '..', 'domains', 'CDB_icra', 'feedback', 'cross_full.data'))
    os.remove(os.path.join(current_file_path, '..', '..', 'domains', 'CDB_icra', 'feedback', 'open.data'))
    os.remove(os.path.join(current_file_path, '..', '..', 'domains', 'CDB_icra', 'feedback', 'open_full.data'))

def clean_params():
    os.remove(os.path.join(current_file_path, '..', '..', 'domains', 'CDB_icra', 'params', 'cross_gam.pkl'))
    os.remove(os.path.join(current_file_path, '..', '..', 'domains', 'CDB_icra', 'params', 'cross_gam_map.pkl'))
    os.remove(os.path.join(current_file_path, '..', '..', 'domains', 'CDB_icra', 'params', 'open_gam.pkl'))
    os.remove(os.path.join(current_file_path, '..', '..', 'domains', 'CDB_icra', 'params', 'open_gam_map.pkl'))
    if os.path.exists(os.path.join(current_file_path, '..', '..', 'domains', 'CDB_icra', 'params', 'kappa.pkl')):
        os.remove(os.path.join(current_file_path, '..', '..', 'domains', 'CDB_icra', 'params', 'kappa.pkl'))

def clean_outputs():
    os.remove(os.path.join(current_file_path, '..', '..', 'output', 'CDB_icra', 'candidate_count.txt'))
    os.remove(os.path.join(current_file_path, '..', '..', 'output', 'CDB_icra', 'execution_trace.txt'))
    os.remove(os.path.join(current_file_path, '..', '..', 'output', 'CDB_icra', 'init_state_candidate_count.txt'))
    os.remove(os.path.join(current_file_path, '..', '..', 'output', 'CDB_icra', 'policies.pkl'))

def clean_features():
    with open(os.path.join(current_file_path, '..', '..', 'domains', 'CDB_icra', 'params', 'used_features.txt'), 'w+') as f:
        f.write('level,obstacle')

def main():
    clean_feedback()
    clean_params()
    clean_features()
    # clean_outputs()

if __name__ == '__main__':
    main()