import os, sys

current_file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_file_path, '..'))

def clean_feedback():
    if os.path.exists(os.path.join(current_file_path, '..', '..', 'domains', 'CDB', 'feedback', 'cross.data')):
        os.remove(os.path.join(current_file_path, '..', '..', 'domains', 'CDB', 'feedback', 'cross.data'))
    if os.path.exists(os.path.join(current_file_path, '..', '..', 'domains', 'CDB', 'feedback', 'cross_full.data')):
        os.remove(os.path.join(current_file_path, '..', '..', 'domains', 'CDB', 'feedback', 'cross_full.data'))
    if os.path.exists(os.path.join(current_file_path, '..', '..', 'domains', 'CDB', 'feedback', 'open.data')):
        os.remove(os.path.join(current_file_path, '..', '..', 'domains', 'CDB', 'feedback', 'open.data'))
    if os.path.exists(os.path.join(current_file_path, '..', '..', 'domains', 'CDB', 'feedback', 'open_full.data')):
        os.remove(os.path.join(current_file_path, '..', '..', 'domains', 'CDB', 'feedback', 'open_full.data'))

def clean_params():
    if os.path.exists(os.path.join(current_file_path, '..', '..', 'domains', 'CDB', 'params', 'kappa.pkl')):
        os.remove(os.path.join(current_file_path, '..', '..', 'domains', 'CDB', 'params', 'kappa.pkl'))
    if os.path.exists(os.path.join(current_file_path, '..', '..', 'domains', 'CDB', 'params', 'cross_gam.pkl')):
        os.remove(os.path.join(current_file_path, '..', '..', 'domains', 'CDB', 'params', 'cross_gam.pkl'))
    if os.path.exists(os.path.join(current_file_path, '..', '..', 'domains', 'CDB', 'params', 'cross_gam_map.pkl')):
        os.remove(os.path.join(current_file_path, '..', '..', 'domains', 'CDB', 'params', 'cross_gam_map.pkl'))
    if os.path.exists(os.path.join(current_file_path, '..', '..', 'domains', 'CDB', 'params', 'open_gam.pkl')):
        os.remove(os.path.join(current_file_path, '..', '..', 'domains', 'CDB', 'params', 'open_gam.pkl'))
    if os.path.exists(os.path.join(current_file_path, '..', '..', 'domains', 'CDB', 'params', 'open_gam_map.pkl')):
        os.remove(os.path.join(current_file_path, '..', '..', 'domains', 'CDB', 'params', 'open_gam_map.pkl'))
    with open(os.path.join(current_file_path, '..', '..', 'domains', 'CDB', 'params', 'used_features.txt'), 'w') as f:
        f.write('level,region,obstacle')

def clean_outputs():
    if os.path.exists(os.path.join(current_file_path, '..', '..', 'output', 'CDB', 'candidate_count.txt')):
        os.remove(os.path.join(current_file_path, '..', '..', 'output', 'CDB', 'candidate_count.txt'))
    if os.path.exists(os.path.join(current_file_path, '..', '..', 'output', 'CDB', 'execution_trace.txt')):
        os.remove(os.path.join(current_file_path, '..', '..', 'output', 'CDB', 'execution_trace.txt'))
    if os.path.exists(os.path.join(current_file_path, '..', '..', 'output', 'CDB', 'init_state_candidate_count.txt')):
        os.remove(os.path.join(current_file_path, '..', '..', 'output', 'CDB', 'init_state_candidate_count.txt'))
    if os.path.exists(os.path.join(current_file_path, '..', '..', 'output', 'CDB', 'policies.pkl')):
        os.remove(os.path.join(current_file_path, '..', '..', 'output', 'CDB', 'policies.pkl'))

def main():
    clean_feedback()
    clean_params()
    clean_outputs()

if __name__ == '__main__':
    main()