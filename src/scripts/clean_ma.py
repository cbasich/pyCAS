import os, sys

current_file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_file_path, '..'))

def clean_agents():
    os.remove(os.path.join(current_file_path, '..', '..', 'domains', 'CDB_ma', 'agents', 'none', '*'))
    os.remove(os.path.join(current_file_path, '..', '..', 'domains', 'CDB_ma', 'agents', 'naive', '*'))
    os.remove(os.path.join(current_file_path, '..', '..', 'domains', 'CDB_ma', 'agents', 'soft_labeling', '*'))
    os.remove(os.path.join(current_file_path, '..', '..', 'domains', 'CDB_ma', 'agents', 'multi_source', '*'))
    os.remove(os.path.join(current_file_path, '..', '..', 'domains', 'CDB_ma', 'agents', 'multi_task', '*'))

def clean_outputs():
    os.remove(os.path.join(current_file_path, '..', '..', 'output', 'CDB_ma', 'none', 'output.txt'))
    os.remove(os.path.join(current_file_path, '..', '..', 'output', 'CDB_ma', 'naive', 'output.txt'))
    os.remove(os.path.join(current_file_path, '..', '..', 'output', 'CDB_ma', 'soft_labeling', 'output.txt'))
    os.remove(os.path.join(current_file_path, '..', '..', 'output', 'CDB_ma', 'multi_source', 'output.txt'))
    os.remove(os.path.join(current_file_path, '..', '..', 'output', 'CDB_ma', 'multi_task', 'output.txt'))

def main():
    clean_agents()
    clean_outputs()

if __name__ == '__main__':
    main()
