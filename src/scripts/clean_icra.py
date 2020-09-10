import os, sys

current_file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_file_path, '..'))

def clean_feedback():
	os.remove(os.path.join(current_file_path, '..', '..', 'domains', 'CDB_icra', 'feedback', 'cross.data'))
	os.remove(os.path.join(current_file_path, '..', '..', 'domains', 'CDB_icra', 'feedback', 'cross_full.data'))
	os.remove(os.path.join(current_file_path, '..', '..', 'domains', 'CDB_icra', 'feedback', 'open.data'))
	os.remove(os.path.join(current_file_path, '..', '..', 'domains', 'CDB_icra', 'feedback', 'open_full.data'))

def clean_params():
	os.remove(os.path.join(current_file_path, '..', '..', 'domains', 'CDB_icra', 'params', 'kappa.pkl'))
	os.remove(os.path.join(current_file_path, '..', '..', 'domains', 'CDB_icra', 'params', 'cross_gam.pkl'))
	os.remove(os.path.join(current_file_path, '..', '..', 'domains', 'CDB_icra', 'params', 'cross_gam_map.pkl'))
	os.remove(os.path.join(current_file_path, '..', '..', 'domains', 'CDB_icra', 'params', 'open_gam.pkl'))
	os.remove(os.path.join(current_file_path, '..', '..', 'domains', 'CDB_icra', 'params', 'open_gam_map.pkl'))

def main():
	clean_feedback()
	clean_params()

if __name__ == '__main__':
	main()