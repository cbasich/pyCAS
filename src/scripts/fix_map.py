import os,sys,json
import numpy as np

def main():
	with open(os.path.join('..', '..', 'domains', 'CDB_icra', 'maps', 'map_info.json'), 'r+') as f:
		data = json.load(f)
		new_map_info = {}
		for k, v in data.items():
			if 'doorheight' in v.keys():
				new_map_info[k] = {k2: v2 for k2,v2 in v.items() if k2 not in ['doorheight', 'doorwidth']}
				new_map_info[k]['doorheight'] = int(round(v['doorheight']))
				new_map_info[k]['doorwidth'] = int(round(v['doorwidth']))
			# if 'doorsize' in v.keys():
			# 	new_map_info[k] = {k2: v2 for k2,v2 in v.items() if k2 != 'doorsize'}
			# 	if v['doorsize'] == 'small':
			# 		size = np.random.uniform(13,17)
			# 		height = np.random.uniform(4,6)
			# 		width = size / height
			# 	elif v['doorsize'] == 'medium':
			# 		size = np.random.uniform(17,22)
			# 		height = np.random.uniform(6,8)
			# 		width = size / height
			# 	elif v['doorsize'] == 'large':
			# 		size = np.random.uniform(22,28)
			# 		height = np.random.uniform(6,8)
			# 		width = size / height
			# 	new_map_info[k]['height'] = np.round(height, 3)
			# 	new_map_info[k]['width'] = np.round(width, 3)
			else:
				new_map_info[k] = v
		with open(os.path.join('..', '..', 'domains', 'CDB_icra', 'maps', 'new_map_info.json'), 'w+') as g:
			json.dump(new_map_info, g, indent = 4)

if __name__ == '__main__':
	main()