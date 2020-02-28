import time

import numpy as np

from numba import jit

def run_numpy(R, T, V):
	for i in range(10):
		R + np.sum( T * V, axis = 2)

if __name__ == '__main__':
	R = np.random.rand(1600, 28).astype('float32')
	T = np.random.rand(1600, 28, 1600).astype('float32')
	V = np.random.rand(1, 1, 1600).astype('float32')

	start = time.time()
	run_numpy(R,T,V)
	end = time.time()

	print("numpy time is " , (end-start)/10)