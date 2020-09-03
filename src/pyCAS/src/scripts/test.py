import time
import numpy as np


def main():
	arr = np.random.randn(3500, 28, 3500)
	average_time_1 = []
	for i in range(10):
		print(i)
		s_time = time.time()
		0.5 * (arr * arr + 0.9*arr)
		e_time = time.time()
		average_time_1.append((e_time - s_time))

	arr = np.random.randn(1750, 14, 1750)
	average_time_2 = []
	for i in range(10):
		print(i)
		s_time = time.time()
		0.5 * (arr * arr + 0.9*arr)
		e_time = time.time()
		average_time_2.append((e_time - s_time))

	print(np.mean(np.array(average_time_1)), "\n", np.mean(np.array(average_time_2)))	

if __name__ == '__main__':
	main()