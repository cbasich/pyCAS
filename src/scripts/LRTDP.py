import random
import numpy as np

class LRTDPSolver():
	def __init__(self, mdp, max_trials, epsilon = .001, dont_label = False):
		self.mdp = mdp
		self.max_trials = max_trials
		self.epsilon = epsilon
		self.dont_label = dont_label

		self.solved = []
		self.V = np.zeros(len(self.mdp.states))

	def solve(self, s):
		trials = 0
		while (not s in self.solved) and (trials < self.max_trials):
			self.trial(s)
			trials += 1
		return self.greedy_action(s)

	def trial(self, s):
		accum_cost = 0
		visited = []

		while not s in self.solved:
			if s in self.mdp.goals or accum_cost > 1000:
				break

			visited.append(s)
			self.update(s)

			a, _ = self.greedy_action(s)
			accum_cost += self.mdp.C(s, a)
			s = self.generate_successor(s, a)

		if self.dont_label:
			return

		while len(visited) > 0:
			s = visited.pop()
			if not self.check_solved(s):
				return

	def check_solved(self, s):
		rv = True

		open_ = []
		closed_ = []

		if not s in self.solved:
			open_.append(s)

		while len(open_) > 0:
			s = open_.pop()
			if s in self.mdp.goals:
				continue

			a, _ = self.greedy_action(s)
			closed_.append(s)

			if self.residual(s) > self.epsilon:
				rv = False

			for sp, p in enumerate(self.mdp.T(s, a)):
				if p == 0.0:
					continue
				elif sp not in self.solved and sp not in (open_ + closed_):
					open_.append(sp)

		if rv:
			for sp in closed_:
				self.solved.append(sp)

		else:
			while len(closed_) > 0:
				s = closed_.pop()
				self.update(s)

		return rv

	def q_value(self, s, a):
		return self.mdp.costs[s][a] + np.sum(self.mdp.T(s, a) * self.V)

	def greedy_action(self, s):
		q_values = [self.q_value(s, a) for a in range(len(self.mdp.actions))]
		a = np.argmax(q_values)
		return a, q_values[a]

	def update(self, s):
		_, q = self.greedy_action(s)
		self.V[s] = q

	def generate_successor(self, s, a):
		threshold = 0
		rand = np.random.uniform()
		for sp, p in enumerate(self.mdp.T(s,a)):
			threshold += p
			if rand <= threshold:
				return sp

	def residual(self, s):
		_, q = self.greedy_action(s)
		return abs(self.V[s] - q)