import pygame
import math
from queue import PriorityQueue
import numpy as np
import time

WIDTH = 800
HEIGHT = 900
WIN = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("A* Path Finding Algorithm")

RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 255, 0)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165 ,0)
GREY = (128, 128, 128)
TURQUOISE = (64, 224, 208)

COLOR_INACTIVE = (100, 80, 255)
COLOR_ACTIVE = (100, 200, 255)
COLOR_LIST_INACTIVE = (255, 100, 100)
COLOR_LIST_ACTIVE = (255, 150, 150)

class Spot:
	def __init__(self, row, col, width, total_rows):
		self.row = row
		self.col = col
		self.x = row * width
		self.y = col * width
		self.color = WHITE
		self.neighbors = []
		self.width = width
		self.total_rows = total_rows
		self.score = -1

	def get_pos(self):
		return self.row, self.col

	def is_closed(self):
		return self.color == RED

	def is_path(self):
		return self.color == PURPLE

	def is_open(self):
		return self.color == GREEN

	def is_barrier(self):
		return self.color == BLACK

	def is_start(self):
		return self.color == ORANGE

	def is_end(self):
		return self.color == TURQUOISE

	def reset(self):
		self.color = WHITE
		self.score = -1

	def make_start(self):
		self.color = ORANGE

	def make_closed(self):
		self.color = RED

	def make_open(self):
		self.color = GREEN

	def make_barrier(self):
		self.color = BLACK
		self.score = -100

	def make_end(self):
		self.color = TURQUOISE
		self.score = 100

	def make_path(self):
		self.color = PURPLE

	def draw(self, win):
		pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))

	def update_neighbors(self, grid):
		self.neighbors = []
		if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_barrier(): # DOWN
			self.neighbors.append(grid[self.row + 1][self.col])

		if self.row > 0 and not grid[self.row - 1][self.col].is_barrier(): # UP
			self.neighbors.append(grid[self.row - 1][self.col])

		if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrier(): # RIGHT
			self.neighbors.append(grid[self.row][self.col + 1])

		if self.col > 0 and not grid[self.row][self.col - 1].is_barrier(): # LEFT
			self.neighbors.append(grid[self.row][self.col - 1])

	def __lt__(self, other):
		return False


def h(p1, p2):
	x1, y1 = p1
	x2, y2 = p2
	return abs(x1 - x2) + abs(y1 - y2)


def reconstruct_path(came_from, current, draw):
	last = None
	while current in came_from:
		if last:
			last.reset()
		current = came_from[current]
		current.make_path()
		draw()
		last = current


def algorithm(draw, grid, start, end):
	count = 0
	open_set = PriorityQueue()
	open_set.put((0, count, start))
	came_from = {}
	g_score = {spot: float("inf") for row in grid for spot in row}
	g_score[start] = 0
	f_score = {spot: float("inf") for row in grid for spot in row}
	f_score[start] = h(start.get_pos(), end.get_pos())

	open_set_hash = {start}

	while not open_set.empty():
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				pygame.quit()

		current = open_set.get()[2]
		open_set_hash.remove(current)

		if current == end:
			reconstruct_path(came_from, end, draw)
			end.make_end()
			return True

		for neighbor in current.neighbors:
			temp_g_score = g_score[current] + 1

			if temp_g_score < g_score[neighbor]:
				came_from[neighbor] = current
				g_score[neighbor] = temp_g_score
				f_score[neighbor] = temp_g_score + h(neighbor.get_pos(), end.get_pos())
				if neighbor not in open_set_hash:
					count += 1
					open_set.put((f_score[neighbor], count, neighbor))
					open_set_hash.add(neighbor)
					# neighbor.make_open()

		draw()

		if current != start:
			pass
			#current.make_closed()

	return False


def make_grid(rows, width, height):
	grid = []
	gap = width // rows
	height_row = height // gap

	for i in range(rows):
		grid.append([])
		for j in range(height_row + 1):
			spot = Spot(i, j, gap, rows)
			grid[i].append(spot)

	return grid


def draw_grid(win, rows, width):
	gap = width // rows
	for i in range(rows):
		pygame.draw.line(win, GREY, (0, i * gap), (width, i * gap))
		for j in range(rows):
			pygame.draw.line(win, GREY, (j * gap, 0), (j * gap, width))


def draw(win, grid, rows, width):
	win.fill(WHITE)

	for row in grid:
		for spot in row:
			spot.draw(win)

	draw_grid(win, rows, width)
	pygame.display.update()


def get_clicked_pos(pos, rows, width):
	gap = width // rows
	y, x = pos

	row = y // gap
	col = x // gap

	return row, col


def main(win, width):
	ROWS = 50
	grid = make_grid(ROWS, width, HEIGHT)

	select_barrier_spot = grid[30][51]
	select_astar_spot = grid[5][51]
	select_qlearning_spot = grid[5][53]

	select_barrier_spot.make_barrier()
	select_astar_spot.make_path()
	select_qlearning_spot.make_closed()

	start = None
	end = None

	run = True

	barrier_select = False
	astar_select = False
	qlearning_select = False

	while run:
		draw(win, grid, ROWS, width)
		event_list = pygame.event.get()


		for event in event_list:
			if event.type == pygame.QUIT:
				run = False

			if pygame.mouse.get_pressed()[0]: # LEFT
				pos = pygame.mouse.get_pos()
				if pos[1] > WIDTH:
					row, col = get_clicked_pos(pos, ROWS, width)
					spot = grid[row][col]
					if spot.is_barrier():
						barrier_select = True
					else:
						astar_select = spot.is_path()
						qlearning_select = spot.is_closed()
						barrier_select = spot.is_barrier()

				else:
					row, col = get_clicked_pos(pos, ROWS, width)
					spot = grid[row][col]

					if barrier_select:
						spot.make_barrier()
					else:
						if not end and spot != start:
							end = spot
							end.make_end()
						elif not start and spot != end:
							start = spot
							start.make_start()


			elif pygame.mouse.get_pressed()[2]: # RIGHT
				pos = pygame.mouse.get_pos()
				row, col = get_clicked_pos(pos, ROWS, width)
				spot = grid[row][col]
				spot.reset()
				if spot == start:
					start = None
				elif spot == end:
					end = None

			if event.type == pygame.KEYDOWN:
				if event.key == pygame.K_SPACE and start and end:
					for row in grid:
						for spot in row:
							spot.update_neighbors(grid)
					print("qlearning_select %s" % qlearning_select)
					print("astar_select %s" % astar_select)
					if qlearning_select:
						shortest_path = ql(grid, start)
						print("shortest_path")
						print(shortest_path)
						draw_ql_path(lambda: draw(win, grid, ROWS, width), grid, shortest_path, win)
					elif astar_select:	
						algorithm(lambda: draw(win, grid, ROWS, width), grid, start, end)
						

				if event.key == pygame.K_c:
					start = None
					end = None
					grid = make_grid(ROWS, width)

	pygame.quit()

def draw_ql_path(draw, grid, shortest_path, win):
	last = None
	for point in shortest_path:
		if last:
			last.reset()
			spot.draw(win)
		row = point[0]
		col = point[1]
		spot = grid[row][col]
		spot.make_path()
		spot.draw(win)
		last = spot
		draw()

def get_poss_next_states(spot, F, ns):
	poss_next_states = []
	for i in range(spot.row, ns):																						
		for j in range(ns):
			if F[i][j].score > -100: 
				poss_next_states.append(F[spot.row][j])
	return poss_next_states

def get_rnd_next_state(s, F, ns):
	poss_next_states = get_poss_next_states(s, F, ns)
	next_state = poss_next_states[np.random.randint(0,len(poss_next_states))]
	return next_state

def train(F, Q, gamma, lrn_rate, max_epochs, start, end):
	for i in range(0,max_epochs):
		curr_s = start
		while(True):
			next_s = get_rnd_next_state(curr_s, F, len(F))
			poss_next_next_states = get_poss_next_states(next_s, F, len(F))
			max_Q = -9999.99
			for j in range(len(poss_next_next_states)):
				nn_s = poss_next_next_states[j]
				q = Q[next_s.row][nn_s.row]
				if q > max_Q:
					max_Q = q
			Q[curr_s.row][next_s.row] = ((1 - lrn_rate) * Q[curr_s.row] \
				[next_s.row]) + (lrn_rate * (F[curr_s.row][next_s.row].score + \
				(gamma * max_Q)))
			curr_s = next_s
			print("Row %d Col %d " % (curr_s.row, curr_s.col))
			if curr_s == end: break

def ql(grid, start):
	
	environment_rows = 50
	environment_columns = 50

	# Create a 3D numpy array to hold the current Q-values for each state and action pair: Q(s, a)
	# The array contains 11 rows and 11 columns (to match the shape of the environment), as well as a third "action" dimension.
	# The "action" dimension consists of 4 layers that will allow us to keep track of the Q-values for each possible action in
	# each state (see next cell for a description of possible actions).
	# The value of each (state, action) pair is initialized to 0.
	q_values = np.zeros((environment_rows, environment_columns, 4))

	# define actions
	# numeric action codes: 0 = up, 1 = right, 2 = down, 3 = left
	actions = ['up', 'right', 'down', 'left']

	# Create a 2D numpy array to hold the rewards for each state.
	# The array contains 11 rows and 11 columns (to match the shape of the environment), and each value is initialized to -100.
	rewards = np.full((environment_rows, environment_columns), -1.)
	for i in range(50):
		row = grid[i]
		for j in range(50):
			spot = row[j]
			if spot.score == -100 or spot.score == 100:
				rewards[spot.row, spot.col] = spot.score

	# define aisle locations (i.e., white squares) for rows 1 through 9
	aisles = {}  # store locations in a dictionary
	for i in range(50):
		row = grid[i]
		aisles[i] = []
		for j in range(50):
			spot = row[j]
			if spot.score == -1:
				aisles[i].append(spot.col)

	# set the rewards for all aisle locations (i.e., white squares)
	for row_index in range(1, 10):
		for column_index in aisles[row_index]:
			rewards[row_index, column_index] = -1.

	# print rewards matrix
	for row in rewards:
		print(row)

	
	def is_terminal_state(current_row_index, current_column_index):
		# if the reward for this location is -1, then it is not a terminal state (i.e., it is a 'white square')
		if rewards[current_row_index, current_column_index] == -1.:
			return False
		else:
			return True

	# define a function that will choose a random, non-terminal starting location


	def get_starting_location():
		# get a random row and column index
		current_row_index = np.random.randint(environment_rows)
		current_column_index = np.random.randint(environment_columns)
		# continue choosing random row and column indexes until a non-terminal state is identified
		# (i.e., until the chosen state is a 'white square').
		while is_terminal_state(current_row_index, current_column_index):
			current_row_index = np.random.randint(environment_rows)
			current_column_index = np.random.randint(environment_columns)
		return current_row_index, current_column_index

	# define an epsilon greedy algorithm that will choose which action to take next (i.e., where to move next)


	def get_next_action(current_row_index, current_column_index, epsilon):
		# if a randomly chosen value between 0 and 1 is less than epsilon,
		# then choose the most promising value from the Q-table for this state.
		if np.random.random() < epsilon:
			return np.argmax(q_values[current_row_index, current_column_index])
		else:  # choose a random action
			return np.random.randint(4)

	# define a function that will get the next location based on the chosen action


	def get_next_location(current_row_index, current_column_index, action_index):
		new_row_index = current_row_index
		new_column_index = current_column_index
		if actions[action_index] == 'up' and current_row_index > 0:
			new_row_index -= 1
		elif actions[action_index] == 'right' and current_column_index < environment_columns - 1:
			new_column_index += 1
		elif actions[action_index] == 'down' and current_row_index < environment_rows - 1:
			new_row_index += 1
		elif actions[action_index] == 'left' and current_column_index > 0:
			new_column_index -= 1
		return new_row_index, new_column_index

	# Define a function that will get the shortest path between any location within the warehouse that
	# the robot is allowed to travel and the item packaging location.


	def get_shortest_path(start_row_index, start_column_index):
		# return immediately if this is an invalid starting location
		if is_terminal_state(start_row_index, start_column_index):
			return []
		else:  # if this is a 'legal' starting location
			current_row_index, current_column_index = start_row_index, start_column_index
			shortest_path = []
			shortest_path.append([current_row_index, current_column_index])
			# continue moving along the path until we reach the goal (i.e., the item packaging location)
			while not is_terminal_state(current_row_index, current_column_index):
				# get the best action to take
				action_index = get_next_action(
					current_row_index, current_column_index, 1.)
				# move to the next location on the path, and add the new location to the list
				current_row_index, current_column_index = get_next_location(
					current_row_index, current_column_index, action_index)
				shortest_path.append([current_row_index, current_column_index])
			return shortest_path

	# define a function that determines if the specified location is a terminal state
	# define training parameters
	# the percentage of time when we should take the best action (instead of a random action)
	epsilon = 0.9
	discount_factor = 0.9  # discount factor for future rewards
	learning_rate = 0.9  # the rate at which the AI agent should learn

	# run through 1000 training episodes
	for episode in range(1000):
		# get the starting location for this episode
		row_index, column_index = get_starting_location()

		# continue taking actions (i.e., moving) until we reach a terminal state
		# (i.e., until we reach the item packaging area or crash into an item storage location)
		while not is_terminal_state(row_index, column_index):
			# choose which action to take (i.e., where to move next)
			action_index = get_next_action(row_index, column_index, epsilon)

			# perform the chosen action, and transition to the next state (i.e., move to the next location)
			# store the old row and column indexes
			old_row_index, old_column_index = row_index, column_index
			row_index, column_index = get_next_location(
				row_index, column_index, action_index)

			# receive the reward for moving to the new state, and calculate the temporal difference
			reward = rewards[row_index, column_index]
			old_q_value = q_values[old_row_index, old_column_index, action_index]
			temporal_difference = reward + \
				(discount_factor *
				np.max(q_values[row_index, column_index])) - old_q_value

			# update the Q-value for the previous state and action pair
			new_q_value = old_q_value + (learning_rate * temporal_difference)
			q_values[old_row_index, old_column_index, action_index] = new_q_value

	print('Training complete!')
	return get_shortest_path(start.row, start.col)


main(WIN, WIDTH)
