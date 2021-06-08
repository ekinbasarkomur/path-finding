import pygame
import math
from queue import PriorityQueue
import numpy as np
import time
from AVG import Qlearning, Astar

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
	time.sleep(0.05)



def get_clicked_pos(pos, rows, width):
	gap = width // rows
	y, x = pos

	row = y // gap
	col = x // gap

	return row, col


def main(win, width):
	ROWS = 20
	grid = make_grid(ROWS, width, HEIGHT)

	select_barrier_spot = grid[12][20]
	select_astar_spot = grid[5][20]
	select_qlearning_spot = grid[10][20]

	a_robot  = Astar(grid)

	select_barrier_spot.make_barrier()
	select_astar_spot.make_path()
	select_qlearning_spot.make_closed()

	start = None
	end = None

	run = True

	barrier_select = False
	astar_select = False
	qlearning_select = False
	bariers = [(6, 14), (7, 14), (8, 14), (9, 14),
					(10, 14), (11, 14), (12, 14), (13, 14),
					(6, 4), (6, 5), (6, 6), (6, 7), (6, 8),
					(7, 8), (7, 9), (7, 10), (7, 11), (6, 9),
					(6, 10), (6, 11), (11, 4), (12, 4), (13, 4),
					(14, 4), (15, 4), (16, 4), (17, 4), (14, 7),
					(14, 8), (14, 9), (14, 10), (15, 11), (15, 12),
					(14, 11), (14, 12), (15, 7), (16, 7), (17, 7),
					(11, 7), (11, 8), (11, 9), (11, 10), (11, 11),
					(11, 12), (10, 12), (6, 12), (14, 2), (14, 3),
					(13, 2), (12, 2), (12, 1), (11, 1), (2, 1), (2, 2),
					(2, 3), (2, 4), (1, 4), (1, 5), (1, 6), (2, 13),
					(2, 14), (2, 15), (2, 16), (3, 16), (3, 17), (9, 17),
					(10, 17), (11, 17), (12, 17), (13, 17), (14, 17), (15, 17),
					(16, 17), (1, 9), (2, 9), (3, 9), (7, 0), (7, 1), (17, 0),
					(17, 1), (6, 19), (6, 18)]

	for (row, col) in bariers:
		spot = grid[row][col]
		spot.make_barrier()

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
						qlearning = Qlearning(grid)
						get_shortest_path = qlearning.ql(end)
						shortest_path = get_shortest_path(start.row, start.col)
						print(shortest_path)
						draw_ql_path(lambda: draw(win, grid, ROWS, width), grid, shortest_path, win)
					elif astar_select:
						a_robot.algorithm(lambda: draw(win, grid, ROWS, width), start, end)



				if event.key == pygame.K_c:
					start = None
					end = None
					grid = make_grid(ROWS, width)

		time.sleep(0.01)
		a_robot.update()

	pygame.quit()

def draw_ql_path(draw, grid, shortest_path, win):
	last = None
	for i in range(len(shortest_path)):
		if i == 0:
			continue
		point = shortest_path[i]
		if last and i > 1:
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


main(WIN, WIDTH)
