import pygame
import math
from queue import PriorityQueue
import numpy as np
import time
from AVG import Qlearning, Astar
from Spot import Spot

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
	qlearning = Qlearning(grid)

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
						get_shortest_path = qlearning.ql(end)
						shortest_path = get_shortest_path(start.row, start.col)
					elif astar_select:
						a_robot.algorithm(lambda: draw(win, grid, ROWS, width), start, end)

				if event.key == pygame.K_c:
					start = None
					end = None
					grid = make_grid(ROWS, width)

		time.sleep(0.01)
		qlearning.update()
		a_robot.update()

	pygame.quit()


main(WIN, WIDTH)
