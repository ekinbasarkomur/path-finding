import pygame
import math
from queue import PriorityQueue
import numpy as np
import time

def qlearning_train():
    pass

class Qlearning:
    def __init__(self, grid):
        self.grid = grid
        self.path = []

class Astar:
    def __init__(self, grid):
        self.grid = grid
        self.path = []
        self.busy = False

    def update(self):
        if len(self.path) > 1:
            self.path[0].reset()
            self.path.pop(0)
            self.path[0].make_path()

    def get_current_spot(self):
        return self.path[0]

    def h(self, p1, p2):
        x1, y1 = p1
        x2, y2 = p2
        return abs(x1 - x2) + abs(y1 - y2)


    def reconstruct_path(self, came_from, current, draw):
        last = None
        while current in came_from:
            if last:
                last.reset()
            current = came_from[current]
            current.make_path()
            draw()
            last = current


    def algorithm(self, draw, start, end):
        count = 0
        open_set = PriorityQueue()
        open_set.put((0, count, start))
        came_from = {}
        g_score = {spot: float("inf") for row in self.grid for spot in row}
        g_score[start] = 0
        f_score = {spot: float("inf") for row in self.grid for spot in row}
        f_score[start] = self.h(start.get_pos(), end.get_pos())

        open_set_hash = {start}

        while not open_set.empty():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()

            current = open_set.get()[2]
            open_set_hash.remove(current)

            if current == end:
                for spot in came_from:
                    self.path.append(spot)
                return True

            for neighbor in current.neighbors:
                temp_g_score = g_score[current] + 1

                if temp_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = temp_g_score
                    f_score[neighbor] = temp_g_score + self.h(neighbor.get_pos(), end.get_pos())
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
