import pygame
import math
from queue import PriorityQueue

WIDTH = 1000
WIN = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("A* Path Finding Algorithm")

RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 255, 0)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165, 0)
GREY = (128, 128, 128)
TURQUOISE = (64, 224, 208)


class Spot:
    def __init__(self, row, col, width, total_rows, flag):
        self.row = row
        self.col = col
        self.x = row * width
        self.y = col * width
        self.color = WHITE
        self.neighbors = []
        self.width = width
        self.total_rows = total_rows
        self.flag = flag

    def get_pos(self):
        return self.row, self.col

    def is_closed(self):
        return self.color == RED

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

    def make_start(self):
        self.color = ORANGE

    def make_closed(self):
        self.color = RED

    def make_open(self):
        self.color = GREEN

    def make_barrier(self):
        self.color = BLACK

    def make_end(self):
        self.color = TURQUOISE

    def make_path(self):
        self.color = PURPLE

    def draw(self, win):
        pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))

    def update_neighbors(self, grid):
        self.neighbors = []
        if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_barrier():  # DOWN
            self.neighbors.append(grid[self.row + 1][self.col])

        if self.row > 0 and not grid[self.row - 1][self.col].is_barrier():  # UP
            self.neighbors.append(grid[self.row - 1][self.col])

        if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrier():  # RIGHT
            self.neighbors.append(grid[self.row][self.col + 1])

        if self.col > 0 and not grid[self.row][self.col - 1].is_barrier():  # LEFT
            self.neighbors.append(grid[self.row][self.col - 1])

    def __lt__(self, other):
        return False


class Target:  # Hedef Clasımız
    def __init__(self, x, y, idNo):
        self.x = x
        self.y = y
        self.idNo = idNo

    def get_pos(self):
        return self.x, self.y


class Agv:  # Robotlar Classımız
    def __init__(self, x, y, idNo):
        self.idNo = idNo
        self.x = x
        self.y = y
        self.flag = 0
        self.firstPath = []
        self.secondPath = []

    def getNo(self):
        return self.idNo

    def get_pos(self):
        return self.x, self.y

    # def update_path(self, came_from, current):
    #     while current in came_from:
    #         current = came_from[current]
    #         self.currentPath.append(current)
    #         print(self.currentPath)

    def set_path(self, path):
        self.firstPath = path

    def get_path(self):
        return self.firstPath

    def set_secondPath(self, path):
        self.secondPath = path

    def get_secondPath(self):
        return self.secondPath


class Simulation:  # Tüm pyGame için yazılmış fonksiyonları bir clasın içine topladım bu sayede gride fonksiyon kullanarak araç ekleyebiliyoruz vs.
    def __init__(self, rows, width, win):
        self.rows = rows
        self.width = width
        self.grid = []
        self.AGVs = []
        self.targets = []
        self.win = win

    def make_grid(self):
        self.grid = []
        gap = self.width // self.rows
        for i in range(self.rows):
            self.grid.append([])
            for j in range(self.rows):
                spot = Spot(i, j, gap, self.rows, 0)
                self.grid[i].append(spot)
        return self.grid

    def get_grid(self):
        return self.grid

    def create_agv(self, x, y, idNo):  # Agv clasından obje üretip listesine atar
        vehicle = Agv(x, y, idNo)
        self.AGVs.append(vehicle)
        self.grid[x][y].make_start()

    def createTarget(self, x, y, idNo):  # Target clasından obje üretip listesine atar
        target = Target(x, y, idNo)
        self.targets.append(target)
        self.grid[x][y].make_end()

    def algorithm(self, draw, start,
                  end):  # aStar algoritması kullanarak verilen 2 nokta arasındaki en kısa mesafenin pathini dönderiyor
        count = 0
        spotAgv = self.get_grid()[start.get_pos()[0]][start.get_pos()[1]]
        spotTarget = self.get_grid()[end.get_pos()[0]][end.get_pos()[1]]
        open_set = PriorityQueue()
        open_set.put((0, count, spotAgv))
        came_from = {}
        g_score = {spot: float("inf") for row in self.grid for spot in row}
        g_score[spotAgv] = 0
        f_score = {spot: float("inf") for row in self.grid for spot in row}
        f_score[spotAgv] = self.h(spotAgv.get_pos(), spotTarget.get_pos())
        path = []

        open_set_hash = {spotAgv}

        while not open_set.empty():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()

            current = open_set.get()[2]
            open_set_hash.remove(current)

            if current == spotTarget:
                # start.set_path()
                # spotTarget.make_end()
                return self.reconstruct_path(came_from, spotTarget, path, draw)

            for neighbor in current.neighbors:
                temp_g_score = g_score[current] + 1

                if temp_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = temp_g_score
                    f_score[neighbor] = temp_g_score + self.h(neighbor.get_pos(), spotTarget.get_pos())
                    if neighbor not in open_set_hash:
                        count += 1
                        open_set.put((f_score[neighbor], count, neighbor))
                        open_set_hash.add(neighbor)
                # neighbor.make_open()

            draw()
        return False

    def aStar(self, target,
              goal):  # aStar algoritmasını kullanarak mevcut robotlardan en yakın olanının path'ini set ediyor
        result = [None] * len(self.AGVs)
        path = [None] * len(self.AGVs)
        path2 = self.algorithm(lambda: self.draw(), target, goal)
        path2.reverse()
        i = 0
        for agv in self.AGVs:
            path[i] = self.algorithm(lambda: self.draw(), agv, target)
            result[i] = len(path[i])
            i += 1
        chosen = result.index(min(result))
        self.AGVs[chosen].set_path(path[chosen])
        self.AGVs[chosen].set_secondPath(path2)
        self.draw_path(self.AGVs[chosen])

    def h(self, p1, p2):
        x1, y1 = p1
        x2, y2 = p2
        return abs(x1 - x2) + abs(y1 - y2)

    def reconstruct_path(self, came_from, current, path, draw):  # nerden geldiğine göre path'i buluyor
        path.append(current.get_pos())
        draw()
        while current in came_from:
            current = came_from[current]
            path.append(current.get_pos())
        path.remove(current.get_pos())
        return path

    def draw_grid(self, win, rows, width):
        gap = width // rows
        for i in range(rows):
            pygame.draw.line(win, GREY, (0, i * gap), (width, i * gap))
            for j in range(rows):
                pygame.draw.line(win, GREY, (j * gap, 0), (j * gap, width))

    def draw_path(self, agv):  # Pathi çizdiriyorum
        for x, y in agv.get_path()[1:]:
            spotAgv = self.get_grid()[x][y]
            spotAgv.make_path()
        for x, y in agv.get_secondPath()[:-1]:
            spotAgv = self.get_grid()[x][y]
            spotAgv.make_path()

    def draw(self):
        self.win.fill(WHITE)

        for row in self.grid:
            for spot in row:
                spot.draw(self.win)
        self.draw_grid(self.win, self.rows, self.width)
        pygame.display.update()

    def get_clicked_pos(self, pos, rows, width):
        gap = width // rows
        y, x = pos

        row = y // gap
        col = x // gap

        return row, col


def main(win, width):
    ROWS = 50
    # grid = make_grid(ROWS, width)
    simulation = Simulation(ROWS, WIDTH, win)
    simulation.make_grid()
    run = True
    while run:
        simulation.draw()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    for row in simulation.get_grid():
                        for spot in row:
                            spot.update_neighbors(simulation.get_grid())
                    simulation.create_agv(3, 5, 3)
                    simulation.create_agv(11, 11, 7)
                    simulation.create_agv(11, 9, 1)
                    simulation.create_agv(17, 21, 1)
                    simulation.createTarget(8, 8, 0)
                    goal = simulation.get_grid()[4][10]
                    simulation.get_grid()[4][10].make_closed()
                    # simulation.algorithm(lambda: simulation.draw(), simulation.AGVs[0], simulation.targets[0])
                    simulation.aStar(simulation.targets[0], goal)
                    # print(simulation.AGVs[2].get_path())
                    # print(simulation.AGVs[2].get_secondPath())
    pygame.quit()


main(WIN, WIDTH)
