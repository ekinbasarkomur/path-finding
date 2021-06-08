import pygame
import math
from queue import PriorityQueue
import numpy as np
import time

WHITE = (255, 255, 255)

def qlearning_train():
    pass

class Qlearning:
    def __init__(self, grid):
        self.grid = grid
        self.path = []

    def update(self):
        if len(self.path) > 1:
            self.path[0].reset()
            self.path.pop(0)
            self.path[0].make_path()

    def ql(self, end):
        environment_rows = 20
        environment_columns = 20

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
        for i in range(20):
            row = self.grid[i]
            for j in range(20):
                spot = row[j]
                if spot.score == -100:
                    rewards[spot.row, spot.col] = spot.score
                if i == end.row and j == end.col:
                    rewards[spot.row, spot.col] = 100

        # define aisle locations (i.e., white squares) for rows 1 through 9
        aisles = {}  # store locations in a dictionary
        for i in range(20):
            row = self.grid[i]
            aisles[i] = []
            for j in range(20):
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
                i = 0
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
                    i += 1
                    if i > 1000000:
                        print("Couldn't find shortest path")
                        return []
                self.path.append(shortest_path)
                return shortest_path

        # define a function that determines if the specified location is a terminal state
        # define training parameters
        # the percentage of time when we should take the best action (instead of a random action)
        epsilon = 0.9
        discount_factor = 0.9  # discount factor for future rewards
        learning_rate = 0.9  # the rate at which the AI agent should learn

        # run through 1000 training episodes
        for episode in range(4000):
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
        return get_shortest_path



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


    def reconstruct_path(self, came_from, current):
        last = None
        while current in came_from:
            if last:
                last.reset()
            current = came_from[current]
            self.path.append(current)
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
                #import pdb; pdb.set_trace()
                self.reconstruct_path(came_from, end)
                end.make_end()
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
