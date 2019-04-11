import datetime
import time
import copy
import random
import sys

# 1 -> North
# 2 -> South
# 3 -> West
# 4 -> East


class VI:

    def __init__(self, world, policy, walls, end_states, step_cost, start_state, rows, cols, no_end_states, no_walls):

        self.ispart = "B"
        self.issubpart = "a"
        self.team_num = 25
        self.board = world
        self.dflag = True
        self.walls = walls
        self.tolerance = 0.01
        self.old_board = copy.deepcopy(self.board)
        self.start_state = start_state
        self.debug = True
        self.rows = rows
        self.pid = 0.8
        self.cols = cols
        self.step_cost = step_cost
        self.original = copy.deepcopy(self.board)
        self.end_states = end_states
        self.no_end_states = no_end_states
        self.no_walls = no_walls
        self.policy = policy
        self.npid = 0.1
        self.discount_factor = 0.99
        self.n_deci = 3
        self.ta_understanding = 1

        if self.ispart != "A":
            self.step_cost = -2.5
            self.discount_factor = 0.99

        if self.debug == True and self.dflag == True:
            print("Board = ", self.board)

        if self.debug == True and self.dflag == True:
            print(" rows,  cols = ",  self.rows,  self.cols)
            print("world = ", world)

        if self.debug == True and self.dflag == True:
            print("no of end states = ", self.no_end_states)
            print("End states  = ", self.end_states)

        if self.debug == True and self.dflag == True:
            print("no of walls = ", self.no_walls)
            print("Walls = ", self.walls)

        if self.debug == True and self.dflag == True:
            print("Start state = ", self.start_state)
            # print ("Step Cost = " , self.step_cost)

        if self.debug == True and self.dflag == False:

            print('--------------------------------Curr_pol------------------------')
            self.print_policy()

            print('--------------------------------Fin_pol---------------------------')
            self.print_policy()

            print('------------------------------Board------------------------------')
            self.print_world()

        if self.debug == True and self.dflag == False:
            print(self.is_valid(-3, 3))
            print(self.is_valid(-1, 2))
            print(self.is_valid(-10, 0))
            print(self.is_valid(-1, 2))
            print(self.is_valid(1, 22))

        self.init_world()
        self.value_iteration()
        self.get_policy()

    def is_valid(self, x, y):

        if x < 0 or x >= self.rows or y < 0 or y >= self.cols:
            return False

        for wall in self.walls:
            if x == wall[0] and y == wall[1]:
                return False

        return True

    def init_world(self):
        ''' WALLS initialsed to None '''
        for i in range(self.no_walls):
            x = self.walls[i][0]
            y = self.walls[i][1]

            self.board[x][y] = None
            if self.debug == True and self.dflag == False:
                print("x,y", x, y)
            self.policy[x][y] = None

        # init self.policy
        for i in range(self.no_end_states):
            x = end_states[i][0]
            y = end_states[i][1]

            if self.debug == True and self.dflag == False:
                print("x,y", x, y)

            if self.board[x][y] <= 0:
                self.policy[x][y] = "Bad"
            else:
                self.policy[x][y] = "Goal"

    # returns true if x,y is not an end state or wall.
    def get_valid_pos(self, x, y):
        flag = 1
        if (x, y) in self.end_states:
            flag = 0

        if (x, y) in self.walls:
            flag = 0

        if flag == 1:
            return True
        else:
            return False

    def print_policy(self):
        print()
        for i in range(self.rows):
            for j in range(self.cols):
                print('%15s' % self.policy[i][j], end=' ')
            print()
        print()

    def print_world(self):
        print()
        for i in range(self.rows):
            for j in range(self.cols):
                x = self.board[i][j]
                if type(x) == float:
                    x = round(x, self.n_deci)
                print('%15s' % x, end=' ')
            print()
        print()

    def value_iteration(self):
        print("Iteration #0")
        iter_no = 0
        self.print_world()
        # time.sleep(1)
        while True:
            changed_pairs = []
            iter_no += 1
            flag = 1
            print('Iteration #%d' % iter_no)
            for i in range(self.rows):
                for j in range(self.cols):
                    if (self.get_valid_pos(i, j) == True):
                        if self.debug == True and self.dflag == False:
                            print(i, j)
                        self.board[i][j] = self.board_update(tuple((i, j)))

                        if self.old_board[i][j] > 0 or self.old_board[i][j] < 0:
                            diff = ((self.board[i][j] - self.old_board[i][j]))
                            rel_diff = diff / (self.old_board[i][j])
                            if self.old_board[i][j] == None:
                                if self.debug == True and self.dflag == False:
                                    print('Is none')
                            changed_pairs.append(rel_diff)
                        else:
                            if self.old_board[i][j] == None:
                                if self.debug == True and self.dflag == False:
                                    print('Is None')
                            changed_pairs.append(69.0)
            # time.sleep(1)
            self.print_world()

            check_val = (self.tolerance * (1 - self.discount_factor)
                         ) / (self.discount_factor)

            if(self.ta_understanding != 1):
                for val in changed_pairs:
                    if (val > check_val):
                        flag = 0
            
            if(self.ta_understanding == 1):
                for val in changed_pairs:
                    if (val > self.tolerance):
                        flag = 0

            if flag == 1:
                return

            self.old_board = copy.deepcopy(self.board)

    def board_update(self, pair):
        neigh_utilis = [0.0, 0.0, 0.0, 0.0]
        vals = [0.0, 0.0, 0.0, 0.0]
        x = pair[0]
        y = pair[1]

        right_n = (x, y+1)
        left_n = (x, y-1)
        upar_n = (x-1, y)
        neeche_n = (x+1, y)

        neigh_utilis[0] = self.get_state_vals(
            right_n, self.old_board[x][y], 0)  # move east
        neigh_utilis[1] = self.get_state_vals(
            neeche_n, self.old_board[x][y], 0)  # move south
        neigh_utilis[2] = self.get_state_vals(
            left_n, self.old_board[x][y], 0)   # move west
        neigh_utilis[3] = self.get_state_vals(
            upar_n, self.old_board[x][y], 0)   # move north

        # what does the value come of P(J | I,A)* Ut(J) , a state can lead to 3 states, one in the intended direction , others in the non intended dir
        vals[0] = (neigh_utilis[0] * self.pid) + (neigh_utilis[1]
                                                  * self.npid) + (neigh_utilis[3] * self.npid)
        vals[1] = (neigh_utilis[1] * self.pid) + (neigh_utilis[0]
                                                  * self.npid) + (neigh_utilis[2] * self.npid)
        vals[2] = (neigh_utilis[2] * self.pid) + (neigh_utilis[1]
                                                  * self.npid) + (neigh_utilis[3] * self.npid)
        vals[3] = (neigh_utilis[3] * self.pid) + (neigh_utilis[0]
                                                  * self.npid) + (neigh_utilis[2] * self.npid)

        vals[0] = vals[0] * self.discount_factor
        vals[1] = vals[1] * self.discount_factor
        vals[2] = vals[2] * self.discount_factor
        vals[3] = vals[3] * self.discount_factor

        max_all = max(vals)

        return (self.step_cost + max_all)
        # return step_cost

    def get_policy(self):

        for i in range(self.rows):
            for j in range(self.cols):
                if (self.get_valid_pos(i, j) == True):
                    move = "0"
                    neigh_utilis = [0.0, 0.0, 0.0, 0.0]
                    vals = [0.0, 0.0, 0.0, 0.0]

                    x = i
                    y = j

                    right_n = (x, y+1)
                    left_n = (x, y-1)
                    upar_n = (x-1, y)
                    neeche_n = (x+1, y)

                    neigh_utilis[0] = self.get_state_vals(
                        right_n, self.old_board[x][y], 1)
                    neigh_utilis[1] = self.get_state_vals(
                        neeche_n, self.old_board[x][y], 1)
                    neigh_utilis[2] = self.get_state_vals(
                        left_n, self.old_board[x][y], 1)
                    neigh_utilis[3] = self.get_state_vals(
                        upar_n, self.old_board[x][y], 1)

                    vals[0] = neigh_utilis[0] * self.pid + \
                        (neigh_utilis[1] + neigh_utilis[3]) * self.npid
                    vals[1] = neigh_utilis[1] * self.pid + \
                        (neigh_utilis[0] + neigh_utilis[2]) * self.npid
                    vals[2] = neigh_utilis[2] * self.pid + \
                        (neigh_utilis[1] + neigh_utilis[3]) * self.npid
                    vals[3] = neigh_utilis[3] * self.pid + \
                        (neigh_utilis[0] + neigh_utilis[2]) * self.npid

                    vals[0] = vals[0] * self.discount_factor
                    vals[1] = vals[1] * self.discount_factor
                    vals[2] = vals[2] * self.discount_factor
                    vals[3] = vals[3] * self.discount_factor

                    max_all = max(vals)

                    if (vals[0] == max_all):
                        move = "4"
                    elif (vals[1] == max_all):
                        move = "2"
                    elif (vals[2] == max_all):
                        move = "3"
                    elif (vals[3] == max_all):
                        move = "1"

                    self.policy[i][j] = move

    # if calculated for an iteration , just returns the current board[x][y] , also returns '0' for a wall
    def get_state_vals(self, pair, val, p_flag):
        x = pair[0]
        y = pair[1]
        flag = 1

        if (self.is_valid(x, y) == False or self.old_board[x][y] == 0):
            flag = 0

        if flag == 0:
            return val
        else:
            if p_flag == 0:
                return self.old_board[x][y]
            else:
                return self.board[x][y]


if __name__ == '__main__':

    dflag = False

    rows, cols = map(int, input().split())
    # world = [[float(j) for j in (map(float, input().split()))] for i in range(rows)]
    world = [[0 for i in range(cols)] for j in range(rows)]
    end_states = []
    walls = []

    for i in range(rows):
        rowss = input()
        rowss = rowss.split()
        for j in range(cols):
            world[i][j] = float(rowss[j])

    no_end_states, no_walls = map(int, input().split())

    for i in range(no_end_states):
        inp = input()
        inp = inp.split()
        end_states.append(tuple((int(inp[0]), int(inp[1]))))

    if dflag == True:
        end_states = [[j for j in map(int, input().split())]
                      for i in range(no_end_states)]
        walls = [[j for j in map(int, input().split())]
                 for i in range(no_walls)]

    for i in range(no_walls):
        inp = input()
        inp = inp.split()
        walls.append(tuple((int(inp[0]), int(inp[1]))))

    inp = input()
    inp = inp.split()

    start_state = tuple((inp[0], inp[1]))

    if dflag == True:
        start_state = [[j for j in map(int, input().split())]
                       for i in range(1)]

    step_cost = float(input())
    policy = [["0" for i in range(cols)] for j in range(rows)]

    vi = VI(world, policy, walls, end_states, step_cost,
            start_state, rows, cols, no_end_states, no_walls)
    print("Optimal Policy :")
    vi.print_policy()
    print("Step Cost = ", vi.step_cost)
    print("Discount Factor = ", vi.discount_factor)
    # print("No self.org add")
