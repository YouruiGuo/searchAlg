import numpy as np
import copy

class State(object):
    """docstring for State."""
    def __init__(self, state):
        self.stateValue = state
        self.gcost = 0
        self.hcost = 0
        self.open_id = -10 # not initialized
        self.parent = -1
        self.value = -1

class slidingPuzzle(object):
    """docstring for slidingPuzzle."""
    def __init__(self, start, goal):
        # Stores the index of state in array
        # key: hash value, value: index in array
        self.hashStates = {}
        # Stores all info of states (State())
        self.array = []
        self.size = 4
        self.gcost = 0
        self.start = start
        self.goal = goal
        self.initialization(start, goal)


    def initialization(self, start, goal):
        start_state = State(start)
        goal_state = State(goal)
        self.array.append(start_state)
        index = len(self.array)-1
        s, self.start_hash = self.getStateHash(start)
        self.writeHashTable(self.start_hash, index)
        self.array.append(goal_state)
        index += 1
        s, self.goal_hash = self.getStateHash(goal)
        self.writeHashTable(self.goal_hash, index)

    def writeHashTable(self, key, index):
        self.hashStates[key] = index

    def getStateHash(self, state):
        value = 0
        for row in state:
            for el in row:
                value = value << 4
                value += el
        if value not in self.hashStates:
            return [1, value] # first visit state
        else:
            return [0, value] # state expanded before

    def getEmpty(self, index): # get the position of empty tile
        state = self.array[index].stateValue
        for i in range(self.size):
            for j in range(self.size):
                if state[i][j] == 0:
                    #print state[i][j]
                    return(i, j)
        return False

    def checkSuccess(self, index): # check if the game is over
        count = 0
        state = self.array[index]
        size = self.size*self.size
        for i in range(self.size):
            for j in range(self.size):
                if state.stateValue[i][j] == self.goal[i][j]:
                    count += 1
        if count == size: # check state hash value
            return True
        else:
            return False

    def getActions(self, index):
        actions = []
        #print self.hashStates[currentState].stateValue
        empty = self.getEmpty(index)
        i = empty[0]
        j = empty[1]
        if i == 0:
            actions.append(2) #down
        elif i == self.size-1:
            actions.append(0) #up
        else:
            actions += [0, 2] # left and right

        if j == 0:
            actions.append(1) # right
        elif j == self.size-1:
            actions.append(3) # left
        else:
            actions += [1, 3] # up and down
        #print(i, j, actions)
        #return sorted(actions)
        return actions

    def applyAction(self, action, index):
        # move empty tile
        # 0: up, 1: right, 2: down, 3: left
        old_state = self.array[index].stateValue
        state = copy.deepcopy(old_state)
        empty = self.getEmpty(index)
        row = empty[0]
        col = empty[1]
        #print(row, col, action)
        if action == 0: # up
            state[row][col] = state[row-1][col]
            state[row-1][col] = 0
        elif action == 1: # right
            state[row][col] = state[row][col+1]
            state[row][col+1] = 0
        elif action == 2: # down
            state[row][col] = state[row+1][col]
            state[row+1][col] = 0
        else: # left
            state[row][col] = state[row][col-1]
            state[row][col-1] = 0

        s, v = self.getStateHash(state)
        if not s:
            return self.hashStates[v]
        self.array.append(State(state))
        id = len(self.array)-1
        self.writeHashTable(v, id)
        #print self.array[index].stateValue, self.array[id].stateValue
        return id

    def undoAction(self, action, index):
        # move empty tile
        # 0: down, 1: left, 2: up, 3: right
        state = self.array[index].stateValue
        empty = self.getEmpty(index)
        row = empty[0]
        col = empty[1]
        if action == 0: # down
            state[row][col] = state[row+1][col]
            state[row+1][col] = 0
        elif action == 1: # left
            state[row][col] = state[row][col-1]
            state[row][col-1] = 0
        elif action == 2: # up
            state[row][col] = state[row-1][col]
            state[row-1][col] = 0
        else: # right
            state[row][col] = state[row][col+1]
            state[row][col+1] = 0
        return 0

    def cost(self, state, next):
        return 1

class Heuristics(object):
    """docstring for Heuristics."""
    def __init__(self, start, goal):
        self.env = slidingPuzzle(start, goal)
        self.size = self.env.size

    def HCost(self, index):
        # Manhattan distance Heuristic
        hcost = 0
        empty = self.env.getEmpty(index)
        state = self.env.array[index].stateValue
        for i in range(self.size):
            for j in range(self.size):
                if i == empty[0] and j == empty[1]: # ignore empty tile
                    pass
                else:
                    r = state[i][j]/self.size
                    c = state[i][j]%self.size
                    hcost += abs(r - i)
                    hcost += abs(c - j)
        return hcost

    def HCost_NBS(self, index, goal):
        hcost = 0
        empty = self.env.getEmpty(index)
        state = self.env.array[index].stateValue
        goal = self.env.array[goal].stateValue
        for i in range(self.size):
            for j in range(self.size):
                if i == empty[0] and j == empty[1]:
                    continue
                else:
                    for k in range(self.size):
                        for l in range(self.size):
                            if state[i][j] == goal[k][l]:
                                hcost += abs(k-i)
                                hcost += abs(l-j)
        return hcost
