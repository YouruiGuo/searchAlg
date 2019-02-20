import random
import numpy as np
from copy import deepcopy

"""
    0 -> empty tile
    start state is randomized from goal state.
    goal state: [[0, 1, 2, 3],
                [4, 5, 6, 7],
                [8, 9, 10, 11],
                [12, 13, 14, 15]]
"""

class State(object):
    """docstring for State."""
    def __init__(self, state):
        self.stateValue = state
        self.gcost = 0
        self.hcost = 0
        self.parent = -1

    def getParent(self):
        return self.parent

    def updateParent(self, parent):
        self.parent = parent


class Action(object):
    """docstring for Action."""
    def __init__(self):
        self.action = 0


class slidingPuzzle(object):
    """docstring for slidingPuzzle."""
    def __init__(self, start, goal):
        self.hashStates = {}
        self.visited = []
        self.size = 4
        self.gcost = 0
        self.goalState = State(goal)
        self.goalHashState = self.getStateHash(self.goalState)
        self.start = self.getStateHash(State(start))

    def getStateHash(self, s):
        value = 0
        state = s.stateValue
        for row in state:
            for el in row:
                value = value << 4
                value += el
        if value not in self.hashStates:
            #print "change hash"
            self.hashStates[value] = s
        #print "print from getStateHash", value, state
        return value

    def checkVisited(self, state): # check state hash value
        if state in self.visited:
            return True #visited
        else:
            return False

    def checkSuccess(self, currentState): # check if the game is over
        count = 0
        size = self.size*self.size
        for i in range(self.size):
            for j in range(self.size):
                if self.hashStates[currentState].stateValue[i][j] == self.goalState.stateValue[i][j]:
                    count += 1
        if count == size: # check state hash value
            return True
        else:
            return False

    def getEmpty(self, currentState): # get the position of empty tile
        s = self.hashStates[currentState]
        state = s.stateValue
        for i in range(self.size):
            for j in range(self.size):
                if state[i][j] == 0:
                    #print state[i][j]
                    return(i, j)
        return False

    def isRepeat(self, prev, curr):
        if abs(curr-prev) == 2: # 0 <-> 2, 1 <-> 3,
            return True
        else:
            return False

    def getActions(self, currentState):
        actions = []
        #print self.hashStates[currentState].stateValue
        empty = self.getEmpty(currentState)
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


    def applyAction(self, action, currentState):
        # move empty tile
        # 0: up, 1: right, 2: down, 3: left
        state = self.hashStates[currentState].stateValue

        #if not self.checkVisited(currentState):
        #    self.visited.append(currentState)
        empty = self.getEmpty(currentState)
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
        cost = 1
        return cost

    def undoAction(self, action, currentState):
        # move empty tile
        # 0: down, 1: left, 2: up, 3: right
        state = self.hashStates[currentState].stateValue
        empty = self.getEmpty(currentState)
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

    def HCost(self, state):
        # Manhattan distance Heuristic
        hcost = 0
        empty = self.env.getEmpty(state)
        state = self.env.hashStates[state].stateValue
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
