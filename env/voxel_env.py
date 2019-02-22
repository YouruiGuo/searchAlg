import random
import numpy as np
import math

class State(object):
    """docstring for State."""
    def __init__(self, state):
        self.stateValue = state # [x, y, z]
        self.gcost = -1
        self.hcost = -1
        self.parent = -1

    def getParent(self):
        return self.parent

    def updateParent(self, parent):
        self.parent = parent

class Action(object):
    """docstring for Action."""
    def __init__(self, action):
        self.action = action # [x, y, z]

class VoxelGrids(object):
    """docstring for VoxelGrids."""
    def __init__(self, start, goal):
        self.hashStates = {}
        self.visited = []
        self.size = []
        self.singleActions = [-1, 0, 1]
        self.allActions = [[x,y,z] for x in self.singleActions for y in self.singleActions for z in self.singleActions]
        self.allActions.remove([0,0,0])
        self.gcost = 0
        self.goalState = State(goal)
        self.goalHashState = self.getStateHash(self.goalState)
        self.start = self.getStateHash(State(start))
        self.loadMap("./data/Simple.3dmap")

    def loadMap(self, file):
        with open(file, "r+") as fp:
            line = fp.readline()
            self.size = [int(n) for n in line.split() if n.isdigit()]
            lines = fp.readlines()
            for line in lines:
                state = [int(n) for n in line.split() if n.isdigit()]
                st = State(state)
                hash = self.getStateHash(st)
                self.hashStates[hash].gcost = float("inf") # state is filled

    def getStateHash(self, s):
        # maximum of x/y/z is 1024 = 2^10
        state = s.stateValue
        hash = state[0] << 12
        hash = (hash + state[1]) << 12
        hash += state[2]
        if hash not in self.hashStates:
            self.hashStates[hash] = s
        return hash

    def getState(self, state):
        # maximum of x/y/z is 1024 = 2^10
        hash = state[0] << 12
        hash = (hash + state[1]) << 12
        hash += state[2]
        return hash

    def checkVisited(self, state): # check state hash value
        if state in self.visited:
            return True #visited
        else:
            return False

    def checkSuccess(self, currentState): # check if the game is over
        count = 0
        state = self.hashStates[currentState].stateValue
        for i in range(3):
            if self.goalState.stateValue[i] == state[i]:
                count += 1
        if count == 3:
            return True
        else:
            return False

    def isInMap(self, state): # check state s in map or not
        #state = self.hashStates[s].stateValue
        check = 1
        if state[0] < 0 or state[0] > self.size[0]:
            check = 0
        if state[1] < 0 or state[1] > self.size[1]:
            check = 0
        if state[2] < 0 or state[2] > self.size[2]:
            check = 0
        return check

    def isRepeat(self, prev, curr):
        count = 0
        if prev[0] == -curr[0]:
            count += 1
        if prev[1] == -curr[1]:
            count += 1
        if prev[2] == -curr[2]:
            count += 1
        if count == 3:
            return True
        else:
            return False


    def getActions(self, state):
        # state = [x, y, z]
        # action = [x^, y^, z^]
        actions = []
        for action in self.allActions: # check all possible actions
            lx, ly, lz = [0], [0], [0]
            if abs(action[0]) != 0:
                lx.append(action[0])
            if abs(action[1]) != 0:
                ly.append(action[1])
            if abs(action[2]) != 0:
                lz.append(action[2])

            aact = [[x, y, z] for x in lx for y in ly for z in lz]
            aact.remove([0,0,0])
            # permutations of all possible single actions for an action
            filled = 0
            for a in aact: # check for every single action
                s = self.checkAction(a, state)
                self.undoAction(a, state)
                if not s:
                    filled = 1
            if not filled:
                actions.append(action)
        return actions

    def applyAction(self, action, currentState):
        state = self.hashStates[currentState].stateValue

        for i in range(3):
            state[i] += action[i]

        return self.cost(action)

    def checkAction(self, action, currentState):
        state = self.hashStates[currentState].stateValue

        for i in range(3):
            state[i] += action[i]

        hash = self.getState(state)
        try:
            if self.hashStates[hash].gcost == float("inf"):
                return False
        except Exception as e:
            pass
        if not self.isInMap(state):
            return False
        return True

    def undoAction(self, action, currentState):
        state = self.hashStates[currentState].stateValue
        for i in range(3):
            state[i] -= action[i]
        return

    def cost(self, action):
        delta = 0
        for i in range(3):
            delta += abs(action[i])
        return math.sqrt(delta)

class Heuristics(object):
    """docstring for Heuristics."""
    def __init__(self, start, goal):
        self.env = VoxelGrids(start, goal)
        self.goal = goal

    def HCost(self, state):
        # voxel Heuristic
        state = self.env.hashStates[state].stateValue
        delta_x = abs(state[0] - self.goal[0])
        delta_y = abs(state[1] - self.goal[1])
        delta_z = abs(state[2] - self.goal[2])
        dmax = max(delta_x, delta_y, delta_z)
        dmin = min(delta_x, delta_y, delta_z)
        dmid = delta_x + delta_y + delta_z - dmax - dmin
        hvoxel = (math.sqrt(3)-math.sqrt(2))*dmin + (math.sqrt(2)-1)*dmid + dmax

        return hvoxel
