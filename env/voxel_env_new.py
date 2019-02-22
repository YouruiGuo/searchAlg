import random
import numpy as np
import math
from copy import deepcopy

class State(object):
    """docstring for State."""
    def __init__(self, state):
        self.stateValue = state # [x, y, z]
        self.gcost = -1
        self.hcost = -1
        self.parent = -1
        self.open_id = -10 # not initialized
        self.value = -1

class VoxelGrids(object):
    """docstring for VoxelGrids."""
    def __init__(self, start, goal):
        self.hashStates = {}
        self.gcost = 0
        self.size = []
        self.array = []
        self.start = start
        self.goal = goal
        self.initialization(start, goal)

    def initialization(self, start, goal):
        self.singleActions = [-1, 0, 1]
        self.allActions = [[x,y,z] for x in self.singleActions for y in self.singleActions for z in self.singleActions]
        self.allActions.remove([0,0,0])
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

        self.loadMap("./data/Simple.3dmap")


    def loadMap(self, file):
        with open(file, "r+") as fp:
            line = fp.readline()
            self.size = [int(n) for n in line.split() if n.isdigit()]
            lines = fp.readlines()
            for line in lines:
                state = [int(n) for n in line.split() if n.isdigit()]
                st = State(state)
                self.array.append(st)
                index = len(self.array)-1
                s, hash = self.getStateHash(state)
                self.writeHashTable(hash, index)
                self.array[index].gcost = float("inf") # state is filled

    def writeHashTable(self, key, index):
        self.hashStates[key] = index

    def getStateHash(self, state):
        # maximum of x/y/z is 4096 = 2^12
        hash = state[0] << 12
        hash = (hash + state[1]) << 12
        hash += state[2]
        if hash not in self.hashStates:
            return [1, hash]
        else:
            return [0, hash]

    def checkSuccess(self, index): # check if the game is over
        count = 0
        state = self.array[index].stateValue
        for i in range(3):
            if self.goal[i] == state[i]:
                count += 1
        if count == 3:
            return True
        else:
            return False

    def isInMap(self, index): # check state s in map or not
        state = self.array[index].stateValue
        check = 1
        if state[0] < 0 or state[0] > self.size[0]:
            check = 0
        if state[1] < 0 or state[1] > self.size[1]:
            check = 0
        if state[2] < 0 or state[2] > self.size[2]:
            check = 0
        return check

    def getActions(self, index):
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
            # permutations of all possible single actions for an action
            filled = 0
            for a in aact: # check for every single action
                s = self.applyAction(a, index)
                if self.array[index].gcost == float("inf"):
                    # check if the voxel is filled or not
                    filled = 1
                if not self.isInMap(s): # check if the state is in map
                    filled = 1
            if not filled:
                actions.append(action)
        return actions

    def applyAction(self, action, index):
        old_state = self.array[index].stateValue
        state = deepcopy(old_state)

        for i in range(3):
            state[i] += action[i]

        s, v = self.getStateHash(state)
        if not s:
            return self.hashStates[v]
        self.array.append(State(state))
        id = len(self.array)-1
        self.writeHashTable(v, id)

        return id

    def cost(self, index, next):
        state = self.array[index].stateValue
        next = self.array[next].stateValue
        delta = 0
        for i in range(3):
            delta += abs(state[i]-next[i])
        return math.sqrt(delta)

class Heuristics(object):
    """docstring for Heuristics."""
    def __init__(self, start, goal):
        self.env = VoxelGrids(start, goal)
        self.goal = goal

    def HCost(self, index):
        # voxel Heuristic
        state = self.env.array[index].stateValue
        delta_x = abs(state[0] - self.goal[0])
        delta_y = abs(state[1] - self.goal[1])
        delta_z = abs(state[2] - self.goal[2])
        dmax = max(delta_x, delta_y, delta_z)
        dmin = min(delta_x, delta_y, delta_z)
        dmid = delta_x + delta_y + delta_z - dmax - dmin
        hvoxel = (math.sqrt(3)-math.sqrt(2))*dmin + (math.sqrt(2)-1)*dmid + dmax

        return hvoxel

    def HCost_NBS(self, index, goal):
        # voxel Heuristic
        state = self.env.array[index].stateValue
        goal = self.env.array[goal].stateValue
        delta_x = abs(state[0] - goal[0])
        delta_y = abs(state[1] - goal[1])
        delta_z = abs(state[2] - goal[2])
        dmax = max(delta_x, delta_y, delta_z)
        dmin = min(delta_x, delta_y, delta_z)
        dmid = delta_x + delta_y + delta_z - dmax - dmin
        hvoxel = (math.sqrt(3)-math.sqrt(2))*dmin + (math.sqrt(2)-1)*dmid + dmax

        return hvoxel
