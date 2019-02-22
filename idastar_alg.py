import numpy as np
import signal, time
import sys
sys.path.append("env/")
#from stp_env import *
from voxel_env import *

class IDAstar(object):
    """docstring for IDAstar."""
    def __init__(self, start, goal):
        self.heu = Heuristics(start, goal)
        self.env = self.heu.env
        self.start = self.env.start
        self.expanded = 0
        self.nextThreshold = self.heu.HCost(self.start)

    def search(self, initAct):
        success = None
        threshold = self.nextThreshold
        gcost = 0
        path = []
        while success is None:
            success = self.DFS(threshold, gcost, self.start, initAct, path)
            threshold = self.nextThreshold
        #print threshold
        return success

    def getCost(self, prev, curr):
        self.env.hashStates[curr].gcost = self.env.hashStates[prev].gcost + \
                                        self.env.cost(prev, curr)
        return self.env.hashStates[curr].gcost + self.heu.HCost(curr)

    def DFS(self, threshold, gcost, rootState, prevAction, path):
        self.expanded += 1
        fcost = gcost + self.heu.HCost(rootState)
        #print "threshold", threshold, "hcost", self.heu.HCost(rootState), "gcost", gcost
        #print rootState, self.env.hashStates[rootState].stateValue
        #path.append(rootState)
        if self.expanded % 10000 == 0:
            print self.expanded

        if self.env.checkSuccess(rootState):
            return rootState
        if threshold < fcost:
            if self.nextThreshold > fcost or self.nextThreshold == threshold:
                #print "next threshold", fcost, "hcost", self.heu.HCost(rootState), "gcost", gcost
                self.nextThreshold = fcost
            return None

        actions = self.env.getActions(rootState)
        for action in actions:
            #print action, actions
            #print rootState
            cost = self.env.applyAction(action, rootState)
            if self.env.isRepeat(prevAction, action):
                self.env.undoAction(action, rootState)
                continue
            #print(action, tempState, rootState)
            #print(tempState, self.env.checkVisited(tempState))
            p = self.DFS(threshold, gcost+cost, rootState, action, path)
            self.env.undoAction(action, rootState)
            if p is not None:
                path.insert(0, rootState)
                return path


def load3dFile(file, index):
    benchmarks = []
    with open(file, "r+") as fp:
        fp.readline()
        fp.readline()
        lines = fp.readlines()
        for line in lines:
            benchmark = {}
            splited = [float(n) for n in line.split()]
            benchmark["start"] = [int(splited[0]), int(splited[1]), int(splited[2])]
            benchmark["goal"] = [int(splited[3]), int(splited[4]), int(splited[5])]
            benchmark["path"] = splited[6]
            benchmark["ratio"] = splited[7]
            benchmarks.append(benchmark)
    return benchmarks[index]

def loadstpFile(index):
    benchmarks = []
    #file = "./data/felner1000.txt"
    file = "./data/korf100.txt"
    print "run on", file
    with open(file, "r") as fp:
        lines = fp.readlines()
    for line in lines:
        benchmark = {}
        nums = line.split()
        start = []
        for i in range(4):
            start.append([int(nums[i*4+1]), int(nums[i*4+2]), int(nums[i*4+3]), int(nums[i*4+4])])
        benchmark["start"] = start
        benchmark["goal"] = np.arange(0, 16).reshape(4, 4);
        benchmarks.append(benchmark)
    return benchmarks[index]

def timeoutHandler(signum, frame):
    raise IndexError("3 minute mark")

if __name__ == '__main__':
    if sys.argv[1] == "stp":
        benchmark = loadstpFile(int(sys.argv[2]))
        print "sliding tile puzzle"
        initAct = -1
    if sys.argv[1] == "3d":
        initAct = [-1, -1, -1]
        benchmark = load3dFile("./data/Simple.3dmap.3dscen", int(sys.argv[2]))
        print "3d voxel-based pathfinding"

    print benchmark["start"],  benchmark["goal"]
    signal.signal(signal.SIGALRM, timeoutHandler)
    signal.alarm(1800)
    #stp = slidingPuzzle(benchmark["start"], benchmark["goal"])
    search = IDAstar(benchmark["start"],  benchmark["goal"])
    #vg = VoxelGrids(benchmark["start"], benchmark["goal"])
    #search = IDAstar(vg)
    #try:
    path = search.search(initAct)
    print "path length:",len(path), "expanded:", search.expanded
    '''
    except Exception as e:
        print "expanded:", search.expanded
        exit()
    '''

    #print(search.env.hashStates[search.start])
    #path = search.DFS(1, search.start, path=[])

    #print search.env.hashStates[path[-1]].stateValue
