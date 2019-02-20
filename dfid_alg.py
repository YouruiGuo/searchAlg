import numpy as np
from stp_env import *
#from voxel_env import *
import signal
import sys

class DFID(object):
    """docstring for DFID."""
    def __init__(self, env):
        self.env = env
        self.start = self.env.start
        self.expanded = 0

    def search(self, initAct):
        depth = 0
        success = None
        while success is None:
            self.env.visited = []
            #if depth > 10:
            #    break
            depth += 1
            path = []
            success = self.DFS(depth, self.start, initAct, path)
        return success

    def DFS(self, depth, rootState, prevAction, path):
        r = rootState
        self.expanded += 1
        #print "depth:", depth
        #print rootState, self.env.hashStates[rootState].stateValue
        #path.append(rootState)
        if self.env.checkSuccess(rootState):
            return rootState
        if depth == 0:
            return None

        actions = self.env.getActions(rootState)
        #print(self.env.hashStates[rootState],actions)
        for action in actions:
            #print action, actions
            #print rootState
            tempState = self.env.applyAction(action, rootState)
            if self.env.isRepeat(prevAction, action):
                continue
            #print(action, tempState, rootState)
            #print(tempState, self.env.checkVisited(tempState))
            p = self.DFS(depth-1, tempState, action, path)
            if p is not None:
                path.insert(0, rootState)
                return path
def load3dFile(file):
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
    return benchmarks

def loadstpFile():
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
    return benchmarks

def timeoutHandler(signum, frame):
    raise IndexError("3 minute mark")

if __name__ == '__main__':
    if sys.argv[1] == "stp":
        benchmarks = loadstpFile()
        print "sliding tile puzzle"
        initAct = -1
    if sys.argv[1] == "3d":
        initAct = [-1, -1, -1]
        benchmarks = load3dFile("./data/Simple.3dmap.3dscen")
        print "3d voxel-based pathfinding"
    count = 0
    signal.signal(signal.SIGALRM, timeoutHandler)
    signal.alarm(60)
    for benchmark in benchmarks:
        count += 1
        stp = slidingPuzzle(benchmark["start"], benchmark["goal"])
        search = DFID(stp)
        #vg = VoxelGrids(benchmark["start"], benchmark["goal"])
        #search = DFID(vg)

        try:
            path = search.search(initAct)
        except Exception as e:
            print "count:", count, "expanded:", search.expanded
            break
        #print(search.env.hashStates[search.start])
        #path = search.DFS(1, search.start, path=[])
        print "count:", count, "path:", path, "expanded:", search.expanded
        #print search.env.hashStates[path[-1]].stateValue
