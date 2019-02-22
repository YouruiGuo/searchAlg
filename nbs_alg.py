import numpy as np
import signal, time
import sys
sys.path.append("env/")
from stp_env_new import *
#from voxel_env_new import *

class Heap(object):
    """docstring for Heap."""
    def __init__(self, array):
        self.queue = []
        self.array = array

    def heap_push(self, index, hcost):
        self.queue.append(index)
        self.array[index].open_id = len(self.queue)-1
        self.array[index].hcost = hcost
        if self.array[index].value in [0, 1]:
            self.siftUpByFcost(self.array[index].open_id)
        else:
            self.siftUpByGcost(self.array[index].open_id)

    def heap_pop(self):
        result = self.queue[0]
        self.queue[0] = self.queue[len(self.queue)-1]
        self.queue.pop(-1)
        if self.array[result].value in [0, 1]:
            self.siftDownByFcost(0)
        else:
            self.siftDownByGcost(0)
        return result

    def peak(self):
        return self.queue[0]

    def siftUpByFcost(self, index):
        #index = len(self.queue)-1
        while (index-1)/2 >= 0:
            a = self.array[self.queue[index]].gcost + \
                self.array[self.queue[index]].hcost # child
            b = self.array[self.queue[(index-1)/2]].gcost + \
                self.array[self.queue[(index-1)/2]].hcost # parent
            if a < b:
                self.swap(index, (index-1)/2)
            index = (index-1)/2

    def siftUpByGcost(self, index):
        #index = len(self.queue)-1
        while (index-1)/2 >= 0:
            a = self.array[self.queue[index]].gcost # child
            b = self.array[self.queue[(index-1)/2]].gcost # parent
            if a < b:
                self.swap(index, (index-1)/2)
            index = (index-1)/2

    def siftDownByFcost(self, i):
        while i*2+1 < len(self.queue):
            minchild = self.min_childByFcost(i)
            a = self.array[self.queue[i]].gcost + \
                self.array[self.queue[i]].hcost # parent
            b = self.array[self.queue[minchild]].gcost + \
                self.array[self.queue[minchild]].hcost # child
            if a > b:
                self.swap(i, minchild)
            i = minchild

    def siftDownByGcost(self, i):
        while i*2+1 < len(self.queue):
            minchild = self.min_childByGcost(i)
            a = self.array[self.queue[i]].gcost  # parent
            b = self.array[self.queue[minchild]].gcost # child
            if a > b:
                self.swap(i, minchild)
            i = minchild

    def min_childByFcost(self, i):
        if i*2+2 >= len(self.queue): # right child is null
            return i*2+1              # return left child

        a = self.array[self.queue[i*2+1]].gcost + \
            self.array[self.queue[i*2+1]].hcost # left child
        b = self.array[self.queue[i*2+2]].gcost + \
            self.array[self.queue[i*2+2]].hcost # right child
        if a == b:
            if self.array[self.queue[i*2+1]].gcost > \
                self.array[self.queue[i*2+2]].gcost:
                return i*2+1
            else:
                return i*2+2
        if a < b:
            return i*2+1

        return i*2+2

    def min_childByGcost(self, i):
        if i*2+2 >= len(self.queue): # right child is null
            return i*2+1              # return left child
        a = self.array[self.queue[i*2+1]].gcost # left child
        b = self.array[self.queue[i*2+2]].gcost  # right child
        if a < b:
            return i*2+1
        return i*2+2

    def swap(self, i, j):
        temp = self.queue[i]
        self.queue[i] = self.queue[j]
        self.queue[j] = temp
        self.array[self.queue[i]].open_id = i
        self.array[self.queue[j]].open_id = j
        return

class SearchAlgorithm(object):
    """docstring for SearchAlgorithm."""
    def __init__(self, start, goal):
        self.heu = Heuristics(start, goal)
        self.env = self.heu.env
        self.start_index = self.env.hashStates[self.env.start_hash]
        self.goal_index = self.env.hashStates[self.env.goal_hash]
        self.env.array[self.start_index].gcost = 0
        self.env.array[self.start_index].hcost = self.heu.HCost_NBS(self.start_index, self.goal_index)
        self.env.array[self.goal_index].gcost = 0
        self.env.array[self.goal_index].hcost = self.heu.HCost_NBS(self.goal_index, self.start_index)
        self.openf = Heap(self.env.array) # open list for forward
        self.openb = Heap(self.env.array) # open list for backward
        self.readyf = Heap(self.env.array) # ready list for forward
        self.readyb = Heap(self.env.array) # ready list for backward
        self.closedf = []
        self.closedb = []
        self.Clb = 0 #  lower bound on lbmin
        self.C = float("inf")
        self.max_open = 0
        self.expanded = 0
        self.updated = 0
        self.printt()

    def printt(self):
        #print self.env.array[self.start_index].hcost
        print self.env.array[self.start_index].stateValue

    def search(self):
        self.openf.heap_push(self.start_index, self.env.array[self.start_index].hcost)
        self.env.array[self.start_index].value = 0 # in forward openlist
        self.openb.heap_push(self.goal_index, self.env.array[self.goal_index].hcost)
        self.env.array[self.goal_index].value = 1 # in backward openlist
        self.Clb = self.env.array[self.start_index].hcost

        while True:
            if self.Clb >= self.C:
                return self.C
            f_index, b_index = self.bestPair()
            self.forwardExpand(f_index)
            self.backwardExpand(b_index)

    def bestPair(self):

        if len(self.openf.queue) == 0:
            f_min = float("inf")
        else:
            f_min = self.env.array[self.openf.peak()].gcost+\
                self.env.array[self.openf.peak()].hcost

        if len(self.openb.queue) == 0:
            b_min = float("inf")
        else:
            b_min = self.env.array[self.openb.peak()].gcost+\
                self.env.array[self.openb.peak()].hcost

        while f_min < self.Clb:
            index = self.openf.heap_pop()
            self.readyf.heap_push(index, self.heu.HCost_NBS(index, self.goal_index))
            self.env.array[index].value = 2 # in forward ready list
            if len(self.openf.queue) == 0:
                f_min = float("inf")
            else:
                f_min = self.env.array[self.openf.peak()].gcost+\
                    self.env.array[self.openf.peak()].hcost

        while b_min < self.Clb:
            index = self.openb.heap_pop()
            self.readyb.heap_push(index, self.heu.HCost_NBS(index, self.start_index))
            self.env.array[index].value = 3 # in backward ready list
            if len(self.openb.queue) == 0:
                b_min = float("inf")
            else:
                b_min = self.env.array[self.openb.peak()].gcost+\
                    self.env.array[self.openb.peak()].hcost
        while True:
            if ((len(self.openf.queue) == 0) and (len(self.readyf.queue) == 0)) \
                or ((len(self.openb.queue) == 0) and (len(self.readyb.queue) == 0)):
                return None

            if len(self.readyf.queue) == 0:
                f_min = float("inf")
            else:
                f_min = self.env.array[self.readyf.peak()].gcost

            if len(self.readyb.queue) == 0:
                b_min = float("inf")
            else:
                b_min = self.env.array[self.readyb.peak()].gcost

            if f_min + b_min <= self.Clb:
                self.env.array[self.readyf.peak()].value = -1
                self.env.array[self.readyf.peak()].open_id = -1
                self.env.array[self.readyb.peak()].value = -1
                self.env.array[self.readyb.peak()].open_id = -1
                return [self.readyf.heap_pop(), self.readyb.heap_pop()]

            ggsum = f_min + b_min

            if len(self.openf.queue) == 0:
                f_min = float("inf")
            else:
                f_min = self.env.array[self.openf.peak()].gcost+\
                    self.env.array[self.openf.peak()].hcost

            if len(self.openb.queue) == 0:
                b_min = float("inf")
            else:
                b_min = self.env.array[self.openb.peak()].gcost+\
                    self.env.array[self.openb.peak()].hcost

            if f_min <= self.Clb:
                index = self.openf.heap_pop()
                self.readyf.heap_push(index, self.heu.HCost_NBS(index, self.goal_index))
                self.env.array[index].value = 2 # in ready list
            else:
                self.Clb = min(f_min, b_min, ggsum)

            if b_min <= self.Clb:
                index = self.openb.heap_pop()
                self.readyb.heap_push(index, self.heu.HCost_NBS(index, self.start_index))
                self.env.array[index].value = 3 # in ready list
            else:
                self.Clb = min(f_min, b_min, ggsum)

    def forwardExpand(self, index):
        #print "forward expand"
        # expand index, move it to forward closed list
        self.closedf.append(index)
        # get actions
        actions = self.env.getActions(index)
        #print "actions", actions
        for action in actions:
            # get the adjacent state
            next = self.env.applyAction(action, index)
            if self.env.array[next].value in [1, 3]: # in backward openlist
                sum_gcost = self.env.array[index].gcost + \
                    self.env.cost(index, next) + self.env.array[next].gcost
                if sum_gcost < self.C:
                    self.C = sum_gcost

                self.max_open += 1
                self.env.array[next].gcost = self.env.array[index].gcost + \
                                                self.env.cost(index, next)
                self.env.array[next].parent = index
                self.openf.heap_push(next,self.heu.HCost_NBS(next, self.goal_index))
                self.env.array[next].value = 0

            elif (self.env.array[next].value in [0, 2]) or (next in self.closedf):
                gcost = self.env.array[index].gcost + self.env.cost(index, next)
                if gcost >= self.env.array[next].gcost:
                    continue
                else:
                    if self.env.array[next].value == 0:
                        self.updated += 1
                        self.env.array[next].gcost = gcost
                        self.env.array[next].parent = index
                        self.openf.siftUpByFcost(self.env.array[next].open_id)
                    elif self.env.array[next].value == 2:
                        self.updated += 1
                        self.env.array[next].gcost = gcost
                        self.env.array[next].parent = index
                        self.readyf.siftUpByGcost(self.env.array[next].open_id)
                    '''
                    elif next in self.closedf:
                        self.closedf.remove(next)
                        self.openf.heap_push(next, self.heu.HCost_NBS(next, self.goal_index))
                        self.env.array[next].value = 0
                    '''
            else:
                self.max_open += 1
                self.env.array[next].gcost = self.env.array[index].gcost + \
                                                self.env.cost(index, next)
                self.env.array[next].parent = index
                self.openf.heap_push(next,self.heu.HCost_NBS(next, self.goal_index))
                self.env.array[next].value = 0

    def backwardExpand(self, index):
        #print "backward expand"
        # expand index, move it to backward closed list
        self.closedb.append(index)
        # get actions
        actions = self.env.getActions(index)
        for action in actions:
            # get the adjacent state
            next = self.env.applyAction(action, index)
            if self.env.array[next].value in [0, 2]: # in forward openlist
                sum_gcost = self.env.array[index].gcost + \
                    self.env.cost(index, next) + self.env.array[next].gcost
                if sum_gcost < self.C:
                    self.C = sum_gcost

                self.max_open += 1
                self.env.array[next].gcost = self.env.array[index].gcost + \
                                                self.env.cost(index, next)
                self.env.array[next].parent = index
                self.openb.heap_push(next,self.heu.HCost_NBS(next, self.start_index))
                self.env.array[next].value = 1

            if self.env.array[next].value in [1, 3] or \
                next in self.closedb:
                gcost = self.env.array[index].gcost + self.env.cost(index, next)
                if gcost >= self.env.array[next].gcost:
                    continue
                else:
                    if self.env.array[next].value == 1:
                        self.updated += 1
                        self.env.array[next].gcost = gcost
                        self.env.array[next].parent = index
                        self.openb.siftUpByFcost(self.env.array[next].open_id)
                    elif self.env.array[next].value == 3:
                        self.updated += 1
                        self.env.array[next].gcost = gcost
                        self.env.array[next].parent = index
                        self.readyb.siftUpByGcost(self.env.array[next].open_id)
                    '''
                    elif next in self.closedb: # not used
                        self.closedb.remove(next)
                        self.openb.heap_push(next, self.heu.HCost_NBS(next, self.start_index))
                        self.env.array[next].value = 1
                    '''
            else:
                self.max_open += 1
                self.env.array[next].gcost = self.env.array[index].gcost + \
                                                self.env.cost(index, next)
                self.env.array[next].parent = index
                self.openb.heap_push(next,self.heu.HCost_NBS(next, self.start_index))
                self.env.array[next].value = 1

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

def timeoutHandler(signum, frame):
    raise IndexError("3 minute mark")

if __name__ == '__main__':
    benchmark = loadstpFile(int(sys.argv[1]))
    print "sliding tile puzzle"
    test = [[1,2,7,6],[4,10,3,15],[8,11,5,14],[9,12,13,0]]
    #benchmark = load3dFile("./data/Simple.3dmap.3dscen",int(sys.argv[1]))
    #print "3d voxel-based pathfinding"
    count = 0
    signal.signal(signal.SIGALRM, timeoutHandler)
    signal.alarm(1800)
    #for benchmark in benchmarks:
    count += 1
    search = SearchAlgorithm(benchmark["start"], benchmark["goal"])
    t1 = time.time()
    try:
        path = search.search()
        t2 = time.time()
        print "count", count, "cost", path, "updated", search.updated,\
             "closedlist", len(search.closedb)+len(search.closedf), \
             "openlist", search.max_open, "time", t2-t1

    except IndexError as e:
        print e
        t2 = time.time()
        print  "updated", search.updated,\
             "closedlist", len(search.closedb)+len(search.closedf), \
             "openlist", search.max_open, "time", t2-t1
