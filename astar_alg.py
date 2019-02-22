import numpy as np
import signal, time
import sys
sys.path.append("env/")
#from stp_env_new import *
from voxel_env_new import *

class Heap(object):
    """docstring for Heap."""
    def __init__(self, array):
        self.queue = []
        self.array = array

    def heap_push(self, index, hcost):
        self.queue.append(index)
        self.array[index].open_id = len(self.queue)-1
        self.array[index].hcost = hcost
        self.siftUp(self.array[index].open_id)

    def heap_pop(self):
        result = self.queue[0]
        self.queue[0] = self.queue[len(self.queue)-1]
        self.queue.pop(-1)
        self.siftDown(0)
        return result

    def siftUp(self, index):
        #index = len(self.queue)-1
        while (index-1)/2 >= 0:
            a = self.array[self.queue[index]].gcost + \
                self.array[self.queue[index]].hcost # child
            b = self.array[self.queue[(index-1)/2]].gcost + \
                self.array[self.queue[(index-1)/2]].hcost # parent
            if a < b:
                self.swap(index, (index-1)/2)
            index = (index-1)/2

    def siftDown(self, i):
        while i*2+1 < len(self.queue):
            minchild = self.min_child(i)
            a = self.array[self.queue[i]].gcost + \
                self.array[self.queue[i]].hcost # parent
            b = self.array[self.queue[minchild]].gcost + \
                self.array[self.queue[minchild]].hcost # child
            if a > b:
                self.swap(i, minchild)
            i = minchild


    def min_child(self, i):
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
        self.env.array[self.start_index].gcost = 0
        self.env.array[self.start_index].hcost = self.heu.HCost(self.start_index)
        # heap, stores the index in array
        self.openlist = Heap(self.env.array)
        self.max_open = 0
        self.expanded = 0
        self.updated = 0
        self.printt()

    def printt(self):
        print self.env.array[self.start_index].stateValue

    def search(self):

        self.openlist.heap_push(self.start_index, self.heu.HCost(self.start_index))

        while len(self.openlist.queue) != 0:

            #print self.openlist.queue
            index = self.openlist.heap_pop()
            #print "next index", index
            #print self.env.array[index].stateValue
            # add to closed list
            self.env.array[index].open_id = -1 # open_id == -1 is in closed list
            self.expanded += 1
            #print "next step:", self.env.hashStates[currentState].stateValue
            #print len(self.openlist.queue)
            if self.env.checkSuccess(index):
                print self.env.array[index].stateValue
                return self.getPath(index)
                #return [self.env.array[index].gcost, self.getPath(index)]
            # get actions
            actions = self.env.getActions(index)
            # update steps -> g-cost
            for action in actions:
                # get the adjacent state
                next = self.env.applyAction(action, index)
                open_id = self.env.array[next].open_id
                #print next, open_id
                if open_id == -1:
                    # ignore if this state is in closedlist
                    #print "closed", "index", next, "state", self.env.array[next].stateValue
                    continue
                elif open_id > -1:
                    # update g-cost if is in openlist
                    #print "update"
                    newCost = self.env.array[index].gcost + \
                        self.env.cost(index, next)
                    if newCost < self.env.array[next].gcost:
                        self.updated += 1
                        self.env.array[next].gcost = newCost
                        self.env.array[next].parent = index
                        #self.openlist.queue.pop(self.env.array[next].open_id)
                        self.openlist.siftUp(self.env.array[next].open_id)
                        #print next, self.env.array[next].open_id
                else:
                    #print "add open", "index", next, "state", self.env.array[next].stateValue
                    # update the g-cost of this state
                    self.env.array[next].gcost = self.env.array[index].gcost \
                            + self.env.cost(index, next)
                    # update parent
                    self.env.array[next].parent = index
                    # add the state to open list
                    self.openlist.heap_push(next, self.heu.HCost(next))
                    self.max_open += 1
                    #heappush(self.openlist, (fcost, next))
                    #self.env.array[self.start_index].open_id = self.openlist.index((fcost, next))

    def getPath(self, goal):
        path = [goal]
        state = self.env.array[goal]
        prev = goal
        while state.parent != -1:
            path.insert(0, state.parent)
            prev = state.parent
            state = self.env.array[state.parent]
        return path

    def statistics(self):
        if self.max_open < len(self.openlist):
            self.max_open = len(self.openlist)
            if self.max_open % 1000 == 0:
                print("openlist:",len(self.openlist))
        if len(self.closedlist)%1000 == 0:
            print("closedlist:",len(self.closedlist))

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

if __name__ == '__main__':
    #benchmark = loadstpFile(int(sys.argv[1])-1)
    #print "sliding tile puzzle"
    benchmark = load3dFile("./data/Simple.3dmap.3dscen", int(sys.argv[2]))
    print "3d voxel-based pathfinding"
    count = 0

    #for benchmark in benchmarks:
    count += 1
    search = SearchAlgorithm(benchmark["start"], benchmark["goal"])
    t1 = time.time()
    try:
        path = search.search()
        t2 = time.time()
        print "count", count, "path", path, "expanded", search.expanded,\
            "updated", search.updated, "openlist", search.max_open, "time", t2-t1
    except Exception as e:
        t2 = time.time()
        print "***Exception:", "expanded", search.expanded,\
            "updated", search.updated, "openlist", search.max_open, "time", t2-t1


        #print "count:", count,"optimal path:,path[0],ratio:, 1.0*(path[0]/search.env.array[search.start_index].hcost)", \
        #    "openlist:", len(search.openlist.queue), "updated:", search.updated
