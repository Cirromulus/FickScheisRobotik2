from mars_interface import *
import random
import intersect
import math
import numpy as np

goal = [4.,4.]  #change value for new goal
nodes = []      #list of nodes
wps = []        #list of waypoint vectors
ini = False     #check if already initialized or not

#datastucture used for handling coordinates and predecessors of wapoints
class Node:
    comingFrom = None   #reference to predecessor
    sDistance = 10000000.    #shortest path from start to goal
    gDistance = 10000000.    #heuristical score. 
    coords = [0.,0.]
    neighbours = []     #neighbour nodes

    def __init__(self, coords):
        self.coords = coords
        self.comingFrom = self

    def __eq__(self, other):
        if isinstance(other, Node):
            return self.coords == other.coords
        else:
            return NotImplemented

#calculate neighbours of a given node
def calcNeighbourNodes(node):
    global nodes
    neighbours = []
    for n in nodes:
        if(not np.array_equal(n.coords, node.coords) and checkIntersect(n.coords, node.coords)):
            neighbours.append(n)
    return neighbours

#use waypoints to create all nodes and add goal to waypoint
def initNodes():
    global wps, nodes, goal
    wps.append(goal)
    for wp in wps:
        newNode = Node(wp)
        nodes.append(newNode)
    for n in nodes:
        n.neighbours = calcNeighbourNodes(n)

#reconstruct from goalnode
def constructPath(node):
    i = node
    path = []
    path.append(np.array(i.coords))
    while i.comingFrom != None:
        path.append(np.array(i.comingFrom.coords))
        i = i.comingFrom
    path.append(np.array(i.coords))
    #needed in reverse
    result = path[::-1]
    return result


def aStar(pos):
    global nodes, goal
    path = []
    pos2d = [pos[0], pos[1]]
    openList = []
    closedList = []
    #add startnode, given by robotposition
    start = Node(pos2d)
    start.comingFrom = None
    start.sDistance = 0
    start.hDistance = dist(start.coords, goal)
    start.neighbours = calcNeighbourNodes(start)
    openList.append(start)

    while openList:
        #find node with lowest heuristic score
        current = min(openList, key=lambda x: x.hDistance)
        if current.coords == goal:
            #print "FOUND PATH"
            path = constructPath(current)
            return path
        #remove from openList since it is going to be expanded now
        openList.remove(current)
        closedList.append(current)

        #EXPANSION of current node
        for n in current.neighbours:
            #successor already checked, continue
            if n in closedList:
                continue
            #calculate distance travelled on this path
            tempDist = current.sDistance + dist(current.coords, n.coords)

            #add neighbour to openList
            if n not in openList:
                openList.append(n)
            #dont update distance if path is longer than the one before
            elif tempDist >= n.sDistance:
                continue

            # update neighbour's data
            n.comingFrom = current
            n.sDistance = tempDist
            n.hDistance = n.sDistance + dist(n.coords, goal)


    #failed, so returning an empty path
    return path

def checkIntersect(pos, wp):
    # for wall in walls:
    #     if intersect.doIntersect(line, wall):
    #         return False
    if intersect.get_intersect(pos[0], pos[1], wp[0], wp[1]) < 1:
       return False
    return True

def dist(p1, p2):
    return np.sum(np.square(np.subtract(p1, p2)))

def doBehavior(distance, marsData, waypoints, walls, pos):
    global ini, wps, nodes
    #calculating the neighbours just once!
    if not ini:
        for wp in waypoints:
            wps.append([wp[0], wp[1]])
        initNodes()
        ini = True
    #print "doBehavior: " + str(pos[0])
    clearLines("path")
    configureLines("path", 5, 0.2, 0.8, 0.2)

    result = aStar(pos)

    #drawing path
    for i in range(len(result)-1):
        print result[i]
        appendLines("path", result[i][0], result[i][1], 0.5)
        appendLines("path", result[i+1][0], result[i+1][1], 0.5)

#    message = "sensor:"

    return
