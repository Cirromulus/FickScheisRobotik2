from mars_interface import *
import random
import intersect
import math
import numpy as np

goal = [6.,0.]
nodes = []
wps = []
ini = False

class Node:
    comingFrom = None
    distance = 10000000.
    coords = [0.,0.]
    neighbours = []

    def __init__(self, coords):
        self.coords = coords
        self.comingFrom = self

    def __eq__(self, other):
        if isinstance(other, Node):
            return self.coords == other.coords
        else:
            return NotImplemented

    #def expand(self):


def calcNeighbours(pos):
    global wps
    neighbours = []
    for wp in wps:
        #print "pos "+ str(pos) + " wp " + str(wp)
        if(not np.array_equal(wp, pos) and checkIntersect(pos, wp)):
            n = Node(wp)
            neighbours.append(n)
    return neighbours

def initNodes():
    global wps, goal
    wps.append(goal)
    for wp in wps:
        newNode = Node(wp)
        newNode.neighbours = calcNeighbours(newNode.coords)
        nodes.append(newNode)

def aStar(pos):
    global nodes, goal
    #print "goal is: " + str(goal)
    pos2d = [pos[0], pos[1]]
    openList = []
    closedList = []
    start = Node(pos2d)

    start.neighbours = calcNeighbours(start.coords)
    openList.append(start)

    while openList:
        current = min(openList, key=lambda x: x.distance)
        if current.coords == goal:
            print "FOUND PATH"
            return True
        openList.remove(current)
        closedList.append(current)
        print "current is: " + str(current.coords)

        for n in current.neighbours:
            if n in closedList:
                continue
            tempDist = current.distance + dist(current.coords, n.coords)

            if n not in openList:
                openList.append(n)
            elif current.distance >= n.distance:
                continue

            n.comingFrom = current
            n.distance = tempDist

    #print "did not find a path"
    return False

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
    global ini, wps
    if not ini:
        for wp in waypoints:
            wps.append([wp[0], wp[1]])
        initNodes()
        print "initialized!!"
        ini = True
    #print "doBehavior: " + str(pos[0])
    clearLines("path")
    configureLines("path", 5, 0.2, 0.8, 0.2)

    aStar(pos)

    for wp in waypoints:
        if checkIntersect(pos,wp):
             #print str(pos[0]) + ":" + str(pos[1]) + " to " + str(wp[0]) + ":" + str(wp[1]) + ": no intersection"
             appendLines("path", pos[0], pos[1], 0.5)
             appendLines("path", wp[0], wp[1], 0.5)




#    message = "sensor:"

    return
