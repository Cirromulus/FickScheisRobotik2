from mars_interface import *
import behavior
import math
import intersect
import numpy as np
from euclid import *
import time

distance = np.zeros(16)
pos = Vector3()
direction = 0.0
pointCloudData = {}
t = 0.
walls = None
waypoints = None
path = None
firstUpdate = True
numParticles = 0;

def addPointCloudData(name, points):
    global pointCloudData, numParticles
    pointCloudData[name] = points
    logMessage(name)
    if name == "waypoints":
        i = 0
        for waypoint in waypoints:
            points[i*3] = waypoint[0]
            points[i*3+1] = waypoint[1]
            points[i*3+2] = 0.5
            i += 1
    if name == "particles":
        for i in range(numParticles):
            points[i*3] = 0.0
            points[i*3+1] = 0.0
            points[i*3+2] = 0.5

def loadPoints(fName):
    array = []
    with open(fName) as f:
        for line in f:
            line = line.strip()
            if len(line) == 0 or line[0] == "#":
                continue
            arrLine = line.split()
            if len(arrLine) == 4:
                array.append([float(arrLine[0]), float(arrLine[1]),
                              float(arrLine[2]), float(arrLine[3])])
            elif len(arrLine) == 2:
                array.append([float(arrLine[0]), float(arrLine[1])])
    return np.array(array)

def init():
    global light_sensor, walls, waypoints, firstUpdate, numParticles
    firstUpdate = True
    reload(behavior)
    reload(intersect)
    clearDict()
    #setRunning(True)
    requestSensor("position")
    requestSensor("rotation")
    requestSensor("laser_vl")
    requestSensor("laser_vr")
    requestSensor("laser_hl")
    requestSensor("laser_hr")
    requestConfig("VirtualJoystick", "x")
    requestConfig("VirtualJoystick", "y")

    setConfig("Graphics", "showCoords", 0)
    setConfig("Scene", "skydome_enabled", 1)
    setConfig("Simulator", "calc_ms", 40)
    setUpdateTime(10)

    numParticles = behavior.getNumberOfParticles()

    # init point lists
    waypoints = loadPoints("waypoints.txt")
    logMessage("loaded " + str(len(waypoints)) + " waypoints")
    createPointCloud("waypoints", len(waypoints))
    createPointCloud("particles", numParticles)

    walls = loadPoints("walls.txt")
    #intersect.initStaticLines(walls)
    logMessage("loaded " + str(len(walls)) + " walls")
    clearLines("walls")
    clearLines("path")
    for w in walls:
        appendLines("walls", w[0], w[1], 2.4)
        appendLines("walls", w[2], w[3], 2.4)
    configureLines("walls", 3.0, 0, 1, 0)

    # caluculate squared lenth of walls
    print "init"
    return sendDict()

def getVector3(myList):
    v = Vector3()
    if len(myList) == 3:
        v = Vector3(myList[0], myList[1], myList[2])
    return v

def readSensor(marsData):
    global distance, pos, direction
    for i in range(4):
        distance[i] = marsData["Sensors"]["laser_vl"][i]
    for i in range(4):
        distance[4+i] = marsData["Sensors"]["laser_vr"][i]
    for i in range(4):
        distance[8+i] = marsData["Sensors"]["laser_hl"][i]
    for i in range(4):
        distance[12+i] = marsData["Sensors"]["laser_hr"][i]
    pos = getVector3(marsData["Sensors"]["position"])
    direction = math.radians(getVector3(marsData["Sensors"]["rotation"]).z)

def update(marsData):
    global numParticles, distance, t, pointCloudData, firstUpdate, walls, numParticles
    clearDict()
    readSensor(marsData)

    joystickLeft = marsData["Config"]["VirtualJoystick"]["x"]
    joystickRight = marsData["Config"]["VirtualJoystick"]["y"]

    first = False
    if firstUpdate:
        firstUpdate = False
        configurePointCloud("waypoints", 8.0, 0.2, 0.6, 0.8)
        configurePointCloud("particles", 6.0, 0.2, 0.8, 0.2)
        first=True


    particles = behavior.doBehavior(distance, direction, pos, marsData, waypoints, walls, joystickLeft, joystickRight)

    if not first:
        pData = pointCloudData["particles"]
        for i in range(len(particles)):
            logMessage("particle" + str(i) + ": " + str(particles[i]))
            pData[i*3] = particles[i][0]
            pData[i*3+1] = particles[i][1]
            #pData[i*3+2] = 5.2

    setMotor("motor_links", joystickLeft * 0.5)
    setMotor("motor_rechts", joystickRight * 0.5)



    return sendDict()
