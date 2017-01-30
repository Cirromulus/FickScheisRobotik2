from mars_interface import *
import behavior
import astar
import particlefilter as pf
import math
import intersect
import numpy as np
from euclid import *
import time

distance = np.zeros(16)
pos = Vector3()
wheels = np.array([0., 0.])
direction = 0.0
pointCloudData = {}
t = 0.
walls = None
waypoints = None
numParticles = 100
firstUpdate = True

def addPointCloudData(name, points):
    global pointCloudData, numParticles
    pointCloudData[name] = points
    behavior.pointCloudData[name] = points
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
    global light_sensor, walls, waypoints, firstUpdate
    reload(behavior)
    reload(astar)
    reload(pf)
    reload(intersect)
    clearDict()
    #setRunning(True)
    requestSensor("position")
    requestSensor("rotation")
    requestSensor("laser_vl")
    requestSensor("laser_vr")
    requestSensor("laser_hl")
    requestSensor("laser_hr")
    requestSensor("wheels")
    requestConfig("VirtualJoystick", "x")
    requestConfig("VirtualJoystick", "y")
    
    setConfig("Graphics", "showCoords", 0)
    setConfig("Scene", "skydome_enabled", 1)
    setConfig("Simulator", "calc_ms", 40)
    setUpdateTime(40)

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
    firstUpdate = True
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
    wheels[0] = marsData["Sensors"]["wheels"][0]*0.28 #wheel radian plus small damping
    wheels[1] = marsData["Sensors"]["wheels"][1]*0.28
    pos = getVector3(marsData["Sensors"]["position"])
    direction = math.radians(getVector3(marsData["Sensors"]["rotation"]).z)

def update(marsData):
    global distance, t, pointCloudData, walls, numParticles, wheels
    global firstUpdate

    clearDict()
    if firstUpdate:
        firstUpdate = False
        behavior.initBehavior(waypoints, walls, pointCloudData,
                              numParticles)
    readSensor(marsData)

    joystickLeft = marsData["Config"]["VirtualJoystick"]["x"]
    joystickRight = marsData["Config"]["VirtualJoystick"]["y"]
    

    behavior.doBehavior(distance, joystickLeft, joystickRight,
                        wheels, marsData)

    # pf.generate_measurement(np.array([pos[0], pos[1],
    #                                   direction, 1.]),
    #                         walls, True, 2)

    setMotor("motor_links", behavior.left_actuator)
    setMotor("motor_rechts", behavior.right_actuator)
    return sendDict()
