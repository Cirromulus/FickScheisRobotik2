from mars_interface import *
import astar
import particlefilter as pf
import numpy as np
import math

right_actuator = 0.
left_actuator = 0.
walls = None
waypoints = None
path = None
pointCloudData = {}
updatePath = True
numParticles = 100
bestP = None

def initBehavior(waypoints_, walls_, pointCloudData_,
                 numParticles_):
     global waypoints, walls, averageP
     waypoints = waypoints_
     walls = walls_
     pointCloudData = pointCloudData_
     numParticles = numParticles_
     updatePath = True
     configureLines("guess", 5, 0.2, 1.0, 0.2)
     configureLines("debugRays", 2, 0.5, 0.4, 1.0)
     configureLines("debugRays2", 2, 0.5, 1.0, 0.4)
     configurePointCloud("waypoints", 8.0, 0.2, 0.6, 0.8)
     configurePointCloud("particles", 4.0, 0.9, 0.3, 0.9)
     # init particle filter
     pf.init(numParticles, 3,
             np.array([-0.5, -0.5, -0.1]),
             np.array([0.5, 0.5, 0.1]))
     averageP = np.array([0.,0.,0.,0.])

     print(angle_between(np.array([1,0]), np.array([0,1])) * (180/np.pi))
     print(angle_between(np.array([0,1]), np.array([1,0])) * (180/np.pi))

def runAstar():
     global path, waypoints, walls, updatePath, averageP
     updatePath = False
     path = astar.run(waypoints, walls, np.array([averageP[0], averageP[1]]),
                      np.array([4., 5.]))
     clearLines("path")
     configureLines("path", 5, 0.8, 0.6, 0.2)
     for i in range(len(path)-1):
          appendLines("path", path[i][0], path[i][1], 0.45)
          appendLines("path", path[i+1][0], path[i+1][1], 0.45)

def runParticleFilter(distance, wheels):
    global walls, pointCloudData, numParticles, bestP, averageP
    pf.sample_and_weight(walls, distance, wheels)

    # update particles in mars
    pData = pointCloudData["particles"]
    for i in range(numParticles):
         pData[i*3] = pf.population[i][0]
         pData[i*3+1] = pf.population[i][1]

    pf.select_best_trunc()
    clearLines("debugRays")
    clearLines("debugRays2")

    bestP = pf.population[0]
    p2 = np.array([math.cos(bestP[2]), math.sin(bestP[2])])
    np.multiply(p2, 0.2, p2)
    np.add(bestP[:2], p2, p2)

    averageP = (bestP + averageP * 8.) / 9.
    #vMeasure = pf.generate_measurement(bestP, walls, True, 1)

    clearLines("guess")
    appendLines("guess", averageP[0], averageP[1], 0.8)
    appendLines("guess", bestP[0], bestP[1], 0.5)
    appendLines("guess", p2[0], p2[1], 0.5)

def obstacleAvoidance(intention):
    if(False):
        #Avoid obstacle
        return (-1., -1.)
    else:
        return intention


def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::"""

    return math.atan2(v1[1], v1[0]) - math.atan2(v2[1], v2[0])

def normAngle(angle):
    angle %= (2 * np.pi)
    if angle < -np.pi:
        angle += 2 * np.pi
    elif angle > np.pi:
        angle -= 2*np.pi
    return angle

def autonomousDrive():
    global path, averageP, updatePath
    #mega KI thinking
    nextTarget = path[-2]
    print("nextTarget: "+ str(nextTarget))
    diff = nextTarget - np.array([averageP[0], averageP[1]])
    print("self: " +str([averageP[0], averageP[1]]))
    print("diff: "+ str(diff) + " ("+ str(np.linalg.norm(diff)) +")")
    vorne = np.array([1,0])
    selfRot = averageP[3]
    rotMatrix = np.array([[np.cos(selfRot), -np.sin(selfRot)],
                         [np.sin(selfRot),  np.cos(selfRot)]])
    vorne = rotMatrix.dot(vorne)
    angleDiff = normAngle(angle_between(vorne,diff))

    if(np.linalg.norm(diff) < .08):
        print("Assume to have been arrived at waypoint")
        print("Waypoints to go: " + str(len(path)-2))
        if(len(path) <= 2):
            print("arrived at destination")
            return (0.,0.)
        updatePath = True

    print("Angle to Waypoint: " + str(angleDiff*(180/np.pi)) + "deg")
    if(abs(angleDiff) < .05 or np.linalg.norm(diff) < .9):
        return (1.8,1.8)
    else:
        return (np.sign(angleDiff) * .5, np.sign(angleDiff) *-.5)

def doBehavior(distance, joystickLeft, joystickRight, wheels,
               marsData, pos, direction):
    global updatePath, left_actuator, right_actuator, averageP

    if updatePath:
         runAstar()

    #runParticleFilter(distance, wheels)

    xdiff = pos[0] - averageP[0]
    ydiff = pos[1] - averageP[1]
    adiff = direction - averageP[3]
    print("Diff of belief: x: " + str(xdiff) + ", y: " + str(ydiff) + ", r: " + str(adiff))
    averageP = np.array([pos[0], pos[1], pos[2], direction])


    dreiv = autonomousDrive()
    dreiv = obstacleAvoidance(dreiv)
    if(abs(joystickLeft) > .5 and abs(joystickRight) > .5):
        dreiv = (joystickLeft, joystickRight)

    left_actuator = dreiv[0]
    right_actuator = dreiv[1]
