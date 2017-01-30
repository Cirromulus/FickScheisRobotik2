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
     global waypoints, walls
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

def runAstar():
     global path, waypoints, walls, updatePath

     updatePath = False
     path = astar.run(waypoints, walls, np.array([0., 0.]),
                      np.array([4., 5.]))
     clearLines("path")
     configureLines("path", 5, 0.8, 0.6, 0.2)
     for i in range(len(path)-1):
          appendLines("path", path[i][0], path[i][1], 0.45)
          appendLines("path", path[i+1][0], path[i+1][1], 0.45)

def runParticleFilter(distance, wheels):
    global walls, pointCloudData, numParticles, bestP
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

    #vMeasure = pf.generate_measurement(bestP, walls, True, 1)

    clearLines("guess")
    appendLines("guess", bestP[0], bestP[1], 0.5)
    appendLines("guess", p2[0], p2[1], 0.5)

def doBehavior(distance, joystickLeft, joystickRight, wheels,
               marsData):
    global updatePath, left_actuator, right_actuator, bestP

    if updatePath:
         runAstar()

    runParticleFilter(distance, wheels)

    left_actuator = joystickLeft
    right_actuator = joystickRight
