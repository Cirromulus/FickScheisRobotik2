from mars_interface import *
from mars_plugin import *
import random
import intersect
import math
import numpy as np

particles = []
init = False

class State:
    x = 0.
    y = 0.
    z = 0.
    rot = 0.
    weight = 0.

    def __init__(self,x,y,z,rot, weight):
        self.x = x
        self.y = y
        self.z = z
        self.rot = rot
        self.weight = weight

def checkIntersect(pos, wp):
    # for wall in walls:
    #     if intersect.doIntersect(line, wall):
    #         return False
    if intersect.get_intersect(pos[0], pos[1], wp[0], wp[1]) < 1:
       return False
    return True

def checkDistance(pos, wp):
    #print "lines: " + str(pos) + ", " + str(wp)
    return intersect.get_intersect(pos[0], pos[1], wp[0], wp[1])

def dist(p1, p2):
    return np.sum(np.square(np.subtract(p1, p2)))

#bitte diese funktion nutzen um die Anzahl der Partikel zu setzen
#(intitialisiert indern das array um die Partikel zu zeichnen)
def getNumberOfParticles():
    return 100

def initParticles():
    global particles
    data = pointCloudData["particles"]
    for i in range(getNumberOfParticles()):
        part = State(data[i*3], data[i*3+1], data[i*3+2], 0.0, 1/getNumberOfParticles())
        particles.append(part)
    print "initialized"


def particleFilter(particles, control, measurement):
    newParticles = []
    for i in range(len(particles)):
        newParticles.append(particles[i] + control)

def rot2d(ar, direction):
    rotMatrix = np.array([[np.cos(direction), -np.sin(direction)],
                          [np.sin(direction),  np.cos(direction)]])
    return np.dot(ar, rotMatrix)

def rot3d(ar, direction):
    rotMatrix = np.array([[np.cos(direction), -np.sin(direction), 0],
                          [np.sin(direction),  np.cos(direction), 0],
                          [0, 0, 1]])
    return np.dot(ar, rotMatrix)

def probMeasurement(position, rotation):
    # Laser scanner positions fl, fr, bl, br
    basePos = np.array([[ 0.320234,  0.230233],
                        [ 0.320234, -0.230234],
                        [-0.306395,  0.214315],
                        [-0.306395, -0.214316]])

    baseRotz = np.array([0.785, 3*0.785, -0.785, -3*0.785])
    directions = np.array([0.262, -1.309, 1.833, -2.879])
    magicWinkel_offset = 0.349
    laserWinkelOffsets = np.array([magicWinkel_offset / 2 + magicWinkel_offset, magicWinkel_offset / 2, -magicWinkel_offset/2, -(magicWinkel_offset/2+magicWinkel_offset)])

    basePos = [rot2d(bp, -rotation) for bp in basePos]
    basePos = [np.array(bp) + np.array([position[0], position[1]]) for bp in basePos]

    laser = np.array([0, 4])
    targetpoint = [np.array(rot2d(laser, i - rotation)) for i in baseRotz]  #45fucks
    multiLaserTargetPoints = []
    for elem in targetpoint:
        multiLaserTargetPoints.append([np.array(rot2d(elem, offs)) for offs in laserWinkelOffsets])
    #print multiLaserTargetPoints
    multiLaserTargetPointsTranse = []
    for  i in range(len(basePos)):
        multiLaserTargetPointsTranse.append([sub+basePos[i] for sub in multiLaserTargetPoints[i]])
    #print multiLaserTargetPointsTranse
    clearLines("path")
    configureLines("path", 2, 0.2, 0.8, 0.2)
    for i in range(len(multiLaserTargetPointsTranse)):
        for j in range(len(multiLaserTargetPointsTranse[i])):
            appendLines("path", basePos[i][0], basePos[i][1], 0.5)
            appendLines("path", multiLaserTargetPointsTranse[i][j][0], multiLaserTargetPointsTranse[i][j][1], 0.5)
            appendLines("path", basePos[i][0], basePos[i][1], 0.5)
    return [[checkDistance(basePos[i], multiLaserTargetPointsTranse[i][j])*4 for j in range(len(multiLaserTargetPointsTranse[i]))] for i in range(len(multiLaserTargetPointsTranse))]



def doBehavior(distance, direction, pos, marsData, waypoints, walls, joystickLeft, joystickRight):
    global init

    flobz = np.array([-2, 0, 0])
    rotMatrix = np.array([[np.cos(direction), -np.sin(direction), 0],
                          [np.sin(direction),  np.cos(direction), 0],
                          [0, 0, 1]])
    gurps = np.dot(flobz, rotMatrix)

    bla = checkDistance(pos, pos + gurps)
    print "Pohs" + str(pos)
    print "dist" + str(bla)
    print "Dirce" + str(direction)
    if not init:
        initParticles()
        init = True

    #pos darf nut zur Kontrolle benutzt werden
    #print "Real Position: " + str(pos[0]) + ":" + str(pos[1])


    #print "using %d particles" % getNumberOfParticles()

    #addPointCloudData("particles",points)
    #pData = pointCloudData["particles"]



    #clearLines("path")
    #configureLines("path", 5, 0.2, 0.8, 0.2)

    # for wp in waypoints:
    #     if checkIntersect(pos,wp):
    #          appendLines("path", pos[0], pos[1], 0.5)
    #          appendLines("path", wp[0], wp[1], 0.5)



    print "Joystick input: (" + str(joystickLeft) + " : " +str(joystickRight) +")" + " direction: " + str(direction)

    #print "Sensors:"
    #jeweils gegen den Uhrzeigersinn, wenn man von oben guckt
    #print "fl (%.2f, %.2f, %.2f, %.2f)" % (distance[0], distance[1],distance[2],distance[3])
    #print "fr (%.2f, %.2f, %.2f, %.2f)" % (distance[4], distance[5],distance[6],distance[7])
    #print "bl (%.2f, %.2f, %.2f, %.2f)" % (distance[8], distance[9],distance[10],distance[11])
    #print "br (%.2f, %.2f, %.2f, %.2f)" % (distance[12], distance[13],distance[13],distance[15])
    #print "Calced: "
    #calc = probMeasurement(pos, direction)
    #for i in calc:
    #    print i

    #Laser Mitte ist jeweils 45 deg verdreht
    #Winkeloffset zwischen den Laserstrahlen: 0.349 rad

    points = []

    points.append([1,1])
    points.append([2,2])

    #Es kann ein Array von punkten zurueck gegeben werden, diese werden in the 3D Szene gezeichnet
    return points
