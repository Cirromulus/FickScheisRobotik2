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
    #print "length particle: " + str(len(particles))
    return np.sum(np.square(np.subtract(p1, p2)))

#bitte diese funktion nutzen um die Anzahl der Partikel zu setzen
#(intitialisiert indern das array um die Partikel zu zeichnen)
def getNumberOfParticles():
    return 150

def initParticles():
    magicDistances = 7
    global particles
    data = pointCloudData["particles"]
    for i in range(getNumberOfParticles()):
        part = State(random.uniform(-magicDistances,magicDistances), random.uniform(-magicDistances,magicDistances), 0., random.uniform(-np.pi, np.pi), 1./getNumberOfParticles())
        #part = State(data[i*3], data[i*3+1], data[i*3+2], 0.0, 1/getNumberOfParticles())
        particles.append(part)
    particles[0] = State(0,0,0,0,1./getNumberOfParticles())
    #print "initialized"

#import heapq
def particleFilter(particles, control, measurement):
    newParticles = []
    resultParticles = []
    for i in range(len(particles)):
        p = sampleParticle(particles[i], control)
        p.weight = calcProb(probMeasurement((p.x,p.y),p.rot),measurement) * p.weight
        #if(p.weight > .50):
            #print( "p: %.2f, %.2f, %.2f" %  (p.x, p.y,p.weight))
        newParticles.append(p)
    weightSum = 0.
    weightMax = 0.
    for p in newParticles:
        if(p.weight > weightMax):
            weightMax = p.weight
        weightSum += p.weight

    #resultParticles.extend(heapq.nlargest(len(particles)/2, newParticles, key=lambda p: p.weight))
    for i in range(getNumberOfParticles()):
        rnd = np.random.uniform(0, weightSum)
        ws = 0
        for p in newParticles:
            ws += p.weight
            if ws >= rnd:
                nutte = State(p.x, p.y, p.z, p.rot, p.weight)
                nutte.weight /= weightMax
                resultParticles.append(nutte)
                break

    return resultParticles



def rot2d(ar, direction):
    rotMatrix = np.array([[np.cos(direction), -np.sin(direction)],
                          [np.sin(direction),  np.cos(direction)]])
    return np.dot(ar, rotMatrix)

def rot3d(ar, direction):
    rotMatrix = np.array([[np.cos(direction), -np.sin(direction), 0],
                          [np.sin(direction),  np.cos(direction), 0],
                          [0, 0, 1]])
    return np.dot(ar, rotMatrix)

def sampleParticle(particle, control):
    global particles
    megaMagicNoiseFactor = 5 + 0 * (1 - particle.weight)
    noiseFactorl = 1 + abs(control[0] * megaMagicNoiseFactor)
    noiseFactorr = 1 + abs(control[1] * megaMagicNoiseFactor)
    left = control[0]
    right = control[1]
    if(noiseFactorl > 0):
        left = np.random.normal(control[0], noiseFactorl)
    if(noiseFactorr > 0):
        right = np.random.normal(control[1], noiseFactorr)
    updateTime = 0.0175
    diam = 0.3
    width= 0.54
    circ = diam * np.pi
    rang = [left / (2 * np.pi * circ) * updateTime, right / (2 * np.pi * circ) * updateTime]

    dist = (rang[0] + rang[1])/2
    angle = (rang[1] - rang[0]) / float(width)
    particle.x += dist * math.cos(particle.rot)
    particle.y += dist * math.sin(particle.rot)
    particle.rot += angle
    #print angle
    particle.rot %= (2 * np.pi)
    if particle.rot < -np.pi:
        particle.rot += 2 * np.pi
    elif particle.rot > np.pi:
        particle.rot -= 2*np.pi
    #add random action

    return particle


def probMeasurement(position, rotation):
    global laserRange
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

    laser = np.array([0, laserRange])
    targetpoint = [np.array(rot2d(laser, i - rotation)) for i in baseRotz]  #45fucks
    multiLaserTargetPoints = []
    for elem in targetpoint:
        multiLaserTargetPoints.append([np.array(rot2d(elem, offs)) for offs in laserWinkelOffsets])
    #print multiLaserTargetPoints
    multiLaserTargetPointsTranse = []
    for  i in range(len(basePos)):
        multiLaserTargetPointsTranse.append([sub+basePos[i] for sub in multiLaserTargetPoints[i]])
    # print multiLaserTargetPointsTranse
    #clearLines("path")
    #configureLines("path", 2, 0.2, 0.8, 0.2)
    #for i in range(len(multiLaserTargetPointsTranse)):
    #   for j in range(len(multiLaserTargetPointsTranse[i])):
    #      appendLines("path", basePos[i][0], basePos[i][1], 0.5)
    #       appendLines("path", multiLaserTargetPointsTranse[i][j][0], multiLaserTargetPointsTranse[i][j][1], 0.5)
    #       appendLines("path", basePos[i][0], basePos[i][1], 0.5)
    calced = []
    for i in range(len(multiLaserTargetPointsTranse)):
        for j in range(len(multiLaserTargetPointsTranse[i])):
            dist = checkDistance(basePos[i], multiLaserTargetPointsTranse[i][j])
            if dist > 1:
                calced.append(laserRange)
            else:
                calced.append(dist * laserRange)
    return calced


def calcProb(list1, list2):
    global laserRange
    err2 = sum((np.array(list1) - np.array(list2))**2)
    return (1 - err2 / ((laserRange**2)*len(list1)))


def doBehavior(distance, direction, pos, marsData, waypoints, walls, joystickLeft, joystickRight):
    global init
    global particles
    # global position
    global laserRange
    laserLength = 4.

    flobz = np.array([-2, 0, 0])
    rotMatrix = np.array([[np.cos(direction), -np.sin(direction), 0],
                          [np.sin(direction),  np.cos(direction), 0],
                          [0, 0, 1]])
    gurps = np.dot(flobz, rotMatrix)

    bla = checkDistance(pos, pos + gurps)
    #print "Pohs" + str(pos)
    #print "dist" + str(bla)
    #print "Dirce" + str(direction)
    if not init:
        initParticles()
        init = True
        laserRange = 4.
        # position = (pos, direction)

    # p = State(position[0][0], position[0][1], position[0][2], position[1], 1)
    #
    # p = sampleParticle(p, (joystickLeft,joystickRight))
    #0
    # position = (pos, direction)

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



    #print "Joystick input: (" + str(joystickLeft) + " : " +str(joystickRight) +")" + " direction: " + str(direction)
    #
    # print "Sensors:"
    # # jeweils gegen den Uhrzeigersinn, wenn man von oben guckt
    # print "fl (%.2f, %.2f, %.2f, %.2f)" % (distance[0], distance[1],distance[2],distance[3])
    # print "fr (%.2f, %.2f, %.2f, %.2f)" % (distance[4], distance[5],distance[6],distance[7])
    # print "bl (%.2f, %.2f, %.2f, %.2f)" % (distance[8], distance[9],distance[10],distance[11])
    # print "br (%.2f, %.2f, %.2f, %.2f)" % (distance[12], distance[13],distance[13],distance[15])
    # print "Calced: "
    # calc = probMeasurement(pos, direction)
    # prob = calcProb(calc, distance)

    #Laser Mitte ist jeweils 45 deg verdreht
    #Winkeloffset zwischen den Laserstrahlen: 0.349 rad

    #Es kann ein Array von punkten zurueck gegeben werden, diese werden in the 3D Szene gezeichnet
    clearLines("path")
    configureLines("path", 2, 0.2, 0.8, 0.2)
    particles = particleFilter(particles, (joystickLeft, joystickRight), distance)

    points = []
    probPosition = [0.0, 0.0]
    meanX = 0.
    meanY = 0.
    for p in particles:
        points.append([p.x, p.y])
        meanX += p.x
        meanY += p.y
    probPosition = [meanX/len(particles), meanY/len(particles)]

    print "real pos: " + str(pos)
    print "estimated pos: " + str(probPosition)

    #necessary for drawing a single point
    points[int(random.uniform(0,len(points)-1))] = [-8,-7]
    #logMessage(str(len(points)) + " Points: " + str(points[0]) + str(particles[0].weight))
    return points
