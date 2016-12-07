from mars_interface import *
import random


random.seed()

lcam = []
rcam = []
#threshold for diameter that is considered to be close. Value distinguished by trial and error
diameterTh = 25

#coordinates for min and max coordinate of red pixel, used a diameter
minX = 10000
maxX = -10000
state = "pt"
width = 160
height = 120

right_actuator = 0.
left_actuator = 0.
right = 0
left = 0

#execute counterclockwise pointturn
def doPointturn():
    global right_actuator
    global left_actuator
    right_actuator = 2.0
    left_actuator = -2.0

#approaches the target detected in a frame. need a tuple consiting of (frame as string(eg "left", "right"), pixelPosition which is set aroung 0 as center)
def approach(position):
    global right_actuator
    global left_actuator
    if(position[0] == "left"):
        right_actuator = 6.0
        left_actuator = 4.0
    if(position[0] == "right"):
        right_actuator = 5.0
        left_actuator = 5.0 + float(position[1]) / float(width)*5
        #print("left actuator: " + str(left_actuator) + ", right actuator: " + str(right_actuator))

#checks for red stuff in cameraframe
#could be modified, e.g. giving the color to search for as a parameter
def checkForRed():
    result = trackBall()
    return result[0] != "none"

#calculates the diameter
def diameter(minx, maxx):
    return (maxx - minx)

#checks wether or not a red pixel has been found. return tuple of (left or right frame as string, vertical pixelposition where center = 0)
#also counts number of red pixels
#could be extended so that the center of mass is returned
def trackBall():
    global lcam, rcam
    global minX,maxX
    redPix = 0
    red = False
    for i in range(len(lcam)):
        if lcam[i][0] >= 200 and lcam[i][1] <= 100 and lcam[i][2] <= 100:
            #move to zero as center
            approx_vert = i % width - width/2
            if(approx_vert < minX):
                minX = approx_vert
            if(approx_vert > maxX):
                maxX = approx_vert
            red = True
    if(red):
            return ("left", approx_vert)
    for i in range(len(rcam)):
        if rcam[i][0] >= 200 and rcam[i][1] <= 100 and rcam[i][2] <= 100:
            approx_vert = i % width - width/2
            if(approx_vert < minX):
                minX = approx_vert
            if(approx_vert > maxX):
                maxX = approx_vert
            red = True
    if(red):
            return ("right", approx_vert)
    return ("none", -1)

def circle():
    global left_actuator, right_actuator
    left_actuator = 0.
    right_actuator = 0.

#simple state machine executing subfunctions
def execute():
    global state
    global t
    global right_actuator, left_actuator
    global maxX,minX
    minX = 10000
    maxX = -10000

    readData()
    if(state == "pt"):
        doPointturn()
        if(checkForRed() == True):
            print("switching to state approach, checkforred is true")
            state = "approach"
            return
    if(state == "approach"):
        position = trackBall()
        print(diameter(minX, maxX))
        if(diameter(minX, maxX) >= diameterTh):
            print("now too close! switching to state circle")
            state = "circle"
            return
        if(position[0] != "none"):
            approach(position)
        else:
            print("switching to state pt, position equals none")
            state = "pt"
            return
    if(state == "circle"):
        circle()

#def readData(camData):
#    global lcam, rcam
#    width = 160
#    height = 120
#    for y in range(height):
#        yy = height-1-y
#        for x in range(width):
#            coord = yy*width*4+x*4
#            r = camData["cam0"][coord]*255
#            g = camData["cam0"][coord+1]*255
#            b = camData["cam0"][coord+2]*255
#            lcam.append([r,g,b])
#            r = camData["cam1"][coord]*255
#            g = camData["cam1"][coord+1]*255
#            b = camData["cam1"][coord+2]*255
#            rcam.append([r,g,b])

def readData():
    global lcam, rcam
    with open('cam1.ppm') as pic:
        data = pic.read()
    lcam_split = data.split()
    r = map(int, lcam_split[4::3])
    g = map(int, lcam_split[5::3])
    b = map(int, lcam_split[6::3])
    lcam = zip(*[r, g, b])

    with open('cam0.ppm') as pic:
        data = pic.read()
    rcam_split = data.split()
    r = map(int, rcam_split[4::3])
    g = map(int, rcam_split[5::3])
    b = map(int, rcam_split[6::3])
    rcam = zip(*[r, g, b])

def doBehavior():
    global right_actuator, left_actuator

    #behavior = marsData["Config"]["Robotik2"]["behavior"]

    execute()

    #randomWalk(distance)

    # if timing(1):
    #     message = "sensor:"
    #     for s in light:
    #         message += " " + str(s)
    #     logMessage(message)
