from mars_interface import *
import random


random.seed()

lcam = []
rcam = []
#threshold for diameter that is considered to be close. Value distinguished by trial and error
diameterTh = 25.
speed = 8.

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
    right_actuator = speed/2.
    left_actuator = -speed/2.

#approaches the target detected in a frame. need a tuple consiting of (frame as string(eg "left", "right"), pixelPosition which is set aroung 0 as center)
def approach(position):
    global right_actuator
    global left_actuator
    if(position[0] == "left"):
        right_actuator = speed
        left_actuator = speed/2
    if(position[0] == "right"):
        right_actuator = speed
        left_actuator = speed + float(position[1]) / float(width)*speed
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
        if lcam[i][0] >= 200. and lcam[i][1] <= 100. and lcam[i][2] <= 100.:
            #move to zero as center
            approx_vert = i % width - width/2.
            if(approx_vert < minX):
                minX = approx_vert
            if(approx_vert > maxX):
                maxX = approx_vert
            red = True
    if(red):
            return ("left", approx_vert)
    for i in range(len(rcam)):
        if rcam[i][0] >= 200. and rcam[i][1] <= 100. and rcam[i][2] <= 100.:
            approx_vert = i % width - width/2.
            if(approx_vert < minX):
                minX = approx_vert
            if(approx_vert > maxX):
                maxX = approx_vert
            red = True
    if(red):
            return ("right", approx_vert)
    return ("none", -1)

#circles around the tracked object
def circle():
    global left_actuator, right_actuator
    global minX, maxX
    global state
    logMessage("Circling around object!")

    bawl = trackBall()

    left_actuator = 1.
    right_actuator = 0.

    if(bawl[0] == "left"):
        factor = -.15   #'magic'
        factor += (bawl[1] / (width / 2.)) * speed
        #logMessage("Factor: " + str(factor))
        if(diameter(minX, maxX) < diameterTh):
            logMessage("Too far away, turning inside a bit")
            factor -= 1.

        left_actuator  = speed / 2. + factor
        right_actuator = speed / 2. - factor
    return True

    if(bawl[0] == "none"):
        return False

#simple state machine executing subfunctions
def execute(camData):
    global state
    global t
    global right_actuator, left_actuator
    global maxX,minX
    minX = 10000
    maxX = -10000

    readData(camData)
    if(state == "pt"):
        doPointturn()
        if(checkForRed() == True):
            logMessage("switching to state approach, checkforred is true")
            state = "approach"
            return
    if(state == "approach"):
        position = trackBall()
        #print(diameter(minX, maxX))
        if(diameter(minX, maxX) >= diameterTh):
            logMessage("now too close! switching to state circle")
            state = "circle"
            return
        if(position[0] != "none"):
            approach(position)
        else:
            logMessage("switching to state pt, position equals none")
            state = "pt"
            return
    if(state == "circle"):
        stillCircling = circle()
        if(!stillCircling):
            state = "pt"

#reads imagedata from cam array just like in mars_plugin.py
def readData(camData):
    global lcam, rcam
    lcam = []
    rcam = []
    width = 160
    height = 120
    for y in range(height):
        yy = height-1-y
        for x in range(width):
            #coord = yy*width*4+x*4
            r = (int)(camData["cam0"][yy*width*4+x*4]*255)
            g = (int)(camData["cam0"][yy*width*4+x*4+1]*255)
            b = (int)(camData["cam0"][yy*width*4+x*4+2]*255)
            rcam.append([r,g,b])
            r = (int)(camData["cam1"][yy*width*4+x*4]*255)
            g = (int)(camData["cam1"][yy*width*4+x*4+1]*255)
            b = (int)(camData["cam1"][yy*width*4+x*4+2]*255)
            lcam.append([r,g,b])
