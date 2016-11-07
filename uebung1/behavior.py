from mars_interface import *
import random

random.seed()

right_actuator = 0.
left_actuator = 0.
right = 0
left = 0

def randomWalk(distance):
    global right_actuator
    global left_actuator
    global right
    global left

    if distance[7] < 0.3 and distance[5] < 0.3:
        right_actuator = -10.;
    elif right < 0:
        right_actuator = random.random()*10.
        right = 10
    else:
        right -= 1

    if distance[2] < 0.3 or distance[0] < 0.3:
        left_actuator = -12.;
    elif left < 0:
        left_actuator = random.random()*10.
        left = 10
    else:
        left -= 1
    
def turn(direction):
    global right_actuator
    global left_actuator
    left_actuator = direction * 1.
    right_actuator = -direction * 1.
    
def driveForward(distance):
    global right_actuator
    global left_actuator
    left_actuator += distance
    right_actuator += distance
        
def sensorFeedback(light):
    sLeft = 0.
    sRight = 0.
    for i in range(4):
        if light[i] > sLeft:
            sLeft = light[i]
    for j in range(4,8):
        if light[j] > sRight:
            sRight = light[j]
    return (sLeft , sRight)
        
def love(light):
    global right_actuator
    global left_actuator
    sensors = sensorFeedback(light)
    logMessage(str(sensors))
    left_actuator  = (.9 - sensors[0]) * (.9 - sensors[0]) * 10
    right_actuator = (.9 - sensors[1]) * (.9     - sensors[1]) * 10
    logMessage(str(left_actuator) + ":" + str(right_actuator))
    
def hate(light):
    global right_actuator
    global left_actuator
    sensors = sensorFeedback(light)
    logMessage(str(sensors))
    left_actuator  = 10 + ((sensors[1]) * 25) * ((sensors[1]) * 25)
    right_actuator = 10 + ((sensors[0]) * 25) * ((sensors[0]) * 25)
    logMessage(str(left_actuator) + ":" + str(right_actuator))
    
    
def feeeeear(light):
    global right_actuator
    global left_actuator
    sensors = sensorFeedback(light)
    logMessage(str(sensors))
    left_actuator  = 1 - 10 * sensors[1]
    right_actuator = 1 - 10 * sensors[0]
    logMessage(str(left_actuator) + ":" + str(right_actuator))    
    
    
def neugear(light):
    global right_actuator
    global left_actuator
    sensors = sensorFeedback(light)
    logMessage(str(sensors))
    left_actuator  = 5 + 10 * (1 - sensors[1])
    right_actuator = 5 + 10 * (1 - sensors[0])
    logMessage(str(left_actuator) + ":" + str(right_actuator))    
    
def illegalLove(light):
    i = 0
    for v in light:
        if v > 0:
            had_light = 1
            if v > light_max:
                light_max = v
                light_position = i
        i += 1
    if had_light == 1:
        logMessage("Have Light: " + str(light_position) + ", intensity: " + str(light_max))
        if light_max < .9:
            turn(light_position - 4)
            driveForward(10-(light_max*10))
        else:
            turn(0)
    
def doBehavior(distance, light, marsData):
    global right_actuator, left_actuator

    behavior = marsData["Config"]["Robotik2"]["behavior"]
    had_light = 0
    light_position = -1
    light_max = 0
    if behavior == 1:
        love(light)
    if behavior == 34:
        feeeeear(light)
    if behavior == 6:
        hate(light)
    if behavior == 0:
        neugear(light)
    if behavior == 5:
        randomWalk(distance)
