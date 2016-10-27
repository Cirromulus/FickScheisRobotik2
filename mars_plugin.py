from mars_interface import *
import behavior
import math
from euclid import *

light_pos = []
light_pos.append(Vector3(3.0, 0.0, 0.1))
light_sensor = []


def init():
    global light_sensor
    reload(behavior)
    clearDict()
    rad = 0.97738438112
    light_sensor = []
    for i in range(8):
        q = Quaternion.new_rotate_axis(rad, Vector3(0, 0, 1));
        light_sensor.append(q*Vector3(1, 0, 0))
        rad -= 0.27925268032
    setRunning(True)
    requestSensor("position")
    requestSensor("rotation")
    requestSensor("laser")
    requestSensor("sensor_position")
    setConfig("Graphics", "showCoords", 0)
    setConfig("Scene", "skydome_enabled", 1)
    setConfig("Simulator", "calc_ms", 40)
    setConfig("Robotik2", "behavior", 0)
    requestConfig("Robotik2", "behavior")
    return sendDict()

def getVector3(myList):
    v = Vector3()
    if len(myList) == 3:
        v = Vector3(myList[0], myList[1], myList[2])
    return v

def update(marsData):
    #if "Nodes" in marsData:
        #print str(marsData["Nodes"]["body"]["pos"])
    clearDict()
    distance = [0, 0, 0, 0, 0, 0, 0, 0]
    light = [0., 0., 0., 0., 0., 0., 0., 0.]
    sensor_pos = []
    q = Quaternion()
    v = Vector3()
    direction = 0
    if "Sensors" in marsData:
        if "laser" in marsData["Sensors"]:
            if len(marsData["Sensors"]["laser"]) == 8:
                for i in range(8):
                    distance[i] = marsData["Sensors"]["laser"][i]
        if "position" in marsData["Sensors"]:
            v = getVector3(marsData["Sensors"]["position"])
        if "rotation" in marsData["Sensors"]:
            r = getVector3(marsData["Sensors"]["rotation"])
            direction = math.radians(r.z)
            q = Quaternion.new_rotate_axis(direction, Vector3(0, 0, 1));
        if "sensor_position" in marsData["Sensors"]:
            p = []
            for data in marsData["Sensors"]["sensor_position"]:
                p.append(data)
            for i in range(8):
                sensor_pos.append(Vector3(p[i*3], p[i*3+1], p[i*3+2]))

    for i in range(8):
        for lightPos in light_pos:
            lightSensor = q * light_sensor[i]
            v = lightPos - sensor_pos[i]
            l = abs(v)
            if l < 2:
                angle = math.fabs(lightSensor.angle(v))
                if angle < 1.04:
                    l = (2-l)*0.5
                    l = math.cos(angle*1.5)*l
                    light[i] += l

    behavior.doBehavior(distance, light, marsData)
    setMotor("motor_left", -behavior.left_actuator)
    setMotor("motor_right", -behavior.right_actuator)
    return sendDict()
