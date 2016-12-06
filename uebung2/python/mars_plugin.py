from mars_interface import *
import behavior
import math
from euclid import *
from time import clock

light_pos = []
light_pos.append(Vector3(3.0, 0.0, 0.1))
light_sensor = []
cameraData = {}
cameraSize = {}
start_times = {}

camera_names = ["cam0", "cam1"]#, "cam2", "cam3"]

def elapsed_time(timer_name, s):
    current_time = clock()
    if current_time > start_times[timer_name]+s:
        start_times[timer_name] = clock()
        return True
    return False

def writeCameraToFile(cam_name):
    global cameraData, cameraSize
    width = 160
    height = 120
    if cameraSize[cam_name] == height*width:
        with open(cam_name+".ppm", "w") as f:
            f.write("P3\n")
            f.write(str(width) + " "+ str(height) + "\n")
            f.write("255\n")
            for y in range(height):
                yy = height-1-y
                for x in range(width):
                    f.write(str(int(cameraData[cam_name][yy*width*4+x*4]*255))+ " ")
                    f.write(str(int(cameraData[cam_name][yy*width*4+x*4+1]*255))+ " ")
                    f.write(str(int(cameraData[cam_name][yy*width*4+x*4+2]*255))+ " ")
                f.write("\n")

def addCameraData(name, data):
# What is here the name for?
    global cameraData, cameraSize
    cameraData[name] = data
    cameraSize[name] = len(data)/4

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
    #requestSensor("laser")
    requestSensor("sensor_position")
    setConfig("Graphics", "showCoords", 0)
    setConfig("Scene", "skydome_enabled", 1)
    #setConfig("Simulator", "calc_ms", 20)
    setConfig("Simulator", "calc_ms", 10)
    setUpdateTime(40)
    #setUpdateTime(10)
    setConfig("Robotik2", "behavior", 0)
    requestConfig("Robotik2", "behavior")
    for camera_name in camera_names:
        requestCameraSensor(camera_name)
        start_times[camera_name] = clock()
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

    behavior.doBehavior()
    setMotor("motor_left", - behavior.left_actuator)
    setMotor("motor_right", - behavior.right_actuator)
    #setMotor("motor_left", -0.2)
    #setMotor("motor_right", -0.2)
    for camera_name in camera_names:
        requestCameraSensor(camera_name)
        if elapsed_time(camera_name, 1.):
            writeCameraToFile(camera_name)
    return sendDict()
