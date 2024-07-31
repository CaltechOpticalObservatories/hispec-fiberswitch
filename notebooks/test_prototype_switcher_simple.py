
import os
import socket
from TLPM import TLPM
import threading
import numpy as np
from astropy.io import fits
from ctypes import c_int16, c_uint32, byref, create_string_buffer, c_bool, c_char_p, c_double
import time
from datetime import datetime
import matplotlib.pyplot as plt
import serial
import cv2
import minimalmodbus

#### GLOBAL VARIABLES ####
threshhold = 0.0008 #INPUT MIN ACCEPTABLE POWER
maxcleannum = 5
Camport = 1
Blowport = 5

CurrPath = os.path.dirname(os.path.realpath(__file__)) + "\\"

SwitchIP = "192.168.0.183" 
"""SwitchIP = "192.168.0.32" 
        #That's the IP of the No.2 Switch"""
SwitchPort = 9001
SwitchTerminator = "LF"
SwitchMoveCheckDelay = 0.5
SwitchCommandRetryDelay = 0.5

MeterName = "0x1313"
MeterID = "0x8078"
TLPMDllPath = "C:\\Program Files\\IVI Foundation\\VISA\\Win64\\Bin\\"
MeterWavelength = 1050
#PowerReadingsPerSec = 100
PowerReadingsAvgNum = 100
PowerFeedDataNum = 1000


Particle_serial_port = 'COM3' 
Arduinoser = 'COM7'

#### DO NOT EDIT ####

TLPMChannel = None
SwitchFinished = False
MonitorPause = False
SwitchConnected = False

def TCPExchange(ip, port, msg, replyLen):
    sck = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sck.connect((ip, port))

    sck.send(bytes(msg, "utf-8"))
    reply = str(sck.recv(replyLen))

    sck.close()

    return reply[2 : len(reply) - 1]

def SwitchInit():
    #time.sleep(1)

    reply = ""

    while (len(reply) != 22):
        print("SWITCH: Initializing...")
        reply = TCPExchange(SwitchIP, SwitchPort, "500000FF03FF00001C0000140200000100W*0000120001", 22)
        #print("SWITCH INIT RESULT: " + reply)

        if (len(reply) != 22) or (reply != "D00000FF03FF0000040000"):
            print("SWITCH: Init command failed! Retrying...")
            time.sleep(SwitchCommandRetryDelay)

    time.sleep(2)

    moveFinished = False

    while (moveFinished is False):
        time.sleep(SwitchMoveCheckDelay)
        status = SwitchGetStatus_newproto()

        if (status is not None) and ((status[0] != 0) and (status[1] != 0)):
            break

        print("SWITCH: Initializing...")

    print("SWITCH: Finished initializing.")

def SwitchGetStatus_newproto():
    #time.sleep(1)

    reply = ""

    while (len(reply) != 78):                       
        reply = TCPExchange(SwitchIP, SwitchPort, "500000FF03FF000018000004010000W*000000000E", 78)
        print("SWITCH STATUS RESULT: " + reply)
    
        if (len(reply) != 78):
            print("SWITCH: Status command failed! Retrying...")
            time.sleep(SwitchCommandRetryDelay)

    #print("SWITCH: Status acquired.")

    status = [
              int(reply[23 : 26]),    # Port A target position 
              int(reply[27 : 30]),    # Port B target position
              int(reply[31 : 34]),    # M01, M03 flag for current and target position match, 0001 == OK
              int(reply[35 : 38]),    # M02 insrer-disconnect axis current position, 0=during move, 0003=insert, 0001=disconnect
              int(reply[43 : 46] + reply[39 : 42], 16),    # Horizontal axis current position
              int(reply[51 : 54] + reply[47 : 50], 16),    # insertion axis current position
              int(reply[59 : 62] + reply[55 : 58], 16),    # vertical axis current position
              int(reply[63 : 66], 16),    # Alert number 1
              int(reply[67 : 70], 16),    # Alert number 2
              int(reply[71 : 74], 16),    # GUI message 1
              int(reply[75 : 78], 16)     # GUI message 2
              ]

    #print("Status = ", +status[0], status[1], status[2], status[3])

    return status


def SwitchSelectTarget(port):
    #time.sleep(1)

    if (port < 1) or (port > 7):
        raise ValueError("ERROR: Invalid switch port #!")
    if not (isinstance(port, int)):
        raise ValueError("ERROR: Switch port # is not an integer!")

    reply = ""

    while (len(reply) != 22):
        print("SWITCH: Selecting port #" + str(port) + "...") 
        if(port==1):
            reply = TCPExchange(SwitchIP, SwitchPort, "500000FF03FF000024000014010000W*0000140003010501040001", 22) # A:Cal B:Cleaner
        elif(port==2):
            reply = TCPExchange(SwitchIP, SwitchPort, "500000FF03FF000024000014010000W*0000140003010502020000", 22) # A:Cal B:Slit2
        elif(port==3):
            reply = TCPExchange(SwitchIP, SwitchPort, "500000FF03FF000024000014010000W*0000140003030602020001", 22) # A:Cleaner B:Slit2
        elif(port==4):
            reply = TCPExchange(SwitchIP, SwitchPort, "500000FF03FF000024000014010000W*0000140003030502010001", 22) # A:air blow B:Slit1

#        reply = TCPExchange(SwitchIP, SwitchPort, "500000FF03FF00001C0000140200000100W*000010000" + str(port), 22)
        print("SWITCH TARGET RESULT: " + reply)
    
        if (len(reply) != 22) or (reply != "D00000FF03FF0000040000"):
            print("SWITCH: Target command failed! Retrying...")
            time.sleep(SwitchCommandRetryDelay)

    print("SWITCH: Target set to port #" + str(port) + ".")
    #print("SWITCH TARGET (" + str(port) + ") RESULT: " + reply)

    #time.sleep(1)

def SwitchGoToTarget(port):
    #time.sleep(1)

    reply = ""

    while (len(reply) != 22):
        print("SWITCH: Connecting...")
        reply = TCPExchange(SwitchIP, SwitchPort, "500000FF03FF00001C0000140200000100W*0000120008", 22)
        print("SWITCH GOTO RESULT: " + reply)
    
        if (len(reply) != 22) or (reply != "D00000FF03FF0000040000"):
            print("SWITCH: GoTo command failed! Retrying...")
            time.sleep(SwitchCommandRetryDelay)

    #time.sleep(2)

    #print("SWITCH CONNECTING...")
    #time.sleep(20)
    #print("SWITCH FINISHED CONNECTING.")

    #print("Status ="+ Status)
    moveFinished = False

    while (moveFinished == False):
        print("SWITCH: Connecting...")
        time.sleep(SwitchMoveCheckDelay)
        status = SwitchGetStatus_newproto()
        
        GUImessage2 = format(status[10], '016b')
        print(GUImessage2[6])
        print(GUImessage2[7])
        if(port==2):
            if (status is not None) and ((status[2] != 0) and (status[3] != 0)): # while cleaner 1 or 2 is not working 
                break
        elif(port==1):
            if (status is not None) and ((status[2] != 0) and (status[3] != 0) and (GUImessage2[6] != '0')): # while cleaner 1 or 2 is not working 
                time.sleep(2)
                break
        elif(port==3):
            if (status is not None) and ((status[2] != 0) and (status[3] != 0) and (GUImessage2[7] != '0')): # while cleaner 1 or 2 is not working 
                time.sleep(2)
                break    

    print("SWITCH: Finished connecting.")

def SwitchDisconnect():
    #time.sleep(1)

    reply = ""

    while (len(reply) != 22):
        print("SWITCH: Disconnecting...")
        reply = TCPExchange(SwitchIP, SwitchPort, "500000FF03FF00001C0000140200000100W*0000120004", 22)
        #print("SWITCH DISCONNECT RESULT: " + reply)
    
        if (len(reply) != 22) or (reply != "D00000FF03FF0000040000"):
            print("SWITCH: Disconnect command failed! Retrying...")
            time.sleep(SwitchCommandRetryDelay)

    #print("SWITCH DISCONNECTING...")
    time.sleep(15)
    #print("SWITCH FINISHED DISCONNECTING.")

    """moveFinished = False

    while (moveFinished == False):
        print("SWITCH: Disconnecting...")
        time.sleep(SwitchMoveCheckDelay)
        status = SwitchGetStatus_newproto()

        if (status is not None) and ((status[0] != 0) and (status[1] != 0)):
            break"""

    print("SWITCH: Finished disconnecting.")


## Power meter ##############################


def TLPMInit(dllPath):
    global TLPMChannel

    TLPMChannel = TLPM(dllPath)
    
def MeterInit(name, id):
    global TLPMChannel

    TLPMInit(TLPMDllPath)

    print("Connected devices: ")
    devices = ListAllDevices()
    foundDevice = None

    for device in devices:
        deviceName = str(device.value)
        print("\t\"" + deviceName + "\"")
        
        if (deviceName.find("::" + name + "::") != -1) and (deviceName.find("::" + id + "::") != -1) and (deviceName.find("::INSTR") != -1):
            foundDevice = device

    if (foundDevice is None):
        raise ValueError("ERROR: Power meter \"" + name + "\" with ID \"" + id + "\" cannot be found!")

    TLPMChannel.open(foundDevice, c_bool(True), c_bool(True))
    TLPMChannel.setWavelength(c_double(MeterWavelength))
    #TLPMChannel.setAvgTime(c_double(5))
    #TLPMChannel.setAvgCnt(c_int16(500))
    #TLPMChannel.setTimeoutValue(c_uint32(10000))


def ListAllDevices():
    global TLPMChannel

    deviceCount = c_uint32()
    TLPMChannel.findRsrc(byref(deviceCount))
    names = []

    nameBuff = create_string_buffer(1024)

    for i in range(0, deviceCount.value):
        TLPMChannel.getRsrcName(i, nameBuff)
        names.append(nameBuff)

    return names


def MeterReadPower():
    #time.sleep(1)

    #readNum = PowerReadingsPerSec * PowerReadingsTime
    #readDelay = 1 / PowerReadingsPerSec
    reads = np.empty(shape = PowerReadingsAvgNum, dtype = float)

    #startTime = time.time()

    for i in range(PowerReadingsAvgNum):
        power = c_double()
        TLPMChannel.measPower(byref(power))
        reads[i] = power.value
        #print("Power =", +reads[i])
        #time.sleep(readDelay)
    
    #print(time.time() - startTime)
    #print(readDelay)

    return [np.average(reads), np.std(reads)]

def WriteToFile(filePath, data):
    file = None

    if not (os.path.isfile(filePath)):
        file = open(filePath, 'w')
    else:
        file = open(filePath, 'a')

    file.write(data)

    file.close()

### main ############################################
#MeterInit(MeterName, MeterID)

SwitchInit()  ## initialize switcher
#time.sleep(40)

port = 2
SwitchSelectTarget(port)
SwitchGoToTarget(port)
SwitchGetStatus_newproto()

"""
# Port 1: A:Cal B:Cleaner
# Port 2: A:Cal B:Slit2
# Port 3: A:Cleaner B:Slit2
# Port 4: A:air5 B:Slit1
Please see SwitchSelectTarget
"""
