#calibrate servos
from time import sleep
import sys
import serial
import time



startMarker = '<'
endMarker = '>'
dataStarted = False
dataBuf = ""
messageComplete = False




def setupSerial(baudRate, serialPortName):
    
    global  serialPort
    
    serialPort = serial.Serial(port= serialPortName, baudrate = baudRate, timeout=0, rtscts=True)

    print("Serial port " + serialPortName + " opened  Baudrate " + str(baudRate))

    waitForArduino()

#========================

def sendToArduino(stringToSend):
    
        # this adds the start- and end-markers before sending
    global startMarker, endMarker, serialPort
    
    stringWithMarkers = (startMarker)
    stringWithMarkers += stringToSend
    stringWithMarkers += (endMarker)

    serialPort.write(stringWithMarkers.encode('utf-8')) # encode needed for Python3


#==================

def recvLikeArduino():

    global startMarker, endMarker, serialPort, dataStarted, dataBuf, messageComplete

    if serialPort.inWaiting() > 0 and messageComplete == False:
        x = serialPort.read().decode("utf-8") # decode needed for Python3
        
        if dataStarted == True:
            if x != endMarker:
                dataBuf = dataBuf + x
            else:
                dataStarted = False
                messageComplete = True
        elif x == startMarker:
            dataBuf = ''
            dataStarted = True
    
    if (messageComplete == True):
        messageComplete = False
        return dataBuf
    else:
        return "XXX" 

#==================

def waitForArduino():

    # wait until the Arduino sends 'Arduino is ready' - allows time for Arduino reset
    # it also ensures that any bytes left over from a previous message are discarded
    
    print("Waiting for Arduino to reset")
     
    msg = ""
    while msg.find("Arduino is ready") == -1:
        msg = recvLikeArduino()
        if not (msg == 'XXX'): 
            print(msg)




setupSerial(115200, "COM3")
prevTime = time.time()

while True:
    alpha1 = input("servo 1: ")
    alpha2 = input("servo 2: ")
    alpha3 = input("servo 3: ")
    alpha4 = input("servo 4: ")
    alpha5 = input("servo 5: ")
    alpha6 = input("servo 6: ")
    arduinoReply = recvLikeArduino()
    # if not (arduinoReply == 'XXX'):
    #     print ("Time %s  Reply %s" %(time.time(), arduinoReply))
        
        # send a message at intervals
    alpha_degs = [alpha1,alpha2,alpha3,alpha4,alpha5,alpha6]
    
        # print("alpha = ", alphas[1]*180/3.14159)
    sendToArduino(",".join(alpha_degs))
        # sendToArduino(str(alphas[1]*(180/3.14159)))
        # prevTime = time.time()
    
    time.sleep(1)