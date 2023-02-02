from math import cos, sin, asin, acos, atan
import numpy as np
from time import sleep
import math
import sys
#Build Constants
#platform diameter = 8 inches = 0.2032 m
#base diameter = 3.5 inches = 0.0889 m
rp = 0.055 #m: this is the platform radius
rb = 0.078 #m: this is the base radius

#a = length of servo operating arm change to sal = servo arm length
sal = 0.021 #meters
s = 0.118 #meters length of operating leg

#---------------------------------


co = np.pi / 180

#Anchor points
#These are for the base and platform in their respective frameworks
# the coordinates of where the legs attach to the servo and platform respectively
# six legs so six sets of coords for both base and platform


p1 = [rp*cos(0),rp*sin(0),0.1]
p2 = [rp*cos(60*co),rp*sin(60*co),0.1]
p3 = [rp*cos(120*co),rp*sin(120*co),0.1]
p4 = [rp*cos(180*co),rp*sin(180*co),0.1]
p5 = [rp*cos(240*co),rp*sin(240*co),0.1]
p6 = [rp*cos(300*co),rp*sin(300*co),0.1]

b1 = [rb*cos(0),rb*sin(0),0]
b2 = [rb*cos(60*co),rb*sin(60*co),0]
b3 = [rb*cos(120*co),rb*sin(120*co),0]
b4 = [rb*cos(180*co),rb*sin(180*co),0]
b5 = [rb*cos(240*co),rb*sin(240*co),0]
b6 = [rb*cos(300*co),rb*sin(300*co),0]

plat = [p1,p2,p3,p4,p5,p6]
base = [b1,b2,b3,b4,b5,b6]

beta1 = 0 #angle of servo arm plane relative to the x-axis
beta2 = 60*co
beta3 = 120*co
beta4 = 180*co
beta5 = 240*co
beta6 = 300*co

betas = [beta1,beta2,beta3,beta4,beta5,beta6]

xp, yp, zp = p1[0],p1[1],p1[2]
xb, yb, zb = b1[0],b1[1],b1[2]

h0 = np.sqrt(s**2 + sal**2 - (xp - xb)**2 - (yp - yb)**2) - zp
# l0 = np.sqrt((xp - xb)**2 + (yp - yb)**2 + (h0 + zp)**2)

L0 = 2*sal**2
M0 = 2*sal*(xp - xb)
N0 = 2*sal*(h0 + zp)
alpha0 = asin(L0 / (np.sqrt(M0**2 + N0**2))) - atan(M0/N0)



#------------------------


#========================
#========================
    # the functions

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


#Matrix Functions (Not really durable in other applications)
#Multiply is for 1d vector meant to be column vector and thus will not apply to 2 matrices 
#could be fixed with an if statement eh.. too much work for now

def mult_matrix(A,B): #multiplies matrix by column vector in form A = [[x,y,z],[a,b,c],..] B = [1,2,2,..]
    C = []
    row_sum = 0
    if len(A[0]) != len(B):
        return "YOU LOSE MULTIPLY FOOL"
    else:
        for i in range(len(A)):
            for k in range(len(B)):
                row_sum += A[i][k] * B[k] #would require if and then B[k][j] to multiply two matrices
            C.append(row_sum)
            row_sum = 0
        return C

# A = [[1,1,1],[0,0,0],[2,3,4]]
# B = [1,2,3]
# L = [1,2,3]

# print(mult_matrix(A,B))



def add_matrix(A,B): #really only adds column vectors of form [1,2,3] due to commented out part being commented out
    A_B = []
    
    if len(A) != len(B):
        return "YOU LOSE"
    else:
        # for i in range(len(A)):
        #     # A_B.append([])
        #     for j in range(len(A[0])):
        #         A_B[i].append(A[i][j]+B[i][j])

        for i in range(len(A)):
            A_B.append(A[i] + B[i])
    return A_B





def sub_matrix(A,B): #really only subtracts column vectors of form [1,2,3] due to commented out part
    A_B = []
    
    if len(A) != len(B):
        return "YOU LOSE!"
    else:
        # for i in range(len(A)):
        #     # A_B.append([])
        #     for j in range(len(A[0])):
        #         A_B[i].append(A[i][j]-B[i][j])

        for i in range(len(A)):
            A_B.append(A[i] - B[i])
        
    return A_B

#---------------------------
def leg(T,R,p,b):
        leg_length_vector = sub_matrix(add_matrix(T,mult_matrix(R,p)),b)
        return leg_length_vector




def pythag(vec):
    total = 0
    for i, j in enumerate(vec):
        total += j**2

    return np.sqrt(total)




import serial
import time



startMarker = '<'
endMarker = '>'
dataStarted = False
dataBuf = ""
messageComplete = False



#Rotation Matrix

t = 0 #psi
a = 0 #theta
d = 0 #phi
x = 0
y = 0
z = 10/1000
change = 0.001
LINE_UP = "\033[1A"



setupSerial(115200, "COM3")
prevTime = time.time()


variable = 0
while True:
    bad = False
    # t = float(input("t = ")) *co  #psi
    # a = float(input("a = ")) *co  #theta
    # d = float(input("d = ")) *co  #phi
    # x = float(input("x = "))/1000
    # y = float(input("y = "))/1000
    z = float(input("z = "))/1000
    
    R = [[cos(t)*cos(a), -sin(t)*cos(d) + cos(t)*sin(a)*sin(d),sin(t)*sin(d) + cos(t)*sin(a)*cos(d)],[sin(t)*cos(a),cos(t)*cos(d) + sin(t)*sin(a)*sin(d),-cos(t)*sin(d)+sin(t)*sin(a)*cos(d)],[-sin(a),cos(a)*sin(d),cos(a)*cos(d)]]
    print(R)


    #----------------------------------


    #Translation Matrix
    # x = 0
    # y = 0
    # z = 0

    T = [x,y,z]


    #----------------------------------
 



    #leg lengths

    #l1 = T + R*p1 - b1
    #l2 = T + R*p2 ...
    #...
   

    l1 = leg(T,R,p1,b1)
    l2 = leg(T,R,p2,b2)
    l3 = leg(T,R,p3,b3)
    l4 = leg(T,R,p4,b4)
    l5 = leg(T,R,p5,b5)
    l6 = leg(T,R,p6,b6)

    #------------------------------------
    
    ls = [l1,l2,l3,l4,l5,l6]
    for i, j in enumerate(ls):
        if i != 1:
            continue
        else:
            # print('leg', i, '= ', pythag(j))
            continue


    
    
    
    #Servo Math
    alphas = []
    for i, j in enumerate(plat):
        # l = np.sqrt((plat[i][0] - base[i][0])**2 + (plat[i][1] - base[i][1])**2 + (plat[i][2] - base[i][2])**2) #np.sqrt((xp - xb)**2 + (yp - yb)**2 + (zp - zb)**2)
        l = pythag(ls[i])
        # print("l = ", l)
        L = l**2 - (s**2 - sal**2)
        # print("L = ", L)
        M = 2*sal*(plat[i][2] - base[i][2])   #2*sal*(zp - zb)

        
        N = 2*sal*(cos(betas[i])*(plat[i][0] - base[i][0]) + sin(betas[i])*(plat[i][1] - base[i][1])) #2*sal*(cos(betas[i])*(xp - xb) + sin(i)*(yp - yb))
        
        # print(L)
        # print(M)
        # print(N)
        # print(np.sqrt(M**2 + N**2))s
        val = ((L / np.sqrt(M**2 + N**2))) - (atan((N/M)) )
        # print('val = ', val)
        # if (val > 1) or (val < -1):
        #     sys.exit()
        try:
            alphas.append( asin(((L / np.sqrt(M**2 + N**2))) - (atan((N/M)) )) )
            # print('alpha = ', alphas[1]*(180/3.14159))
            
            # if (x > 0.2) or (x < -0.2):
            #     change *= -1
            
            
            #-----------------------------------
    # write pos to servo pos being alpha i believe
        except:
            print(val)
            print(L)
            print('leg lenth impossible?')
            bad = True
            break
            
            #change the changes. i want to also store the limits
            # sys.exit()
            # alphas.append( asin(((L / np.sqrt(M**2 + N**2))) - (atan((N/M)) )) )
    if not bad:
        vals = ['alpha = %f' %(i*(180/3.14159) ) for i in alphas]
        # print('alphas = ', vals, end='\r')
        # print('x =', x,end='\r\n')

        print(vals)
        
        alpha_degs = []
        for i in alphas:
            if i > 44.99:
                break
            elif i <= -44.99:
                break
            else:
                alpha_degs.append(math.floor(i * (180/3.14159)))
                
        alpha_degs[0] = str(-(alpha_degs[0]) + 37)
        alpha_degs[2] = str(-(alpha_degs[2]) + 40)
        alpha_degs[4] = str(-(alpha_degs[4]) + 37)
        alpha_degs[1] = str((alpha_degs[1] + 45))
        alpha_degs[3] = str((alpha_degs[3] + 45))
        alpha_degs[5] = str((alpha_degs[5] + 45))
        print(",".join(alpha_degs))
        print(len(",".join(alpha_degs)))
    
   # while True:
            # check for a reply
    # arduinoReply = recvLikeArduino()
    # if not (arduinoReply == 'XXX'):
    #     print ("Time %s  Reply %s" %(time.time(), arduinoReply))
        
        # send a message at intervals
    # alpha_degs = []
    # for i in alphas:
    #     alpha_degs.append(str(i * (180/3.14159)))
    # if time.time() - prevTime > 1.0:
    if not bad:
        # print("alpha = ", alphas[1]*180/3.14159)
        sendToArduino(",".join(alpha_degs))
        # sendToArduino(str(alphas[1]*(180/3.14159)))
        # prevTime = time.time()
    else:
        continue
    
    sleep(0.5)

# print(l1)
# print(l2)
# print(l3)
# print(l4)
# print(l5)
# print(l6)

# print(h0)





    #====================
    #====================
        # the program


    
    