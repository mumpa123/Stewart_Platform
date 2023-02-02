import numpy as np

from math import cos,sin

import time
start_time = time.time()


# p1 = [[-2.8366],[10],[0]] #TODO convert these coordinates in terms of the base and platform radii
# b1 = [[-2.8366*2],[20],[0]] #in order to allow for generalization of code

# p2 = [[2.8366],[10],[0]]
# b2 = [[2.8366*2],[20],[0]]

# p3 = [[9.912028],[-1.32351],[0]]
# b3 = [[9.912028*2],[-1.32351*2],[0]]

# p4 = [[9.036922],[-4.2818],[0]]
# b4 = [[9.036922*2],[-4.2818*2],[0]]

# p5 = [[-9.036922],[-4.2818],[0]]
# b5 = [[-9.036922*2],[-4.2818*2],[0]]

# p6 = [[-9.912028],[-1.32351],[0]]
# b6 = [[-9.912028*2],[-1.32351*2],[0]]





p1 = [-2,10,0]
b1 = [-4,20,0]

p2 = [2,10,0]
b2 = [4,20,0]

p3 = [9,-1,0]
b3 = [18,-2,0]

p4 = [-2,-4,0]
b4 = [-4,-8,0]

p5 = [-9,-4,0]
b5 = [-18,-8,0]

p6 = [-9,-1,0]
b6 = [-19,-2,0]

p = [p1,p2,p3,p4,p5,p6] #platform anchor points in platform reference frame
b = [b1,b2,b3,b4,b5,b6] #base anchor points

# xp = p1[0][0]
# yp = p1[0][0]
# zp = p1[0][0]

# xb = b1[0][0]
# yb = b1[0][0]
# zb = b1[0][0]

xp = 2
yp = 2
zp = 2
xb = 1
yb = 1
zb = 0
#get s a betai
#a = length of servo operating arm change to sal = servo arm length
sal = 0.02 #meters
s = 0.2 #meters length of operating leg
betai = 0

# print(xp,yp,zp,xb,yb,zb)


#3. find h0 and alpha0
h0 = np.sqrt(s**2 + sal**2 - (xp*10**-2 - xb*10**-2)**2 - (yp*10**-2 - yb*10**-2)**2) - zp

print(h0)



x = 4
y = 5
z = 6
T = [[x],[y],[z]]

t = 5 #psi
a = 5 #theta
d = 5 #phi

R = [[cos(t)*cos(a), -sin(t)*cos(d) + cos(t)*sin(a)*sin(d),sin(d)*sin(a) + cos(t)*sin(a)*cos(d)],[sin(t)*cos(a),cos(t)*cos(d) + sin(t)*sin(a)*sin(d),\
      -cos(t)*sin(d)+sin(t)*sin(a)*cos(d)],[-sin(a),cos(a)*sin(d),cos(a)*cos(d)]]

# h0 is the hope position of the platform. no rotation translational. height of platform above base
# h0 = np.sqrt(s**2 + a**2 - (xp - xb)**2 - (yp - yb)**2) - zp 

def add_matrix(A,B):
    A_B = []
    
    if len(A) != len(B):
        return "YOU LOSE"
    else:
        for i in range(len(A)):
            A_B.append([])
            for j in range(len(A[0])):
                A_B[i].append(A[i][j]+B[i][j])
    return A_B


def sub_matrix(A,B):
    A_B = []
    
    if len(A) != len(B):
        return "YOU LOSE!"
    else:
        for i in range(len(A)):
            A_B.append([])
            for j in range(len(A[0])):
                A_B[i].append(A[i][j]-B[i][j])
    return A_B





def mult_matrix(A,B):
    C = []
    row_sum = 0
    if len(A[0]) != len(B):
        return "YOU LOSE MULTIPLY FOOL"
    else:
        for i in range(len(A)):
            
            C.append([])
            for j in range(len(B[0])):
                for k in range(len(B)):
                    row_sum += A[i][k] * B[k][j]
                C[i].append(row_sum)
                row_sum = 0
        return C






# leg_lengths = []
# for i in range(1):
#     for j in range(6):
#         leg_lengths.append(add_matrix(sub_matrix(mult_matrix(R,p[j]),b[j]),T))
    
#     print(leg_lengths)

    
    
            
    # leg_lengths = []
    


leg_length = []
leg_lengths = []
for j in range(6):
    leg_length = T + np.matmul(R,p[j]) - b[j]
    leg_lengths.append(leg_length)
    leg_length = []
print(leg_lengths)

legs = []
for i in leg_lengths:
    for j in i:
        length_abs = np.sqrt(j[0]**2 + j[1]**2 + j[2]**2)
        legs.append(length_abs)


print(legs)
print("--- %s seconds ---" % (time.time() - start_time))