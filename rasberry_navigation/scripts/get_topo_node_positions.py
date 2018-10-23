# -*- coding: utf-8 -*-
"""
Created on Fri Aug 31 09:05:46 2018

@author: Adam Binch

Calculate new node postions on the topological map that are centred between the poles
and write them to a text file.
"""
#####################################################################################
import numpy as np
import matplotlib.pyplot as plt


def get_pole_positions(filename):
    
    with open(filename) as f:
        content = f.readlines()
        content = [x.strip() for x in content] 
        
    vals=[]
    for i in content:
        d=i.split()
        vals.append(d[1:])

    for i in vals:
        i[0]=float(i[0])
        i[1]=float(i[1])
        
    return np.array(vals)
    
    
def get_centres(positions):
    xcs = []
    ycs = []
    count = 0
    for i in range(4):
        pos = positions[i*9:i*9+18]
    
        for j in range(8):
            
            p1 = pos[j]
            p2 = pos[j+1]
            p3 = pos[j+9]
            p4 = pos[j+10]
            
            ps = np.vstack([p1, p2, p3, p4])  
            xc = np.mean(ps[:, 0])
            yc = np.mean(ps[:, 1])
            
            xcs.append(xc)
            ycs.append(yc)
            
            count+=1

    return np.vstack((xcs, ycs)).T


def insert_centres(c):    
    row_top = []
    row_bottom = []
    for i in range(8):
        p1 = c[i]
        p2 = c[i+8]
        p3 = c[i+16]
        p4 = c[i+24]
        p = np.array([p1, p2, p3, p4])
        dx = np.mean(np.abs(np.diff(p[:, 0])))
        dy = np.mean(np.abs(np.diff(p[:, 1])))
        row_top.append([p4[0]+dx, p4[1]+dy])
        row_bottom.append([p1[0]-dx, p1[1]-dy])
    
    c = np.vstack((c, row_top))     
    c = np.vstack((row_bottom, c))    
    
    col_left = []
    col_right = []
    for i in range(6):
        p = c[i*8:(i+1)*8]
        dx = np.mean(np.abs(np.diff(p[:, 0])))
        dy = np.mean(np.abs(np.diff(p[:, 1])))
        col_left.append([p[0][0]-dx, p[0][1]+dy])
        col_right.append([p[-1][0]+dx, p[-1][1]-dy])
    
    indexes_left = np.arange(0, 45+9, 9)
    indexes_right = np.arange(9, 59+10, 10)
    c = insert_top_bottom_centres(c, indexes_left, col_left)
    c = insert_top_bottom_centres(c, indexes_right, col_right)

    return c        
    

def insert_top_bottom_centres(c, indexes, col):
    for i, index in enumerate(indexes):
        c = np.insert(c, [index], col[i], axis=0)
    return c    
#####################################################################################     
    
    
##################################################################################### 
plot = True
write_to_file = False
  
posA = get_pole_positions("polytunnelA_pole_positions.txt")
posB = get_pole_positions("polytunnelB_pole_positions.txt")  

cA = get_centres(posA)
cB = get_centres(posB)

cA = insert_centres(cA)
cB = insert_centres(cB)

# way point numbers for polytunnel A
way_points = [65,6,2,1,5,4,3,7,8,9]
way_points.extend([64,18,17,16,15,14,13,12,11,10])
way_points.extend([60,19,20,21,22,23,24,25,26,27])
way_points.extend([59,28,29,30,31,32,33,34,35,36])
way_points.extend([58,37,38,39,40,41,42,43,44,45])
way_points.extend([57,54,53,52,51,50,49,48,47,46])
way_points = np.array(way_points).reshape(len(way_points), 1)

cA = np.hstack((way_points, cA))
cB = np.hstack((way_points, cB))

#new_node_positions = [[13.138, 51.481], [5.928, 53.080], [5.494, 45.340]]
#yaw = np.arctan2(8.18-0.68, -6.18--6.18) # From (x1,y1) to (x2,y2) the direction is atan2(y2−y1,x2−x1)
#quat = tf.transformations.quaternion_from_euler(0, 0, yaw)

if plot:
    plt.figure(1); plt.clf()
    plt.plot(posA[:, 0], posA[:, 1], 'b.')
    plt.plot(cA[:, 1], cA[:, 2], 'r.')
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.title('Polytunnel 1')
    plt.legend(['Poles', 'Centres'], frameon=False)
    plt.axis('equal')
    
    plt.figure(2); plt.clf()
    plt.plot(posB[:, 0], posB[:, 1], 'b.')
    plt.plot(cB[:, 1], cB[:, 2], 'r.')
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.title('Polytunnel 2')
    plt.legend(['Poles', 'Centres'], frameon=False)
    plt.axis('equal')

if write_to_file:
    np.savetxt('centres_polytunnelA.txt', cA, fmt='%1.3f')
    np.savetxt('centres_polytunnelB.txt', cB, fmt='%1.3f')
######################################################################################