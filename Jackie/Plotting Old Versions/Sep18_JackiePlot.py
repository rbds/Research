# -*- coding: utf-8 -*-
"""
Created on Wed Aug 12 09:54:23 2015

@author: clyman

Last Update: 11 September 2015
"""

import serial
import time
import matplotlib.pyplot as plt
import numpy as np

def draw_robot(pos, rad, theta): #function to draw circle representing robot
    the = np.linspace(0, 2*np.pi, 100, endpoint=True)
    xunit = rad * np.cos(the) + pos[0]
    yunit = rad * np.sin(the) + pos[1]
    
    center = pos
    front = [0, 0]
    front[0] = center[0] + rad*np.cos(theta)
    front[1] = center[1] + rad*np.sin(theta)
    
    plt.plot([center[0], front[0]], [center[1], front[1]], 'b-', xunit, yunit, 'b-')
    plt.draw()

plt.ion() #interactive mode http://matplotlib.org/faq/usage_faq.html#what-is-interactive-mode 
robot_position = [12, -15] #starting position
robot_radius = 3 #size of robot representation
robot_theta = 0 #angle robot is pointed at

straight_gain = 1.0/791.0 # 791 encoder counts per inch
straight_std = 30 #unsure what this is for

plt.figure(1)
plt.subplot(211)
plt.hold(True)
plt.axis([-10, 110, -70, 10])
plt.plot([0, 102], [0, 0], 'b-') #Create box representing boundaries
plt.plot([102,102], [0, -62], 'b-')
plt.plot([102, 0], [-62, -62], 'b-')
plt.plot([0, 0], [-62, 0], 'b-')

plt.figure(1)
plt.subplot(212)
plt.hold(True)
plt.axis([0, 20, 0, 5])
plt.xlabel("Time (s)")
plt.ylabel("Voltage (V)")
plt.title("Voltage vs. Time")
plt.text(15, 4, "Red = Solar\nBlack = Thermo")

robot = serial.Serial('/dev/tty.usbserial-DA00VSB5', 38400, timeout = 1) 

if robot.isOpen() == True:
    print "Port opened"

data_file = open('data.txt', 'w') #Text file to store data sent by robot

robot.flushInput()
robot.flushOutput()

new_pos = [0, 0] #Initializing lists to be used later
all_index = []
all_dist = []
all_enc1 = []
all_enc2 = []
all_enc3 = []
all_turnang = []
all_solarV = []
all_thermoV = []
all_ang = []
all_secs = []

old_seconds = 0
old_solar = 0
old_thermo = 0

while len(all_index) < 50: 
    time.sleep(.1)
    robot.flushInput()
    robot.flushOutput()
    b = robot.inWaiting()
    
    while b < 66: #Wait for 66 bytes to be sent
        b = robot.inWaiting()
    
    if b > 66: #Ensure that 66 bytes are read (the length of one data thread)
        b = 66

    msg = robot.read(b)
    index = float(msg[0])
    dist = float(msg[2:8])
    enc1 = float(msg[9:16])
    enc2 = float(msg[17:24])
    enc3 = float(msg[25:32])
    turnangleave = float(msg[33:37])
    solarVoltage = float(msg[38:44])
    thermoVoltage = float(msg[45:51])
    angle = float(msg[52:58])
    seconds = float(msg[59:])
    
    all_index.append(index) #Store all the collected data
    all_dist.append(dist)
    all_enc1.append(enc1)
    all_enc2.append(enc2)
    all_enc3.append(enc3)
    all_turnang.append(turnangleave)
    all_solarV.append(solarVoltage)
    all_thermoV.append(thermoVoltage)
    all_ang.append(angle)
    all_secs.append(seconds)
    
    if (index==1 or index==0): #Plot the robot's path
        dst = enc1 + enc2 + enc3
        dst = dst/3.0 * straight_gain
        new_pos[0] = robot_position[0] + dst*np.cos(robot_theta)
        new_pos[1] = robot_position[1] + dst*np.sin(robot_theta)
        plt.figure(1)
        plt.subplot(211)
        plt.plot([robot_position[0], new_pos[0]], [robot_position[1], new_pos[1]], 'g-')
        robot_position[0] = new_pos[0]
        robot_position[1] = new_pos[1]
        #h = draw_robot(robot_position, robot_radius, robot_theta)
        
    elif index==3: #Robot turning right
        turnangleave = turnangleave * (np.pi / 180) * (-1)
        robot_theta = turnangleave + robot_theta
        #h = draw_robot(robot_position, robot_radius, robot_theta)
     
    #index 4 removed
    '''elif index==4: #Robot turning left
        turnangleave = turnangleave * (np.pi / 180)
        robot_theta = turnangleave + robot_theta
        #h = draw_robot(robot_position, robot_radius, robot_theta)'''
    
    if (dist!=0 and index==2): #Plot obstacles seen by robot only when robot is stationary panning
        angle = (angle - 90.0) * (-1)
        angle = angle * (np.pi / 180.0)
        theta = robot_theta + angle
        x2 = dist * np.cos(theta) + robot_position[0] + 5 * np.cos(theta) #Last term accounts for distance from robot center to sensor
        y2 = dist * np.sin(theta) + robot_position[1] + 5 * np.sin(theta)
        plt.figure(1)
        plt.subplot(211)
        plt.plot(x2, y2, 'r*')
        
    if solarVoltage>=4.5: #Plot sources of solar power
        plt.figure(1)
        plt.subplot(211)
        plt.plot(robot_position[0], robot_position[1], 'y*')
        
    if thermoVoltage>=0.02: #Plot sources of geothermal power
        plt.figure(1)
        plt.subplot(211)
        plt.plot(robot_position[0], robot_position[1], 'k*')
        
    plt.figure(1)
    plt.subplot(212)
    seconds = seconds / 1000
    plt.plot([old_seconds, seconds] , [old_solar, solarVoltage], 'r-')
    plt.plot([old_seconds, seconds], [old_thermo, thermoVoltage], 'k-')
    old_seconds = seconds
    old_solar = solarVoltage
    old_thermo = thermoVoltage
    plt.draw()
    
    plt.figure(1)
    plt.draw()    
    data_file.write(msg) #Store data in text file
    data_file.write('\n')
    
plt.figure(1)
plt.subplot(211)
h = draw_robot(robot_position, robot_radius, robot_theta)

data = [all_index, all_dist, all_enc1, all_enc2, all_enc3, all_turnang, \
all_solarV, all_thermoV, all_ang, all_secs] #Store data in list


robot.close()
data_file.close()
