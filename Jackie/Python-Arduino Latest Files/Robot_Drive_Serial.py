# -*- coding: utf-8 -*-
"""
Robot Drive Serial Script
Created Conor Lyman 4 November 2015
Last update: 20 November 2015

Function sends commands to Arduino to drive robot. Recieves driving data 
from Arduino
"""

import serial
import time

'''CHANGE THESE BASED ON WHAT IS PLUGGED IN'''
PORT = '/dev/tty.usbserial-DA00VSB5' #.usbserial-DA00VSB5 for XBee
BaudRate = 38400

robot = serial.Serial(PORT, BaudRate, timeout = 1)
msgSize = 10
numDataPoints = 50

if robot.isOpen():
    print 'Robot opened'
    
time.sleep(0.5)

robot.flushInput()
robot.flushOutput()

'''command string:
	[0] = drive 	(1 = drive, 0 = brake)
	[1] = distanceMult1
	[2] = distanceMult2
		(drive for 1 inch * totalDistanceMult)
		totalDistanceMult = distanceMult1 and distance Mult2
		Ex: distanceMult1 = 1, distanceMult2 = 5  ->  totalDistanceMult = 15
	[3] = turn 		(1 = turn, 0 = dont')  		!!! Must brake before turning!!!
	[4] = turnMult	(degreesTurned = turnMult * 30)
	
'''
brake1 = '$0xx13'
brake2 = '$0xx00'
drive = '$1xx00'

robot.write(brake2)
time.sleep(0.5)

robot.write(drive)
time.sleep(1.5)
robot.write(brake1)
time.sleep(3)
robot.write(drive)
time.sleep(1.5)
robot.write(brake1)
time.sleep(3)
robot.write(drive)
time.sleep(1.5)
robot.write(brake1)
time.sleep(3)
robot.write(drive)
time.sleep(1.5)
robot.write(brake1)
time.sleep(3)
robot.close()
robot.isOpen()




