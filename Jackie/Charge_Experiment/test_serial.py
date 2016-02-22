# -*- coding: utf-8 -*-
"""
Created on Sat Feb 20 14:36:33 2016

@author: clyman
"""

import serial
import time

drive = '$14000' #drive 30 inches

def read_robot(): 
    charToRead = 39
    while robot.inWaiting():
        b = robot.inWaiting()
        time.sleep(0.3)
        if b >= charToRead:
            if b > charToRead:
                b = charToRead
            msg = robot.read(b)
            print msg
            print "\n"
        elif b < charToRead:
            robot.flushInput()
            robot.flushOutput() 
            
    
    
'''.....................................................................................'''



'''!!!!!CHANGE THESE BASED ON WHAT IS PLUGGED IN!!!!!'''
PORT = '/dev/tty.usbserial-DA011NKM' #.usbserial-DA00VSB5 for XBee
BaudRate = 38400

robot = serial.Serial(PORT, BaudRate, timeout = 3)
time.sleep(0.5)
if robot.isOpen():
    print 'Robot opened\n'
    
robot.flushInput()
robot.flushOutput()
time.sleep(0.3)
robot.write(drive)
time.sleep(0.3)
read_robot()

robot.close()

if robot.isOpen() == False:
    print '\nRobot closed'