# -*- coding: utf-8 -*-
"""
Created on Sat Feb 20 14:36:33 2016

@author: clyman
"""

import serial
import time

drive = '$14000' #drive 30 inches

def read_robot(): 
    while True:
        charToRead = 25
        checkCharacter = '$1'
        breakCheck = '$0'
        check = robot.read(2)
        if check == checkCharacter:
    #        while robot.inWaiting():
            b = robot.inWaiting()
    #            time.sleep(0.05)
            if b >= charToRead:
    #                if b > charToRead:
    #                    b = charToRead
                msg = robot.readline()
                print msg
     #           print "\n"
        elif check == breakCheck:
            break
        #elif check != checkCharacter and check != breakCheck:
         #   check = robot.read(2)
            
    
    
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