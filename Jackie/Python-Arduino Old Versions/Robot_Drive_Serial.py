# -*- coding: utf-8 -*-
"""
Robot Drive Serial Script
Created Conor Lyman 4 November 2015
Last update: 4 November 2015

Function sends commands to Arduino to drive robot. Recieves driving data 
from Arduino
"""

import serial
import time

'''CHANGE THESE'''
PORT = '/dev/tty.usbserial-DA00VSB5'
BaudRate = 38400

robot = serial.Serial(PORT, BaudRate, timeout = 1)
msgSize = 10
numDataPoints = 50

if robot.isOpen():
    print 'Robot opened'
    
time.sleep(0.5)

robot.flushInput()
robot.flushOutput()

brake1 = '$01300'
brake2 = '$00000'
drive = '$10000'

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




