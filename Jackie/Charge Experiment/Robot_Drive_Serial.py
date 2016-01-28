# -*- coding: utf-8 -*-
"""
Robot Drive Serial Script
Created Conor Lyman 4 November 2015
Last update: 28 January 2016

Function sends commands to Arduino to drive robot. Recieves driving data 
from Arduino
"""

import serial
import time
import csv

data_file = open('data_30in5.csv', 'wb') #CSV file to store data sent by robot
writer = csv.writer(data_file)
writer.writerow(('index', 'encoder1', 'encoder2', 'turn angle degrees', 'ms', 'mA'))

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

brake90 = '$00013' #brake and turn 90 degrees
brake120 = '$00014' #brake and turn 120 degrees
brake180 = '$00016'
brake = '$00000' #brake only
drive = '$13000' #drive 20 inches

'''Need to clean up function'''
def read_robot(): 
    charToRead = 39
#    time.sleep(0.05)
    total_charge = 0
    while robot.inWaiting():
        b = robot.inWaiting()
        time.sleep(0.1)
        #print 'b1 ' + str(b)
        if b >= charToRead:
#            while b < charToRead:
#                b = robot.inWaiting()
            if b > charToRead:
                b = charToRead
            
            #print 'b2 ' + str(b)
            msg = robot.read(b)
            print 'msg ' + str(msg)
            index = float(msg[0])
            encoder1 = float(msg[2:9])
            encoder2 = float(msg[10:17])
            turnAngle = float(msg[18:22])
            milliseconds = float(msg[23:30])
            millicurrent = float(msg[31:])
            
            writer.writerow((index, encoder1, encoder2, turnAngle, milliseconds, millicurrent))
            
            if index == 1:
                current = millicurrent / 1000
                seconds = milliseconds / 1000
                charge = current * seconds
                total_charge += charge
            
        elif b < charToRead:
            robot.flushInput()
            robot.flushOutput()
            
    return total_charge
    
def brake_robot():
    robot.flushInput()
    robot.flushOutput()
    time.sleep(0.2)
    robot.write(brake)
    time.sleep(0.2)
    read_robot()
    time.sleep(0.2)    
    
def drive_robot():
    robot.flushInput()
    robot.flushOutput()
    time.sleep(0.2)
    robot.write(drive)
    time.sleep(0.3)
    charge = read_robot()
    return charge
    
def brake_robot180():
    robot.flushInput()
    robot.flushOutput()
    time.sleep(0.2)
    robot.write(brake180)
    time.sleep(0.2)
    read_robot()
    time.sleep(0.2)    
    

''' Script begins here'''


'''!!!!!CHANGE THESE BASED ON WHAT IS PLUGGED IN!!!!!'''
PORT = '/dev/tty.usbserial-DA011NKM' #.usbserial-DA00VSB5 for XBee
BaudRate = 38400

robot = serial.Serial(PORT, BaudRate, timeout = 1)
msgSize = 10
numDataPoints = 50

if robot.isOpen():
    print 'Robot opened\n'
    
time.sleep(0.5)

robot.flushInput()
robot.flushOutput()

'''Drive and read'''
charge1 = []
charge2 = []
charge3 = []
charge4 = []
charge5 = []

charge1 = drive_robot()
print "The charge for 20 in of drive is: " + str(charge1)
charge2 = drive_robot()
print "The charge for 20 in of drive is: " + str(charge2)
charge3 = drive_robot()
print "The charge for 20 in of drive is: " + str(charge3)
charge4 = drive_robot()
print "The charge for 20 in of drive is: " + str(charge4)
charge5 = drive_robot()
print "The charge for 20 in of drive is: " + str(charge5)
brake_robot()

averageCharge = (charge1 + charge2 + charge3 + charge4 + charge5) / 5
writer.writerow(('Charge1', 'Charge2', 'Charge3', 'Charge4', 'Charge5', 'Average Charge'))
writer.writerow((charge1, charge2, charge3, charge4, charge5, averageCharge))

print "The average charge for 20 in of drive is: " + str(averageCharge)

robot.close()
data_file.close()

if robot.isOpen() == False:
    print '\nRobot closed'