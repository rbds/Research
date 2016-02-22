# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

from matplotlib import pyplot
from matplotlib import rcParams, cm
rcParams['font.family'] = 'serif'
rcParams['font.size'] = 16
import csv
import numpy


f = open('data_40in1_new.csv', 'rb')
reader = csv.reader(f)
rownum = 0
last_row = 116

encoder1 = numpy.zeros(last_row)
encoder2 = numpy.zeros(last_row)
timems = numpy.zeros(last_row)
currentmA = numpy.zeros(last_row)

for r in reader:
    #print( r[1])
    #print(rownum)
    if rownum< last_row and rownum>0:
        encoder1[rownum] = r[1]
        encoder2[rownum] = r[2]
        timems[rownum] = r[4]
        currentmA[rownum] = r[5]
    rownum +=1
	
encAve = 0.5*(encoder1+encoder2)
time = timems/1000.
current = currentmA/1000.

charge = time*current
tot = numpy.cumsum(charge)
#tot = numpy.zeros_like(charge)

pyplot.plot(encAve, tot , 'b-');
pyplot.xlabel('distance(encoder counts)');
pyplot.ylabel('charge C' );

