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


f = open('23inchesFeb26_00.csv', 'rb')
reader = csv.reader(f)
rownum = 0
last_row = 72

encoder = numpy.zeros(last_row)
seconds = numpy.zeros(last_row)
amps = numpy.zeros(last_row)

for r in reader:
    #print( r[1])
    #print(rownum)
    if rownum< last_row and rownum>0:
        encoder[rownum] = r[0]
        seconds[rownum] = r[1]
        amps[rownum] = r[2]
    rownum +=1
	

charge = seconds*amps
tot = numpy.cumsum(charge)
#tot = numpy.zeros_like(charge)

pyplot.plot(encoder, tot , 'b-');
pyplot.xlabel('distance(encoder counts)');
pyplot.ylabel('charge C' );

