# -*- coding: utf-8 -*-
"""
Created on Sun Feb 28 17:06:48 2016

@author: Randy
"""

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


f = open('13_gravel_Feb28_0.csv', 'rt')
reader = csv.reader(f)

#with open('13_gravel_Feb28_0.csv', 'rt') as f:
#    mycsv = csv.reader(f)
#    mycsv = list(mycsv)
    #print(mycsv[2][1])
    
    
rownum = 0
last_row = 30

encoder = numpy.zeros(last_row)
seconds = numpy.zeros(last_row)
amps = numpy.zeros(last_row)

for r in reader:
    #print( r[1])
    #print(rownum)
    if rownum< last_row and rownum>1:
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