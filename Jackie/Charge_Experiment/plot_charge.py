# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

from matplotlib import pyplot
from matplotlib import rcParams
rcParams['font.family'] = 'serif'
rcParams['font.size'] = 16
import csv
import numpy

lengths = numpy.array([13, 23, 33, 43, 53])
#lengths = numpy.array([13])
mn = numpy.zeros(6)
st = numpy.zeros(6)

for ind, j  in enumerate(lengths):
    charge_totals = numpy.array([])
    
    for i in range(0,20):
        filename = str(j) + 'inchesFeb26_0' + str(i) + '.csv'
        with open(filename, 'rb') as f:
            mycsv = csv.reader(f)
            mycsv = list(mycsv)
            row_count = len(mycsv) - 2
            start_at = float(mycsv[1][0])
        
        f = open(filename, 'rb')
        reader = csv.reader(f)
        rownum = 0
        
        encoder = numpy.array([])
        seconds = numpy.array([])
        amps = numpy.array([])
        
        for r in reader:
            #print(r[1])
            if rownum>0 and rownum<row_count:
                encoder = numpy.append(encoder, float(r[0]) - start_at)
                seconds = numpy.append(seconds, float(r[1]))
                amps = numpy.append(amps, float(r[2]))
            rownum +=1
        f.close()
        charge = seconds*amps
        tot = numpy.cumsum(charge)
        charge_totals = numpy.append(charge_totals, tot[-1])

    #print(charge_totals)
    #print(numpy.size(charge_totals))    
    mn[ind+1] = numpy.mean(charge_totals)
    st[ind+1] = numpy.std(charge_totals)
    
l = numpy.append([0], lengths) 
m, b = numpy.polyfit(l, mn, 1)
pyplot.figure()
pyplot.plot(l, m*l+b)
pyplot.errorbar(l, mn, yerr = st, fmt = 'x')
pyplot.title('Charge vs. distance')
pyplot.xlabel('inches travelled')
pyplot.ylabel('charge used (Amp-seconds)')

pyplot.savefig('charge_v_distance.png')

#    pyplot.plot(encoder, tot , 'b-');
#    pyplot.xlabel('distance(encoder counts)');
#    pyplot.ylabel('charge C' );

