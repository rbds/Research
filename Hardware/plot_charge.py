from matplotlib import pyplot
from matplotlib import rcParams, cm
rcParams['font.family'] = 'serif'
rcParams['font.size'] = 16
import csv

f = open('file_name.csv', 'rb')
reader = csv.reader(f)
row_num = 0
last_row = 141

for r in range(1,last_row):
	encoder1[r] = reader[r,0]
	encoder2[ = r[1]
	
print encoder1
	

	

