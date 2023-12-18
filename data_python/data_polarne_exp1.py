import numpy as np 
import matplotlib.pyplot as plt
import pickle 

file = open('data_python/data_namerane_polarne_exp1.txt','r')

file.readline(1)
data = file.read().split(',')

new_data = []
for i in range(0,len(data)-2,2):
    new_data.append([int(data[i][1:]),int(data[i+1][:-1])])

new_data.append([int(data[-2][1:]),int(data[-1][:-2])])    
print('Celkovo',str(len(new_data))+' bodov')



pickle.dump(new_data,open('data_python/data_namerane_polarne_exp1.p','wb'))