

import numpy as np 
import matplotlib.pyplot as plt
import pickle 

file = open('data_python/data_namerane.txt','r')

file.readline(1)
data = file.read().split(',')

new_data = []
for i in range(0,len(data)-2,2):
    new_data.append([int(data[i][1:]),int(data[i+1][:-1])])

new_data.append([int(data[-2][1:]),int(data[-1][:-2])])    
print('Celkovo',str(len(new_data))+' bodov')

#107 merani
dlzka = len(new_data) // 107

new_pole = []
i=0

xlast,ylast = None,None
for pokus in range(107):
    pomoc_pole=[]
    for idx in range(360):
        x,y = new_data[idx+(pokus*107)][0],new_data[idx+(pokus*107)][1]
        found = False
        for porovn in pomoc_pole:
            if x == porovn[0] and y == porovn[1] or x==0 and y==0:
                found=True
        if found==False:
            pomoc_pole.append([x,y])
        xlast=x
        ylast=y

    new_pole.append(pomoc_pole)


for idx in new_pole[0]:
    plt.plot(idx[0],idx[1],',r')
plt.show()

# exp1 = []
# for idx in new_pole[0]:
#     x,y = idx[0],idx[1]
#     found = False
#     for porovn in exp1:
#         if x==porovn[0] and y == porovn[1]:
#             found = True
#     if found == False:
#         exp1.append([x,y])

# print(len(exp1))
# print(exp1)

pickle.dump(new_pole,open('data_python/data_namerane_vsetko2.p','wb'))
# pickle.dump(new_pole[0],open('data_python/data_namerane_exp1.p','wb'))
# pickle.dump(new_pole[1],open('data_python/data_namerane_exp2.p','wb'))
# pickle.dump(new_pole[2],open('data_python/data_namerane_exp3.p','wb'))
# print(len(new_pole[0]))
# print(new_pole[0])

file.close()




