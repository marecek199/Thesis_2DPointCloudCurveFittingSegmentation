


import numpy as np 
import matplotlib.pyplot as plt
import pickle 

file = open('data_python/data_namerane_vsetkoLT.txt','r')

new_data = []
pomocne_data = []

for _ in range(107):
    file.readline(1)
    data = file.readline().split(',')

    for i in range(0,len(data)-2,2):
        pomocne_data.append([float(data[i][1:]), float(data[i+1][:-1])])

    pomocne_data.append([float(data[-2][1:]), float(data[-1][0:-3])])    
    
    # print('Celkovo',str(len(pomocne_data))+' bodov')
    new_data.append(pomocne_data)
    pomocne_data=[]

file.close()


xmin,ymin = 0,0

# 
new_pole=[]
xlast,ylast = None,None
for pokus in range(107):
    pomoc_pole=[]
    for idx in range(360):
        x,y = new_data[pokus][idx][0], new_data[pokus][idx][1]
        found = False
        if pokus==2:
            a='whaat'
        pomoc_pole.append([round(x*1000),round(y*1000)])
        xlast=x
        ylast=y
        if xmin > round(x*1000):
            xmin = round(x*1000)
        if ymin > round(y*1000):
            ymin = round(y*1000)

    new_pole.append(pomoc_pole)


#posunutie nuloveho bodu
for pokus in new_pole:
    for stlpec in pokus:
        if stlpec[0]==0 and stlpec[1] == 0:
            continue
        stlpec[0] += abs(xmin) + 50
        stlpec[1] += abs(ymin) + 50

for i in range(10):
    plt.figure()
    print(i,len(new_pole[i]))
    for idx in new_pole[i]:
        plt.plot(idx[0],idx[1],'.r')
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

pickle.dump(new_pole,open('data_python/data_namerane_vsetkoLT.p','wb'))






