

import numpy as np
from PIL import Image
import pickle
import matplotlib.pyplot as plt
import pickle
import sys
import statistics
import random
import time

def vykresli_pyplot(vysvietene_pole,povodne_pole):
    global w,h
    plt.figure()
    for boddvoj in povodne_pole:
        x,y = boddvoj[0], boddvoj[1]
        plt.axis([0,w,0,h])
        plt.plot(x,y,',b')

    for boddvoj in vysvietene_pole:
        x,y = boddvoj[0], boddvoj[1]
        plt.axis([0,w,0,h])
        plt.plot(x,y,'.r')

def vykresli_pyplot_farba(vysvietene_pole,povodne_pole,farba,sign):
    global w,h
    # for boddvoj in povodne_pole:
    #     x,y = boddvoj[1], boddvoj[0]
    #     plt.axis([0,w,0,h])
    #     plt.plot(x,512-y,',b')

    for boddvoj in vysvietene_pole:
        x,y = boddvoj[0], boddvoj[1]
        # plt.axis([0,w,0,h])
        plt.plot(x,y, marker=sign, color=farba)


def vykresli(pole_bodov):
    global w,data_farba    
    R_farba = []
    G_farba = []
    B_farba = []
    for idx in range(len(pole_bodov)):
        x,y = pole_bodov[idx][0], pole_bodov[idx][1]
        # if x>=w or y>=w:
        #     continue
        R_farba.append(data_farba[x][y][0])
        G_farba.append(data_farba[x][y][1])
        B_farba.append(data_farba[x][y][2])
        data_farba[x][y][0] = 255
        data_farba[x][y][1] = 255
        data_farba[x][y][2] = 255
        if idx == 0 or idx == len(pole_bodov)-1 :
            data_farba[x][y][1] = 0
            data_farba[x][y][2] = 0
    
    Image.fromarray(data_farba,'RGB').show()
    i=0
    for boddvoj in pole_bodov:
        x,y = boddvoj[0], boddvoj[1]
        # if x>=w or y>=w:
        #     continue
        data_farba[x][y][0] = R_farba[i]
        data_farba[x][y][1] = G_farba[i]
        data_farba[x][y][2] = B_farba[i]
        i+=1

    
def vytvor_priamku(i,j,pole):
    x0,y0 = pole[i][0], pole[i][1]
    x1,y1 = pole[j][0], pole[j][1]
    vektX, vektY = x1-x0, y1-y0

    return x0,y0,vektX,vektY

def najmensie_stvorec(patriace):
    global w,h, vykreslenie_stvorce
    xmean, ymean, sum0, xsum,ysum = 0, 0, 0, 0, 0
    xsum1,ysum1 = 0,0
    # xline=[]
    # vykresli(patriace)
    dlzka = len(patriace)
    # vysl = vyber_metodu(patriace)

    for bodymean in patriace:
        xmean += bodymean[1]
        ymean += bodymean[0]
        # xline.append(bodymean[1])
    xmean /= dlzka
    ymean /= dlzka

    for bodymean in patriace:
        x,y = bodymean[1], bodymean[0]
        xsum1 += (x-xmean)**2
        ysum1 += (y-ymean)**2
        sum0 += (x-xmean)*(y-ymean)
        xsum += (x-xmean)**2
        ysum += (y-ymean)**2

    if ysum1 > xsum1:
        n = sum0 / ysum
        p = xmean - n*ymean
        # y = n*y + p 
        yline=range(w)
        min_line_square=[]        
        for i in range(len(yline)):
            y1=yline[i]
            x1=yline[i]*n+p
            if round(x1)> w-1 or round(x1)<0:
                continue
            else:
                min_line_square.append([y1,x1])
        if vykreslenie_stvorce:
            Image.fromarray(data_farba,'RGB').show()
            for bods in min_line_square:
                data_farba[bods[0]][bods[1]][1] = 0
        return min_line_square

    else:
        try:
            m = sum0 / xsum
            b = ymean - m*xmean    
        except ZeroDivisionError as e:
            print('whats')

        # y = m*x + b
        xline=range(w)
        min_line_square=[]
        for i in range(len(xline)):
            x1=xline[i]
            y1=xline[i]*m+b
            if round(y1)> w-1 or round(y1)<0:
                continue
            else:
                min_line_square.append([y1,x1])
                
        if vykreslenie_stvorce:
            Image.fromarray(data_farba,'RGB').show()
            for bods in min_line_square:
                data_farba[bods[0]][bods[1]][1] = 0
        return min_line_square

def adaptive_treshold(pole_bodov):
    max_vzdial,min_vzdial = 0,1000
    vzdial_bodov_pole = []
    xlast,ylast = None,None        

    for idx in range(len(pole_bodov)):

                #adaptivny treshold
        if xlast != None and ylast != None:
            vzdial_bodov = ( (pole_bodov[idx][0]-xlast)**2 + (pole_bodov[idx][1]-ylast)**2 )**0.5
            vzdial_bodov_pole.append(int(vzdial_bodov))

            if max_vzdial < vzdial_bodov:
                max_vzdial = vzdial_bodov
            if min_vzdial > vzdial_bodov:
                min_vzdial = vzdial_bodov

        xlast,ylast = pole_bodov[idx][0],pole_bodov[idx][1]
            
    modus = statistics.mode(vzdial_bodov_pole)
    priemer = sum(vzdial_bodov_pole)/len(vzdial_bodov_pole)
    vzdial_bodov_pole.sort()
    median = vzdial_bodov_pole[len(vzdial_bodov_pole)//2]

    return median,priemer,modus,max_vzdial,min_vzdial


def lt_funkcia(pole_bodov,poradie):
    global min_vzdial
    new_pole, pomocne_pole = [],[]
    # print(average,median,modus)
    treshold = min_vzdial*15*5
    max_vzdialenost_dvochBodov = 3*treshold
    xlast,ylast = None,None

    if vykresluj:
        plt.ion()
    rovnake = False

    for idx in range(len(pole_bodov)):
        #pri mensom pocte bodov ako 2 -> pridavaj body
        if len(pomocne_pole) < 2 or rovnake: 
            zaciatok = True 
            #zistovanie ci 2 po sebe body nie su prilis daleko od seba -> ak je vzdialenost vacsia => rozsekne ich
            if len(pomocne_pole) > 0:
                xlast,ylast = pomocne_pole[-1][0],pomocne_pole[-1][1]
                vzdial_bodov = ( (pole_bodov[idx][0]-xlast)**2 + (pole_bodov[idx][1]-ylast)**2 )**0.5
                if vzdial_bodov > max_vzdialenost_dvochBodov:
                    if vykresluj:
                        plt.axis([0,w,0,h])
                        for bods in pole_bodov:
                            plt.plot(bods[0],bods[1],',b')
                        plt.plot(xlast,ylast,'.r')
                        plt.pause(0.5)
                        plt.clf()
                    new_pole.append(pomocne_pole)
                    pomocne_pole = []
                    pomocne_pole.append(pole_bodov[idx])
                    
                    if len(pomocne_pole) >= 1:
                        pocet_rovnakych_bodov = 0
                        for idx_rovnake in pomocne_pole:
                            if idx_rovnake == pole_bodov[idx] :
                                pocet_rovnakych_bodov += 1
                        if pocet_rovnakych_bodov == len(pomocne_pole):
                            pomocne_pole.append(pole_bodov[idx])
                            rovnake = True
                        else:
                            rovnake = False

                    if vykresluj:
                        plt.plot(pole_bodov[idx][0],pole_bodov[idx][1],'.r')
                    continue
            if len(pomocne_pole) >= 1:
                pocet_rovnakych_bodov = 0
                for idx_rovnake in pomocne_pole:
                    if idx_rovnake == pole_bodov[idx] :
                        pocet_rovnakych_bodov += 1
                if pocet_rovnakych_bodov == len(pomocne_pole):
                    pomocne_pole.append(pole_bodov[idx])
                    rovnake = True
                    continue
                else:
                    rovnake = False
            
            pomocne_pole.append(pole_bodov[idx])
            if vykresluj:
                # if len(pomocne_pole) == 2:
                plt.axis([0,w,0,h])
                for bods in pole_bodov:
                    plt.plot(bods[0],bods[1],',b')
                plt.plot(pole_bodov[idx][0],pole_bodov[idx][1],'.r')
            continue
        

        #pri mensom pocte bodov ako 3 -> priamka iba spojnicou 2 bodoch
        #pri vascom premietne body cez najmensie stvorce

        # prva iteracia vytvorenia cez najmensie stvorce pre (pri rozdeleni bodov..)
        if zaciatok:
            stvorce_body = najmensie_stvorec(pomocne_pole)
            zaciatok = False

        # vzdialenost bodu od priamky 
        x0,y0,vektX,vektY = vytvor_priamku(0,-1,stvorce_body)
        normal = (vektX**2 + vektY**2)**0.5        
        rx, ry = pole_bodov[idx][0] - x0, pole_bodov[idx][1] - y0
        ndx, ndy = -vektY/normal ,vektX/normal
        vzdialenost_bod_priamka = abs(ndx*rx + ndy*ry)
        # vzdialenost 2 po sebe bodov
        vzdial_bodov = ( (pole_bodov[idx][0]-xlast)**2 + (pole_bodov[idx][1]-ylast)**2 )**0.5

        #pridava aj bod bez vytvorenia stvorcov - ak nie je v tresholde -> spocita spatne stvorce a skusi znova....
        if vzdialenost_bod_priamka > treshold/4 or vzdial_bodov > treshold/2:
            # print('stvorce',idx)
            stvorce_body = najmensie_stvorec(pomocne_pole)
            x0,y0,vektX,vektY = vytvor_priamku(0,-1,stvorce_body)
            normal = (vektX**2 + vektY**2)**0.5        
            rx, ry = pole_bodov[idx][0] - x0, pole_bodov[idx][1] - y0
            ndx, ndy = -vektY/normal ,vektX/normal
            vzdialenost_bod_priamka = abs(ndx*rx + ndy*ry)
            # vzdialenost 2 po sebe bodov
            vzdial_bodov = ( (pole_bodov[idx][0]-xlast)**2 + (pole_bodov[idx][1]-ylast)**2 )**0.5
            

        #ak je vzdialenost bodu od priamky mensia < treshold => pridaj bod, inak terminuje pridavanie poli  
        #   
        #zistovanie ci 2 po sebe body nie su prilis daleko od seba -> ak je vzdialenost vacsia => rozsekne ich
        #tato cast pokracuje ak je privelka vzdialenost!
        if vzdialenost_bod_priamka > treshold or vzdial_bodov > 2*treshold:
            if vykresluj:
                plt.plot([stvorce_body[0][0],stvorce_body[-1][0]],[stvorce_body[0][1],stvorce_body[-1][1]],'m')
                plt.pause(0.5) 
                plt.clf()        
            new_pole.append(pomocne_pole)
            pomocne_pole = []
            pomocne_pole.append(pole_bodov[idx])
            # print('vzdialenost rozdelenia',vzdialenost_bod_priamka)
            if vykresluj:
                plt.plot(pole_bodov[idx][0],pole_bodov[idx][1],'.r')
            continue

        pomocne_pole.append(pole_bodov[idx])
        xlast,ylast = pomocne_pole[-1][0],pomocne_pole[-1][1]       
        if vykresluj:
            plt.plot(pole_bodov[idx][0],pole_bodov[idx][1],'.r')
            if vykresluj_priamku_vzdy:
                a = plt.plot([stvorce_body[0][0],stvorce_body[-1][0]],[stvorce_body[0][1],stvorce_body[-1][1]],'m')
                plt.setp(a,'visible',True)
                plt.pause(0.01)
                plt.setp(a,'visible',False)
            else:
                plt.pause(0.01)

    new_pole.append(pomocne_pole)
    if vykresluj:
        plt.plot([stvorce_body[0][0],stvorce_body[-1][0]],[stvorce_body[0][1],stvorce_body[-1][1]],'m')
        plt.pause(0.5)
        plt.close()
        plt.ioff()

    #1. koniec?
    #spoj 1. a -1. pole, ak su body blizko seba, ak maju podobne smernice



    #2. koniec?
    #spoj 1. a -1. pole, tak ze nechas prechadzat algoritmus pre 1. a posl pole este raz
    if poradie == 1:
        prve_pole = new_pole.pop(0)
        posl_pole = new_pole.pop(-1)
        pole_bodov2 = []

        for idx in posl_pole:
            pole_bodov2.append(idx)
        for idx in prve_pole:
            pole_bodov2.append(idx)

        # vykresli_pyplot(pole_bodov2,povodne_pole)
        # plt.show()

        spojene_polia = lt_funkcia(pole_bodov2,2)

        for idx in spojene_polia[::-1]:
            new_pole.insert(0,idx)

    return new_pole


#vykreslovacky
vykresluj = False
vykresluj_priamku_vzdy = False
vykreslenie_stvorce = False





#===========================================================================================        





#                           107 merani 
pole_bodov_vsetko = pickle.load(open('data_python/data_namerane_vsetkoLT.p','rb'))
# volny vyber merani z mnozinu [0:107] merani
pole_bodov2 = pole_bodov_vsetko[0:20]
povodne_pole2 = pole_bodov2[:]

#nastavenie max hodnot na vykreslovanie
maxX,maxY = 0,0
for bods in pole_bodov2[0]:
    x,y = bods[0],bods[1]
    maxX = x if x > maxX else maxX
    maxY = y if y > maxY else maxY
w = h = 50*(maxX//50 + 2) if maxX > maxY else 50*(maxY//50 + 2)
data_farba = np.zeros((h,w,3), dtype=np.uint8)


 # vytvorenie rozdradovacieho pola - sekvencne radenie bodov podla uhla
# algoritmus postupne zaraduje body jedneho stlpca(uhla) za sebou
pole_bodov = []
pocet_riadkov = len(pole_bodov2) -1
pocet_nenajdenych = 0
for idx1 in range(len(pole_bodov2[0])):
    for idx2 in range(len(pole_bodov2)):
        if pole_bodov2[idx2][idx1][1] != 0 and pole_bodov2[idx2][idx1][0] != 0:
            pole_bodov.append([pole_bodov2[idx2][idx1][0],pole_bodov2[idx2][idx1][1]])
povodne_pole = pole_bodov[:]

#filtrovanie totoznych bodov
vysledok_pole_filt_totoz = []
for idx_bod in range(len(pole_bodov)):
    if idx_bod == 0 :
        vysledok_pole_filt_totoz.append(pole_bodov[idx_bod])
        continue
    najdene = False
    for idx_bod2 in range(len(pole_bodov)):
        if idx_bod != idx_bod2:
            if pole_bodov[idx_bod] == pole_bodov[idx_bod2]:
                najdene = True
    if najdene == False:
        vysledok_pole_filt_totoz.append(pole_bodov[idx_bod])
pole_bodov = vysledok_pole_filt_totoz[:]
povodne_pole = pole_bodov[:]


#inicializacia tresholdu
aver,median,modus,max_vzdial,min_vzdial = adaptive_treshold(pole_bodov)
if min_vzdial == 0:
    min_vzdial = 1


#funkcia Line tracking
nove_pole = lt_funkcia(pole_bodov,1)


farba = ['b','g','r','c','m','y','k']
i=0
plt.figure()
for idx in nove_pole:
    vykresli_pyplot_farba(idx,povodne_pole,farba[i],'.')
    i += 1    
    if i == len(farba):
        i=0
plt.title('Line Tracking')        
plt.show()





