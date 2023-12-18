

import numpy as np
from PIL import Image
import pickle
import matplotlib.pyplot as plt
import pickle
import math
import sys
import statistics

def vykresli_pyplot(vysvietene_pole,povodne_pole):
    plt.figure()
    for boddvoj in povodne_pole:
        x,y = boddvoj[0], boddvoj[1]
        plt.plot(x,y,'.b')

    for boddvoj in vysvietene_pole:
        x,y = boddvoj[0], boddvoj[1]
        plt.plot(x,y,'.r')
    plt.plot(0,0,'om')

def vykresli_pyplot_farba(vysvietene_pole,povodne_pole,farba,sign):
    # global w,h
    # for boddvoj in povodne_pole:
    #     x,y = boddvoj[1], boddvoj[0]
    #     plt.axis([0,w,0,h])
    #     plt.plot(x,512-y,',b')

    for boddvoj in vysvietene_pole:
        x,y = boddvoj[0], boddvoj[1]
        # plt.axis([0,w,0,h])
        plt.plot(x,y, marker=sign, color=farba)


def sef_alg(pole_bodov):
    #Lidar vykreslenie spravania
    global vykresluj_proces

    if vykresluj_proces:
        plt.ion()
        plt.axis([-1,3,-1.5,1.8])
        fi=0
        #nakreslenie vsetkych bodov
        for bod in pole_bodov:
            fi -= bod[0]
            r = bod[1]        
            x_cartes = -r*math.cos(fi)
            y_cartes = r*math.sin(fi)
            plt.plot(x_cartes,y_cartes,',b')
        plt.plot(0,0,'om')

        fi=0
        for idx in range(len(pole_bodov)):
            if pole_bodov[idx][1] == 0:
                fi -= pole_bodov[idx][0]
                continue
            fi -= pole_bodov[idx][0]
            r = pole_bodov[idx][1]        
            x_cartes = -r*math.cos(fi)
            y_cartes = r*math.sin(fi)

            # a = plt.plot([0,x_cartes],[0,y_cartes],'r')
            # a = plt.plot(x_cartes,y_cartes,'.r')
            # plt.setp(a,'visible',True)
            # plt.pause(0.01)
            # plt.setp(a,'visible',False)

        plt.ioff()
        plt.close()

    dmin = pole_bodov[0][1]
    polom_minuly = pole_bodov[0][1]

    #inicializacia tresholdu
    for i in range(1,len(pole_bodov)):
        if pole_bodov[i][1] == 0:
            continue
        polomer = pole_bodov[i][1]        
        vzdialenost_zmena = polomer - polom_minuly
        if dmin > abs(vzdialenost_zmena) and abs(vzdialenost_zmena)!= 0:
            dmin = abs(vzdialenost_zmena)
        polom_minuly = polomer

    dmax = dmin*150

    new_pole  = []
    pole_bodov_povodne = []
    pomocne_pole = []

    fi =0
    fi -= pole_bodov[0][0]
    r = pole_bodov[0][1]
    x_cartes = -r*math.cos(fi)
    y_cartes = r*math.sin(fi)
    pomocne_pole.append([x_cartes,y_cartes])
    polomer = pole_bodov[0][1]
    polom_minuly = polomer

    for i in range(1,len(pole_bodov)):    

        if pole_bodov[i][1] == 0:
            fi -= pole_bodov[i][0]
            continue
        polomer = pole_bodov[i][1]

        fi -= pole_bodov[i][0]
        r = pole_bodov[i][1]
        x_cartes = -r*math.cos(fi)
        y_cartes = r*math.sin(fi)
        pole_bodov_povodne.append([x_cartes,y_cartes])
        vzdialenost = abs(polomer - polom_minuly)
        

        if vzdialenost > dmax:
            new_pole.append(pomocne_pole)
            pomocne_pole = []
            pomocne_pole.append([x_cartes,y_cartes])
        else :
            pomocne_pole.append([x_cartes,y_cartes])
        polom_minuly = polomer

    new_pole.append(pomocne_pole)    

    # spojenie prveho a posledneho segmentu -> ak su zhodne
    vzdialenost = abs(new_pole[0][1][1] - new_pole[-1][-1][1])
    if vzdialenost < dmax:
        nuevo = new_pole[:]
        new_pole = []
        
        #spojenie prvy + posledny
        pomocne_pole = []
        for idx in nuevo[0]:
            pomocne_pole.append(idx)
        for idx in nuevo[-1]:
            pomocne_pole.append(idx)
        new_pole.append(pomocne_pole)

        #dospojenie ostatnych
        for idx in nuevo[1:-1]:
            new_pole.append(idx)

    return new_pole,pole_bodov_povodne


#===========================================================================================        

#                           107 merani 
pole_bodov_vsetko = pickle.load(open('data_python/data_namerane_polarne_vsetky.p','rb'))


#            Vybratie jednotliveho merania - prvok pola : [0-106]
pole_bodov2 = pole_bodov_vsetko[20:40]
povodne_pole2 = pole_bodov2[:]

vykresluj_proces = False

# vytvorenie rozdradovacieho pola - sekvencne radenie bodov podla uhla
# totozna rozradovacia skupina cyklov ako v pripade LT algoritmu
# postupne po stlpcoch (uhloch) uklada namerane body
pole_bodov = []
pocet_riadkov = len(pole_bodov2) -1
pocet_nenajdenych = 0
for idx1 in range(len(pole_bodov2[0])):
    for idx2 in range(len(pole_bodov2)):
        najdeny_bod = False
        if pole_bodov2[idx2][idx1][1] != 0 :
            najdeny_bod = True
            if idx2 != pocet_riadkov:
                pole_bodov.append([0,pole_bodov2[idx2][idx1][1]])
            else:
                # hodnota 0.017453 = 1° uhla 
                pole_bodov.append([0.017453,pole_bodov2[idx2][idx1][1]])
    if najdeny_bod != True:
        pole_bodov.append([0.017453,0])       
povodne_pole = pole_bodov[:]

# funkcia SEF
nove_pole,pole_bodov_povodne_kartezske = sef_alg(pole_bodov)



farba = ['b','g','r','c','m','y','k']
i=0
plt.figure()
for idx in nove_pole:
    vykresli_pyplot_farba(idx,povodne_pole,farba[i],'.')
    i += 1    
    if i == len(farba):
        i=0
plt.title('SEF')
plt.show()





