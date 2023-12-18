


import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import math
import random 
import pickle
import statistics
import random


#===========================================================================================


def random_body(pole):
    x,y = 0,0
    max = len(pole)
    while x==y:
        x,y = random.randrange(0,max), random.randrange(0,max)
    return x,y

def firs_last_max(pole_bodov):
    '''Funkcia sluzi na najdenie najvzdialenejsich bodov pointclodu, nasledne sa tie pouzivaju v dalsom 
        postupe pre split&merge algoritmus, volitelna -> heuristicke volenie pociatocnych bodov, lepsie? 
    '''
    vzdialenost_max = 0
    idx0,idy0,idx1,idy1 =0,0,0,0
    for i in range(len(pole_bodov)):
        x0,y0 = pole_bodov[i][0],pole_bodov[i][1]
        for j in range(len(pole_bodov)):
            if i != j:
                x1,y1 = pole_bodov[j][0],pole_bodov[j][1]
                vzdialenost = (x1-x0)**2+(y1-y0)**2
                if vzdialenost > vzdialenost_max:
                    vzdialenost_max = vzdialenost
                    idx0,idy0 = x0,y0
                    idx1,idy1 = x1,y1
    #najdene prvy budu prvy a posl v poli???
    return idx0,idy0,idx1,idy1


def vykresli_pyplot(vysvietene_pole,povodne_pole):
    global w,h
    plt.figure()
    for boddvoj in povodne_pole:
        x,y = boddvoj[1], boddvoj[0]
        plt.axis([0,w,0,h])
        plt.plot(x,512-y,',b')

    for boddvoj in vysvietene_pole:
        x,y = boddvoj[1], boddvoj[0]
        plt.axis([0,w,0,h])
        plt.plot(x,512-y,'.r')

def vykresli_pyplot_farba(vysvietene_pole,povodne_pole,farba):
    global w,h
    # for boddvoj in povodne_pole:
    #     x,y = boddvoj[1], boddvoj[0]
    #     plt.axis([0,w,0,h])
    #     plt.plot(x,512-y,',b')

    for boddvoj in vysvietene_pole:
        x,y = boddvoj[1], boddvoj[0]
        plt.axis([0,w,0,h])
        plt.plot(x,512-y, marker='.', color=farba)        


def zorad_pole(pole_bodov):
    idx0_p0,idy0_p0,idx1_p0,idy1_p0 = firs_last_max(pole_bodov)
    i,k0,k1 = 0, None, None
    for body in pole_bodov:
        x,y = body[0],body[1]
        if x==idx0_p0 and y==idy0_p0:
            k0=i
        if x==idx1_p0 and y==idy1_p0:
            k1=i
        i+=1

    if k1==None or k0==None:
        print('Chyba first-last')

    if pole_bodov[0] == pole_bodov[1]:
        return pole_bodov

    try:
        pole_bodov[0], pole_bodov[k0] = pole_bodov[k0], pole_bodov[0]
        pole_bodov[-1], pole_bodov[k1] = pole_bodov[k1], pole_bodov[-1]
    except TypeError as e:
        print('what')

    return pole_bodov
    
def vytvor_priamku(i,j,pole):
    x0,y0 = pole[i][0], pole[i][1]
    x1,y1 = pole[j][0], pole[j][1]
    vektX, vektY = x1-x0, y1-y0

    return x0,y0,vektX,vektY

def vyrataj_chybu(pole):
    global w,h 
    if len(pole)>1:
        zorad_pole(pole)  
        max_chyba = 0

        x0,y0,vektX,vektY = vytvor_priamku(0,-1,pole)
        normal = (vektX**2 + vektY**2)**0.5

        for bod in pole:            
            rx, ry = bod[0] - x0, bod[1] - y0
            ndx, ndy = -vektY/normal ,vektX/normal
            vzdialenost_bod_priamka = abs(ndx*rx + ndy*ry)
            if vzdialenost_bod_priamka > max_chyba:
                max_chyba = vzdialenost_bod_priamka

        return int(max_chyba)
    else:
        return 0


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


def kolinear(pole):
    global aver,min_vzdial
    treshold_vzdialenost_priamok = 50*min_vzdial
    tolerancna_medz_so_smernicou = 30*min_vzdial
    tolerancna_medz_bez = -15*min_vzdial
    treshold_body = 60*min_vzdial
    
    working = True
    
    dlzka = len(pole)
    if dlzka == 1:
        working = False
    if working and dlzka>1:
        for idx in range(len(pole)):
            if working:
                for jdx in range(len(pole)):
                    if idx != jdx:    
                        if len(pole[idx]) == 0 and len(pole[jdx]) == 0:
                            continue
                        #-----------------------------------------------------------------------------------------------------                 
                        # Pre 2 body -> hladanie body v dostatocnej vzdialenosti
                        if len(pole[idx]) == 1 and len(pole[jdx]) == 1:
                            vektX, vektY = pole[jdx][0][0]-pole[idx][0][0], pole[jdx][0][1]-pole[idx][-1][1]
                            vzdialenostDvochBodov = ( vektX**2 + vektY**2 )**0.5
                            if vzdialenostDvochBodov < treshold_body:
                                index1=idx
                                index2=jdx
                                pole_new=[]                              
                                for indx in range(len(pole)):
                                    pomoc_pole=[]
                                    #vytvara uplne nove cele pole do kt najskor kopiruje body nespajane,
                                    # a nasledne tie polia kt chce spajat -> 2. podmienka
                                    if indx!=index1 and indx!=index2:
                                        for i in range(len(pole[indx])):
                                            pomoc_pole.append(pole[indx][i])
                                        pole_new.append(pomoc_pole)
                                    elif indx==index1:
                                        for i in range(len(pole[index1])):
                                            pomoc_pole.append(pole[index1][i])
                                        for j in range(len(pole[index2])):
                                            pomoc_pole.append(pole[index2][j])                                        
                                        if len(pomoc_pole) > 1:                                        
                                            pole_new.append(zorad_pole(pomoc_pole))  
                                        else:
                                            pole_new.append(pomoc_pole)                                 
                                working = False
                                break

                        else :
                            # V pripade, ze jeden z pola je vektor, a druhy je bod
                            if len(pole[idx]) == 1 or len(pole[jdx]) == 1:
                                v_tolerancii = False
                                if len(pole[jdx]) == 1 :
                                    vektor = pole[idx][:]
                                    bod = pole[jdx]
                                else:
                                    vektor = pole[jdx][:]
                                    bod = pole[idx]

                                for bods in vektor:
                                    try :
                                        vektX, vektY = bods[0]-bod[0][0], bods[1]-bod[0][1]
                                    except TypeError as e:
                                        print('what')
                                    vzdialenostBodVektor = ( vektX**2 + vektY**2 )**0.5
                                    if vzdialenostBodVektor < treshold_body:
                                        v_tolerancii=True
                                if v_tolerancii:
                                    index1=idx
                                    index2=jdx
                                    pole_new=[]                              
                                    for indx in range(len(pole)):
                                        pomoc_pole=[]
                                        #vytvara uplne nove cele pole do kt najskor kopiruje body nespajane,
                                        # a nasledne tie polia kt chce spajat -> 2. podmienka
                                        if indx!=index1 and indx!=index2:
                                            for i in range(len(pole[indx])):
                                                pomoc_pole.append(pole[indx][i])
                                            pole_new.append(pomoc_pole)
                                        elif indx==index1:
                                            for i in range(len(pole[index1])):
                                                pomoc_pole.append(pole[index1][i])
                                            for j in range(len(pole[index2])):
                                                pomoc_pole.append(pole[index2][j])                                        
                                            if len(pomoc_pole) > 1:                                        
                                                pole_new.append(zorad_pole(pomoc_pole))  
                                            else:
                                                pole_new.append(pomoc_pole)                                 
                                    working = False
                                    break

                            else:
                                # Pre vektory -> postup spajania podla kolinearity
                                #-----------------------------------------------------------------------------------------------------
                                #skontrolovat kolinearitu
                                # vektor1 . vektor2 => cos=1 tak maju podobnu smernicu                         
                                try:
                                    vekt1X, vekt1Y = pole[idx][0][0]-pole[idx][-1][0], pole[idx][0][1]-pole[idx][-1][1]
                                    vekt2X, vekt2Y = pole[jdx][0][0]-pole[jdx][-1][0], pole[jdx][0][1]-pole[jdx][-1][1]
                                except TypeError as e:
                                    print('TypeError???')
                                norm1 = (vekt1X**2 + vekt1Y**2)**0.5
                                norm2 = (vekt2X**2 + vekt2Y**2)**0.5
                                sucin = abs(vekt1X*vekt2X + vekt1Y*vekt2Y)/(norm1*norm2)

                                if sucin > 0.95:
                                    #-----------------------------------------------------------------------------------------------------
                                    #skontrolovat vzajomnu vzdialenost priamok
                                    #vseobecna rovnica priamky? z toho zistit vzdialenosti danych priamok                
                                    # -toto patri do podmienky sucin > 0.99 ktora zvrhava k sebe priamky s podobnou smernicou- 
                                    # d = |normXY . r|
                                    rx, ry = pole[jdx][-1][0] - pole[idx][0][0], pole[jdx][-1][1] - pole[idx][0][1]
                                    ndx, ndy = -vekt1Y/norm1 ,vekt1X/norm1
                                    vzdialenost_priamok = abs(ndx*rx + ndy*ry)

                                    if vzdialenost_priamok < treshold_vzdialenost_priamok:
                                        v_tolerancii = False
                                        dlzka1 = ( (pole[idx][-1][0]-pole[idx][0][0])**2 + (pole[idx][-1][1]-pole[idx][0][1])**2 )**0.5 
                                        dlzka2 = ( (pole[jdx][-1][0]-pole[jdx][0][0])**2 + (pole[jdx][-1][1]-pole[jdx][0][1])**2 )**0.5                                 

                                        if dlzka2 > dlzka1:
                                            #zaciatok a koniec mensej priamky - zoradene 
                                            x0,x1,y0,y1 = pole[idx][0][1], pole[idx][-1][1],pole[idx][0][0], pole[idx][-1][0]

                                            #zaciatok a koniec vacsej priamky - zoradene 
                                            X0,X1 = (pole[jdx][0][1], pole[jdx][-1][1]) if pole[jdx][0][1] < pole[jdx][-1][1] else (pole[jdx][-1][1],pole[jdx][0][1])
                                            Y0,Y1 = (pole[jdx][0][0], pole[jdx][-1][0]) if pole[jdx][0][0] < pole[jdx][-1][0] else (pole[jdx][-1][0],pole[jdx][0][0])

                                        else:
                                            #zaciatok a koniec mensej priamky - zoradene         
                                            x0,x1,y0,y1 = pole[jdx][0][1], pole[jdx][-1][1],pole[jdx][0][0], pole[jdx][-1][0]

                                            #zaciatok a koniec vasej priamky - zoradene 
                                            X0,X1 = (pole[idx][0][1], pole[idx][-1][1]) if pole[idx][0][1] < pole[idx][-1][1] else (pole[idx][-1][1],pole[idx][0][1])
                                            Y0,Y1 = (pole[idx][0][0], pole[idx][-1][0]) if pole[idx][0][0] < pole[idx][-1][0] else (pole[idx][-1][0],pole[idx][0][0])


                                        #zistenie ci mensia usecka ma okrajove body v "tele tej vacsej"
                                        #zistenie ci su okrajove body v "tolerancii"                                
                                        if (x0 > X0-tolerancna_medz_so_smernicou and x0 < X1+tolerancna_medz_so_smernicou) and (y0 > Y0-tolerancna_medz_so_smernicou and y0 < Y1+tolerancna_medz_so_smernicou):
                                            v_tolerancii = True
                                        elif (x1 > X0-tolerancna_medz_so_smernicou and x1 < X1+tolerancna_medz_so_smernicou) and (y1 > Y0-tolerancna_medz_so_smernicou and y1 < Y1+tolerancna_medz_so_smernicou): 
                                            v_tolerancii = True

                                        if v_tolerancii:
                                            index1=idx
                                            index2=jdx
                                            pole_new=[]                              
                                            for indx in range(len(pole)):
                                                pomoc_pole=[]
                                                #vytvara uplne nove cele pole do kt najskor kopiruje body nespajane,
                                                # a nasledne tie polia kt chce spajat -> 2. podmienka
                                                if indx!=index1 and indx!=index2:
                                                    for i in range(len(pole[indx])):
                                                        pomoc_pole.append(pole[indx][i])
                                                    pole_new.append(pomoc_pole)
                                                elif indx==index1:
                                                    for i in range(len(pole[index1])):
                                                        pomoc_pole.append(pole[index1][i])
                                                    for j in range(len(pole[index2])):
                                                        pomoc_pole.append(pole[index2][j])

                                                    if len(pomoc_pole) > 1:                                        
                                                        pole_new.append(zorad_pole(pomoc_pole))  
                                                    else:
                                                        pole_new.append(pomoc_pole)
                                            
                                            working = False
                                            break
                                
                                #-----------------------------------------------------------------------------------------------------               
                                #hodnotenie ak nemaju podobnu smernicu, ci sa jeden nenachadza v druhom???
                                v_tolerancii = False
                                dlzka1 = ( (pole[idx][-1][0]-pole[idx][0][0])**2 + (pole[idx][-1][1]-pole[idx][0][1])**2 )**0.5 
                                dlzka2 = ( (pole[jdx][-1][0]-pole[jdx][0][0])**2 + (pole[jdx][-1][1]-pole[jdx][0][1])**2 )**0.5 

                                if dlzka2 > dlzka1:
                                    #zaciatok a koniec mensej priamky - zoradene 
                                    x0,x1,y0,y1 = pole[idx][0][1], pole[idx][-1][1],pole[idx][0][0], pole[idx][-1][0]

                                    #zaciatok a koniec vacsej priamky - zoradene 
                                    X0,X1 = (pole[jdx][0][1], pole[jdx][-1][1]) if pole[jdx][0][1] < pole[jdx][-1][1] else (pole[jdx][-1][1],pole[jdx][0][1])
                                    Y0,Y1 = (pole[jdx][0][0], pole[jdx][-1][0]) if pole[jdx][0][0] < pole[jdx][-1][0] else (pole[jdx][-1][0],pole[jdx][0][0])

                                else:
                                    #zaciatok a koniec mensej priamky - zoradene         
                                    x0,x1,y0,y1 = pole[jdx][0][1], pole[jdx][-1][1],pole[jdx][0][0], pole[jdx][-1][0]

                                    #zaciatok a koniec vasej priamky - zoradene 
                                    X0,X1 = (pole[idx][0][1], pole[idx][-1][1]) if pole[idx][0][1] < pole[idx][-1][1] else (pole[idx][-1][1],pole[idx][0][1])
                                    Y0,Y1 = (pole[idx][0][0], pole[idx][-1][0]) if pole[idx][0][0] < pole[idx][-1][0] else (pole[idx][-1][0],pole[idx][0][0])

                                #zistenie ci mensia usecka ma okrajove body v "tele tej vacsej"
                                #zistenie ci su okrajove body v "tolerancii"                                
                                
                                if (x0 > X0-tolerancna_medz_bez and x0 < X1+tolerancna_medz_bez) and (y0 > Y0-tolerancna_medz_bez and y0 < Y1+tolerancna_medz_bez) and (x1 > X0-tolerancna_medz_bez and x1 < X1+tolerancna_medz_bez) and (y1 > Y0-tolerancna_medz_bez and y1 < Y1+tolerancna_medz_bez):
                                    v_tolerancii = True
                                
                                # plt.clf()

                                if v_tolerancii:
                                    index1=idx
                                    index2=jdx
                                    pole_new=[]                              
                                    for indx in range(len(pole)):
                                        pomoc_pole=[]
                                        #vytvara uplne nove cele pole do kt najskor kopiruje body nespajane,
                                        # a nasledne tie polia kt chce spajat -> 2. podmienka
                                        if indx!=index1 and indx!=index2:
                                            for i in range(len(pole[indx])):
                                                pomoc_pole.append(pole[indx][i])
                                            pole_new.append(pomoc_pole)
                                        elif indx==index1:
                                            for i in range(len(pole[index1])):
                                                pomoc_pole.append(pole[index1][i])
                                            for j in range(len(pole[index2])):
                                                pomoc_pole.append(pole[index2][j])                                        
                                            if len(pomoc_pole) > 1:                                        
                                                pole_new.append(zorad_pole(pomoc_pole))  
                                            else:
                                                pole_new.append(pomoc_pole) 
                                    
                                    working = False
                                    break
    if working==False:
        return kolinear(pole_new)
    else:
        return pole


def delenie_poly_koniec(pole_skupin_bodov,zobrazenie_delenia=False):
    global w,h,min_vzdial,zobraz_postupnost_bodov

    treshold_strihania = 15*min_vzdial
    if zobrazenie_delenia:
        plt.ion()

    # rozdelenie do postupnosti bodov vnutri bloku
    i=0
    for pole in pole_skupin_bodov:
        if len(pole)==0:
            pole_skupin_bodov.pop(i)
            continue
        if len(pole)==1:
            continue
        zorad_pole(pole)
        dlzkaX,dlzkaY = abs( pole[-1][1]-pole[0][1] ), abs( pole[-1][0]-pole[0][0] )

        if zobraz_postupnost_bodov:
            plt.ion()
            plt.axis([0,w,0,h])
            for bods in pole:
                plt.plot(bods[0],bods[1],',b')
            for bods in pole:
                plt.plot(bods[0],bods[1],'.b')
                plt.pause(0.2)
            plt.close()

        if dlzkaX > dlzkaY: 
            for index1 in range(len(pole)):
                minimum = pole[index1][1]
                idx_min = index1
                for index2 in range(index1,len(pole)):
                    if minimum > pole[index2][1]:
                        minimum,idx_min = pole[index2][1],index2
                pole[index1], pole[idx_min] = pole[idx_min], pole[index1]
        else:
            for index1 in range(len(pole)):
                minimum = pole[index1][0]
                idx_min = index1
                for index2 in range(index1,len(pole)):
                    if minimum > pole[index2][0]:
                        minimum,idx_min = pole[index2][0],index2
                pole[index1], pole[idx_min] = pole[idx_min], pole[index1]

        if zobraz_postupnost_bodov:
            plt.axis([0,w,0,h])
            for bods in pole:
                plt.plot(bods[0],bods[1],',r')
            for bods in pole:
                plt.plot(bods[0],bods[1],'.r')
                plt.pause(0.2)
            plt.close()

        i+=1

    new_pole=[]                     
    #rozdeli si cele pole segmentovanych casti na jednotlive celky
    for pole in pole_skupin_bodov:
        indexi_delenia = []        
        
        if len(pole) < 2:
            new_pole.append(pole)
            continue
        if len(pole) == 2:
            vzdialenost = ( (pole[-1][1] - pole[0][1])**2 + (pole[-1][0] - pole[0][0])**2 )**0.5
            if vzdialenost > treshold_strihania:
                new_pole.append([pole[0]])
                new_pole.append([pole[1]])
            else:
                new_pole.append(pole)
            continue

        #param vyjadrenie deliacej pramky cervenej
        dx = pole[-1][1]-pole[0][1]
        dy = pole[-1][0]-pole[0][0]
        #znormovanie vektora -> velkost=1
        # X = x + dx*t
        # Y = y + dy*t
        norm = (dx**2+dy**2)**0.5
        dx = dx/norm
        dy = dy/norm
        ndx = -dy
        ndy = dx
        CPx = ndx
        CPy = ndy

        # zistenie ci dany bod ma body na oboch stranach kolmo na kolmicu
        for idx_bodu in range(len(pole)):
            X,Y  = pole[idx_bodu][1], pole[idx_bodu][0]
            body_kamosov = []

            if zobrazenie_delenia:
                plt.axis([0,w,0,h])
                for pole2 in pole_skupin_bodov:
                    for bods in pole2:
                        plt.plot(bods[0],bods[1],',b')
                for bods in pole:
                    plt.plot(bods[0],bods[1],'.b')
                plt.plot(pole[0][0],pole[0][1],'.r')
                plt.plot(pole[-1][0],pole[-1][1],'.r')

            #vytvorenie kamosov s ktorymi sa porovnava
            for idx_porovn in range(len(pole)):            
                if idx_bodu != idx_porovn: 
                    vzdialenost = ( (pole[idx_bodu][1] - pole[idx_porovn][1])**2 + (pole[idx_bodu][0] - pole[idx_porovn][0])**2 )**0.5
                    if vzdialenost < treshold_strihania: 
                        body_kamosov.append(pole[idx_porovn])
            
            if zobrazenie_delenia:
                for i in range(-100,100):
                    plt.plot(pole[idx_bodu][0]+ndy*i,pole[idx_bodu][1]+ndx*i, ',g')
                plt.plot(pole[idx_bodu][0],pole[idx_bodu][1],'.r')
                plt.show()
                plt.pause(0.01)
                plt.clf()

            # pomocou priamky prechadzajucej danym bodom urcime ci dany kamos lezi ja jednej alebo druhej strane
            left,right = 0,0    
            if idx_bodu != 0 and idx_bodu != len(pole)-1:
                for kamosi in body_kamosov:
                    x,y = kamosi[1], kamosi[0]
                    bodX,bodY = x-X, y-Y    
                    znamienko = CPx*bodY - bodX*CPy
                    if znamienko > 0 :
                        left += 1 
                    else:
                        right += 1
                if left == 0 or right == 0:
                    indexi_delenia.append(idx_bodu)
                    # print('delenie')

            else:
                if len(body_kamosov) == 0:
                    indexi_delenia.append(idx_bodu)

        # if idx_bodu == len(pole)-1:
        #     indexi_delenia.append(idx_bodu)

        if len(indexi_delenia)>0:
            last_index=0
            for idxDel in indexi_delenia:
                if idxDel != 0:
                    if idxDel-last_index >= 1 :
                        if len(pole[last_index:idxDel]) > 1:
                            new_pole.append(zorad_pole(pole[last_index:idxDel]))
                        else:
                            new_pole.append(pole[last_index:idxDel])
                        last_index = idxDel
                    else:
                        last_index = idxDel
                else:
                    new_pole.append([pole[0]])
                    last_index = 1
            #podmienka -> kvoli nezoradovani pola pre pripad 1rozmerneho pola
            if len(pole[last_index:]) > 1:
                new_pole.append(zorad_pole(pole[last_index:]))
            else:
                new_pole.append(pole[last_index:])
        else:
            new_pole.append(zorad_pole(pole))
    
    plt.ioff()
    return new_pole


def vykresli(pole_bodov):
    global w,data_farba
    R_farba = []
    G_farba = []
    B_farba = []
    for boddvoj in pole_bodov:
        x,y = boddvoj[0], boddvoj[1]
        # if x>=w or y>=w:
        #     continue
        R_farba.append(data_farba[x][y][0])
        G_farba.append(data_farba[x][y][1])
        B_farba.append(data_farba[x][y][2])
        data_farba[x][y][0] = 255
        data_farba[x][y][1] = 255
        data_farba[x][y][2] = 255
        
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



def vyber_polia(polia,vysledok_pole):
    for ntica in polia:
        if tuple == type(ntica):
            vyber_polia(ntica,vysledok_pole)
        else:
            vysledok_pole.append(ntica)
    return


def vyber_metodu(pole):
    maxX,maxY = 0,0
    
    for body in pole:
        y = body[0]
        x = body[0]
 

def najmensie_stvorec(patriace):
    global w,h, vykreslenie_ransac
    xmean, ymean, sum0, xsum,ysum = 0, 0, 0, 0, 0
    xsum1,ysum1 = 0,0
    # xline=[]
    # vykresli(patriace)
    dlzka = len(patriace)

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
                min_line_square.append([round(y1),round(x1)])
                # data_farba[round(y1)][round(x1)][1] = 255 if vykreslenie_ransac else 0
        # if vykreslenie_ransac:
            # Image.fromarray(data_farba,'RGB').show()
            # for bods in min_line_square:
                # data_farba[bods[0]][bods[1]][1] = 0
        return min_line_square

    else:
        m = sum0 / xsum
        b = ymean - m*xmean    
        # y = m*x + b
        xline=range(w)
        min_line_square=[]
        for i in range(len(xline)):
            x1=xline[i]
            y1=xline[i]*m+b
            if round(y1)> w-1 or round(y1)<0:
                continue
            else:
                min_line_square.append([round(y1),round(x1)])
                data_farba[round(y1)][round(x1)][1] = 255 if vykreslenie_ransac else 0
                
        if vykreslenie_ransac:
            Image.fromarray(data_farba,'RGB').show()
            for bods in min_line_square:
                data_farba[bods[0]][bods[1]][1] = 0
        return min_line_square




def ransac(pole_bodov,opakovanie,min_pocet_bodov):
    global w,h,min_vzdial
    omega = 0.2
    z = 0.95

    treshold = .7*min_vzdial
    treshold2 = 5*min_vzdial
    working = True
    # vypocita pocet pokusov
    k = math.ceil( (math.log(1-z)) / (math.log(1-omega**2)) )
    k -= opakovanie
    

    for idx in range(k):
        pocet_patriacich = 0
        body_patriace=[]
        najlepsie_body = []        
        # najde dva body, (zatial random / mozno najblizsie body)
        i0, i1 = random_body(pole_bodov)        
        i1 = i0+10
        if i1>=len(pole_bodov):
            i1=i0-10
        
        # z bodov nasledne vytvara vektor/priamku a k nej zistuje kolko bodov je v 
        # "trehsold" vzdialenosti od priaky zisti kolko bodov patri danej priamke s error<treshold
        #pomocka proti rekurzivnemu cykleniu???

        #poistka...
        dlzka = len(pole_bodov)
        if dlzka < min_pocet_bodov:
            return pole_bodov

        x0,y0,vektX,vektY = vytvor_priamku(i0,i1,pole_bodov)
        normal = (vektX**2 + vektY**2)**0.5
        
        for bod in pole_bodov:            
            rx, ry = bod[0] - x0, bod[1] - y0
            ndx, ndy = -vektY/normal ,vektX/normal
            vzdialenost_bod_priamka = abs(ndx*rx + ndy*ry)
            if vzdialenost_bod_priamka < treshold:
                pocet_patriacich+=1
                body_patriace.append([bod[0],bod[1]])
            
        if pocet_patriacich > min_pocet_bodov:
            # print('Najdenie bodov! - 1.')
            # vykresli(body_patriace)
            working = False
            # najlepsie_body = body_patriace[:]
            # break

        if working==False:
            #nasledne pomocou LNS vytvorit lepsi preklad bodov a porovnat to pomocou toho
            square_line_body = najmensie_stvorec(body_patriace)
            pocet_lepsich_patriacich=0
            body_patriace_lepsie = []
            if len(square_line_body) < min_pocet_bodov:
                working=True
                pocet_patriacich = 0
                body_patriace=[]
                najlepsie_body = []
                continue
            x0,y0,vektX,vektY = vytvor_priamku(0,-1,square_line_body)
            normal = (vektX**2 + vektY**2)**0.5

            for bod in pole_bodov:            
                rx, ry = bod[0] - x0, bod[1] - y0
                ndx, ndy = -vektY/normal ,vektX/normal
                vzdialenost_bod_priamka = abs(ndx*rx + ndy*ry)
                if vzdialenost_bod_priamka < treshold2:
                    pocet_lepsich_patriacich+=1
                    body_patriace_lepsie.append([bod[0],bod[1]])

            # vykresli(body_patriace_lepsie)
            if pocet_lepsich_patriacich >= pocet_patriacich:
                # print('Najdenie bodov! - 2.')
                najlepsie_body = body_patriace_lepsie[:]
                break                
            else:
                # print('Plany poplach!')
                working=True
                pocet_patriacich = 0
                body_patriace=[]
                najlepsie_body = []
            

    if working==False:
        new_pole_bodov=[]
        for bod in pole_bodov:
            patrit = False
            for body_patr in najlepsie_body:
                if body_patr == bod:
                    patrit=True
            if patrit==False:
                new_pole_bodov.append(bod)
        
        if len(new_pole_bodov) > min_pocet_bodov:
            return ransac(new_pole_bodov,idx,min_pocet_bodov), najlepsie_body
        else:
            return new_pole_bodov, najlepsie_body

    return pole_bodov





#===========================================================================================



# vybratie nazvu obrazka z priecinka imgs
imagee = np.asarray(Image.open("imgs/pointCloud4.png"))
w,h = np.shape(imagee) 


# vykreslovanie
vykreslenie_ransac = False
zobraz_postupnost_bodov = False


#prevedenie obrazka do pola bodov
data_farba = np.zeros((h,w,3), dtype=np.uint8)
for i in range(w):
    for j in range(h):
        data_farba[i][j][2] = imagee[i][j]
maxi,maxj = np.shape(imagee)
pole_bodov,povodne_pole = [],[]
for i in range(maxi):
    for j in range(maxj):
        if imagee[i][j] == 255:
            pole_bodov.append([i,j])
            povodne_pole.append([i,j])

# inicializacia tresholdu
aver,median,modus,max_vzdial,min_vzdial = adaptive_treshold(pole_bodov)
minPocetBodov = round(len(povodne_pole)*0.1)

#funkcia ransac
rozdelene_polia = ransac(pole_bodov,0,minPocetBodov)
vysledok_pole = []
vyber_polia(rozdelene_polia,vysledok_pole)
vysledok_pole = vysledok_pole[::-1]


# postupne znizovanie min poctu bodov
error = vyrataj_chybu(vysledok_pole[-1])
rozdelene_polia_last = vysledok_pole[-1]
if error > 20*min_vzdial:
    while error > 20*min_vzdial :
        pole_bodov2 = vysledok_pole.pop(-1)
        if minPocetBodov != 1:
            minPocetBodov = round(minPocetBodov*0.5)

        rozdelene_polia2 = ransac(pole_bodov2,0,minPocetBodov)
        vysledok_pole2=[]
        vyber_polia(rozdelene_polia2,vysledok_pole2)
        if rozdelene_polia_last != rozdelene_polia2:
            for idx in vysledok_pole2[::-1]:
                if len(idx)>1:
                    vysledok_pole.append(idx)
            error = vyrataj_chybu(vysledok_pole[-1])
            rozdelene_polia_last = vysledok_pole[-1]
        else:
            vysledok_pole.append(vysledok_pole2)
            error = vyrataj_chybu(vysledok_pole[-1])
            rozdelene_polia_last = vysledok_pole[-1]




#Zobrazenie vysledku Ransac funkcie
farba = ['b','g','r','c','m','y','k']
i=0
plt.figure()
for idx in vysledok_pole:
    vykresli_pyplot_farba(idx,povodne_pole,farba[i])
    i += 1    
    if i == len(farba):
        i=0
plt.title('Ransac')         
plt.show()


# posprocessing ----------------------------------------------------------------------------------------


nove_pole = delenie_poly_koniec(vysledok_pole)
novsie_pole = kolinear(nove_pole)

farba = ['b','g','r','c','m','y','k']
i=0
plt.figure()
for idx in novsie_pole:
    vykresli_pyplot_farba(idx,povodne_pole,farba[i])
    i += 1    
    if i == len(farba)-1:
        i=0
plt.title('Ransac + Postprocessing')        
plt.show()


