import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import math
import pickle
import sys
import statistics
import random

# sys.path.append('D:\VUT\FSI\Ing\DP\Python\delenieKoniec')
# from delenie_funkcie import kolinear,delenie_poly_koniec

#===========================================================================================

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

def vykresli_pyplot_farba(vysvietene_pole,povodne_pole,farba,sign):
    global w,h
    # for boddvoj in povodne_pole:
    #     x,y = boddvoj[1], boddvoj[0]
    #     plt.axis([0,w,0,h])
    #     plt.plot(x,512-y,',b')

    for boddvoj in vysvietene_pole:
        x,y = boddvoj[0], boddvoj[1]
        plt.axis([0,w,0,h])
        plt.plot(x,y, marker=sign, color=farba)

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

    pole_bodov[0], pole_bodov[k0] = pole_bodov[k0], pole_bodov[0]
    pole_bodov[-1], pole_bodov[k1] = pole_bodov[k1], pole_bodov[-1]

    return pole_bodov


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


def split(first_idx,last_idx,pole_bodov,farba):
    '''Rekurzivne definovana funkcia, deli dane pole bodov point cloudu do roznych casti podla deliacej krivky, kriteriom delenia je chyba 
       reprezentujuca najvacsiu vzdialenost mezdi bodom a danou krivkou
    '''
    global vykreslenie_kolmice_RG,min_vzdial

    treshold = 50*min_vzdial
    idx0,idy0 = first_idx
    idx1,idy1 = last_idx
    
    # max dlzka priamky
    vzdialenost = (((idx1-idx0)**2+(idy1-idy0)**2)**(0.5))

    #param vyjadrenie deliacej pramky cervenej
    dx = idx1-idx0
    dy = idy1-idy0
    #znormovanie vektora -> velkost=1
    # X = x + dx*t
    # Y = y + dy*t
    norm = (dx**2+dy**2)**0.5
    dx = (dx/norm)
    dy = (dy/norm)
    ndx = -dy
    ndy = dx
    
    x=idx0
    y=idy0

    if vykreslenie_kolmice_RG:
        plt.axis([0,w,0,h])
        # for idx in povodne_pole0:
        #     plt.plot(idx[1],w-idx[0],',b')
        # for idx in povodne_pole0:
        #     plt.plot(y,w-x,'.b',markersize=0.7)

        # 'vykreslenie' krivky cervenej - priamky p, spajajucu body
    
        for i in range(vzdialenost):
            x += dx
            y += dy
            plt.plot(y,w-x,'.r',markersize=1)
            # data_farba[round(x)][round(y)][1] = 0
            # data_farba[round(x)][round(y)][2] = 0

    indexX,indexY, Px,Py, vzdialenost_bod_max, vzdialenost_od_zac = max_vzdialenost(idx0,idy0,dx,dy,ndx,ndy,norm,pole_bodov)


    if vykreslenie_kolmice_RG:
        x=idx0
        y=idy0
        for i in range(vzdialenost):
            x += dx
            y += dy
            # data_farba[round(x)][round(y)][1] = 255
            # data_farba[round(x)][round(y)][2] = 255

    # rozdelenie pola bodov do dvoch casti - ak error je > treshold
    if vzdialenost_bod_max > treshold:
        pole1,pole2 = rozdel_pole(pole_bodov,indexX,indexY,Px,Py,farba)

        #rozdeli pole 1
        idx0_p1,idy0_p1,idx1_p1,idy1_p1 = firs_last_max(pole1)
        vysl_pole1 = split((idx0_p1,idy0_p1),(idx1_p1,idy1_p1),pole1,farba+90)

        #rozdeli pole 2
        idx0_p2,idy0_p2,idx1_p2,idy1_p2 = firs_last_max(pole2)
        vysl_pole2 = split((idx0_p2,idy0_p2),(idx1_p2,idy1_p2),pole2,farba+70)

        return vysl_pole1, vysl_pole2

    # ak je pole uz v poriadku, tak ho len zoradi a vyhodi - returne
    #ukoncenie a zoradenie pola
    idx0_p0,idy0_p0,idx1_p0,idy1_p0 = firs_last_max(pole_bodov)
    zorad_pole(pole_bodov)

    return pole_bodov


def max_vzdialenost(idx0,idy0,dx,dy,ndx,ndy,norm,pole_bodov):
    '''Funkcia nachadza najvzdialenjsi bod od priamky definovanej dvomi bodmi
    '''
    global min_vzdial
    #najdenie najvacsej vzdialenosti, najvzdialenejsieho bodu     
    vzdialenost_bod_max = 0
    # print(len(pole_bodov))
    
    for boddvoj in pole_bodov:
        xbod,ybod = boddvoj[0], boddvoj[1]
        # d = |normXY . r|        
        rx, ry = xbod-idx0, ybod-idy0
        vzdialenost_bod = abs(ndx*rx + ndy*ry)

        if vzdialenost_bod > vzdialenost_bod_max:
            vzdialenost_bod_max = vzdialenost_bod
            indexX, indexY = xbod, ybod
            vzdial_r = ( rx**2 + ry**2 )**0.5
            vzdialenost_od_zac = ( vzdial_r**2 - vzdialenost_bod_max**2 )**0.5
    # plt.plot(indexY,w-indexX,'.m',markersize=11)
    if vzdialenost_bod_max !=0:
        # print('Najvzdialenejsi bod',indexX,indexY,vzdialenost_bod_max,vzdialenost_od_zac)
                # data_farba[indexX][indexY][1] = 255
                # data_farba[indexX][indexY][0] = 0
        # Image.fromarray(data_farba,'RGB').show() 
        
        #Najdenie priesecnika P
        Px, Py = (idx0 + vzdialenost_od_zac*dx) , (idy0 + vzdialenost_od_zac*dy)
        # print('Vzdialenost',(Px**2+Py**2)**0.5)
                # data_farba[Px][Py][1] = 255
                # data_farba[Px][Py][0] = 0
        # Image.fromarray(data_farba,'RGB').show()
        
        #vykreslenie kolmice priamky
        velkost_kolmica = ((indexX-Px)**2 + (indexY-Py)**2)**0.5
        xP, yP = indexX, indexY
        if velkost_kolmica > 0.1:
            pdx, pdy = (Px-indexX)/velkost_kolmica , (Py-indexY)/velkost_kolmica
        else:
            return 0,0,0,0,0,0
            # print(pdx**2+pdy**2)
        #vykreslenie zelenej kolmice
        if vykreslenie_kolmice_RG:
            for i in range(round(vzdialenost_bod_max)):
                xP += pdx
                yP += pdy
                # data_farba[round(xP)][round(yP)][0] = 0
                # data_farba[round(xP)][round(yP)][2] = 0
                plt.plot(yP,w-xP,'.g',markersize=1)
            plt.show()
            plt.close()
            # Image.fromarray(data_farba,'RGB').show()
            xP, yP = indexX, indexY
            for i in range(round(vzdialenost_bod_max)):
                xP += pdx
                yP += pdy
                # data_farba[round(xP)][round(yP)][0] = 255
                # data_farba[round(xP)][round(yP)][2] = 255


        return indexX,indexY, Px,Py, vzdialenost_bod_max, vzdialenost_od_zac
    else:
        return 0,0,0,0,0,0
    
def rozdel_pole(pole,indexX,indexY,Px,Py,farba):
    '''Funkcia deliaca pole na 2 podla prislusnosti k strane ku ktorej sa v zavislosti na priamku prechadzajucu bodom indexX,indexY a bodom Px,Py nachadza
    '''
    CPx = Px-indexX
    CPy = Py-indexY
    pole1=[]
    pole2=[]
    #delenie bodov do 2 poli p. :  CP x CX => +/-  |  + zafarbenie
    for boddvoj in pole:
        x,y = boddvoj[0], boddvoj[1]
        bodX,bodY = x-indexX, y-indexY    
        znamienko = CPx*bodY - bodX*CPy
        if znamienko > 0 :
            pole1.append([x,y])
            # data_farba[x][y][2] = 0
            # data_farba[x][y][0] += farba
        else:
            pole2.append([x,y])
            # data_farba[x][y][2] = 0
            # data_farba[x][y][1] += farba
    # Image.fromarray(data_farba,'RGB').show()
    return pole1, pole2


# def vykresli(pole_bodov):
#     R_farba = []
#     G_farba = []
#     B_farba = []
#     for boddvoj in pole_bodov:
#         x,y = boddvoj[0], boddvoj[1]
#         R_farba.append(data_farba[x][y][0])
#         G_farba.append(data_farba[x][y][1])
#         B_farba.append(data_farba[x][y][2])
#         data_farba[x][y][0] = 255
#         data_farba[x][y][1] = 255
#         data_farba[x][y][2] = 255
        
#     Image.fromarray(data_farba,'RGB').show()
#     i=0
#     for boddvoj in pole_bodov:
#         x,y = boddvoj[0], boddvoj[1]
#         data_farba[x][y][0] = R_farba[i]
#         data_farba[x][y][1] = G_farba[i]
#         data_farba[x][y][2] = B_farba[i]
#         i+=1

def vyber_polia(polia):
    global vysledok_pole
    for ntica in polia:
        if tuple == type(ntica):
            vyber_polia(ntica)
        else:
            vysledok_pole.append(ntica)
    return





def merging(pole):
    global aver,min_vzdial
    treshold_vzdialenost = 50*min_vzdial
    treshold_body = 30*min_vzdial
    koeficient_chyby = 0.99
    
    working = True    
    dlzka = len(pole)
    if dlzka == 1:
        working = False
    if working and dlzka>1:
        for idx in range(len(pole)):
            if working:
                for jdx in range(len(pole)):
                    if idx != jdx:    
                        # porovnanie zhodnych bodov
                        if len(pole[idx]) == 0 and len(pole[jdx]) == 0:
                            continue
                        #-----------------------------------------------------------------------------------------------------                 
                        # Pre len 2 body -> hladanie ci su body v dostatocne malej vzdialenosti
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
                                #zistinie, ci sa priamky nachadzaju blizsko zeba !! ak nie tak nic, ak hej spoji v zavislosti na splenie kriterii dalsich
                                pole1 = pole[idx][:] 
                                pole2 = pole[jdx][:]
                                
                                #koncove body priamok / segmentov
                                zac1 = [pole1[0][0], pole1[0][1]]
                                kon1 = [pole1[-1][0], pole1[-1][1]]
                                zac2 = [pole2[0][0], pole2[0][1]]
                                kon2 = [pole2[-1][0], pole2[-1][1]]

                                vzdialenost_00 = ( ( zac2[0] - zac1[0] )**2 + ( zac2[1] - zac1[1] )**2 )**0.5
                                vzdialenost_10 = ( ( kon2[0] - zac1[0] )**2 + ( kon2[1] - zac1[1] )**2 )**0.5
                                vzdialenost_01 = ( ( zac2[0] - kon1[0] )**2 + ( zac2[1] - kon1[1] )**2 )**0.5
                                a,b,c,d = kon2[0],kon1[0], kon2[1], kon1[1]
                                vzdialenost_11 = ( ( kon2[0] - kon1[0] )**2 + ( kon2[1] - kon1[1] )**2 )**0.5

                                #vstup iba ak su segmenty pri sebe -> vzdielenost nejakeho z koncovych bodov mensia ako treshold !
                                if vzdialenost_00 < treshold_vzdialenost or vzdialenost_10 < treshold_vzdialenost or vzdialenost_01 < treshold_vzdialenost or vzdialenost_11 < treshold_vzdialenost:
                                # Pre vektory -> postup spajania podla velkosti normalovej spojenej chyby                                
                                # pole1 = pole[idx][:]
                                    pole1_xmax, pole1_ymax, pole1_xmin, pole1_ymin = 0,0,10000,10000
                                    for index in pole1:
                                        if index[0] < pole1_xmin:
                                            pole1_xmin = index[0]
                                        if index[1] < pole1_ymin:
                                            pole1_ymin = index[1]

                                        if index[0] > pole1_xmax:
                                            pole1_xmax = index[0]
                                        if index[1] > pole1_ymax:
                                            pole1_ymax = index[1]                                        
                                    max_dlzka_pole1 = ( (pole1_xmax-pole1_xmin)**2 + (pole1_ymax - pole1_ymin)**2 )**0.5
                                    #param vyjadrenie deliacej pramky cervenej
                                    dx1 = pole1[-1][0]-pole1[0][0]
                                    dy1 = pole1[-1][1]-pole1[0][1]
                                    #znormovanie vektora -> velkost=1
                                    # X = x + dx1*t
                                    # Y = y + dy1*t
                                    norm1 = (dx1**2+dy1**2)**0.5
                                    dx1 = (dx1/norm1)
                                    dy1 = (dy1/norm1)
                                    ndx1 = -dy1
                                    ndy1 = dx1


                                    # pole2 = pole[jdx][:]
                                    pole2_xmax, pole2_ymax, pole2_xmin, pole2_ymin = 0,0,10000,10000
                                    for index in pole2:
                                        if index[0] < pole2_xmin:
                                            pole2_xmin = index[0]
                                        if index[1] < pole2_ymin:
                                            pole2_ymin = index[1]

                                        if index[0] > pole2_xmax:
                                            pole2_xmax = index[0]
                                        if index[1] > pole2_ymax:
                                            pole2_ymax = index[1]                                        
                                    max_dlzka_pole2 = ( (pole2_xmax-pole2_xmin)**2 + (pole2_ymax - pole2_ymin)**2 )**0.5
                                    #param vyjadrenie deliacej pramky cervenej
                                    dx2 = pole2[-1][0]-pole2[0][0]
                                    dy2 = pole2[-1][1]-pole2[0][1]
                                    #znormovanie vektora -> velkost=1
                                    # X = x + dx2*t
                                    # Y = y + dy2*t
                                    norm2 = (dx2**2+dy2**2)**0.5
                                    dx2 = (dx2/norm2)
                                    dy2 = (dy2/norm2)
                                    ndx2 = -dy2
                                    ndy2 = dx2


                                    pole_spolocne = pole1 + pole2
                                    pole_spolocne_xmax, pole_spolocne_ymax, pole_spolocne_xmin, pole_spolocne_ymin = 0,0,10000,10000
                                    for index in pole_spolocne:
                                        if index[0] < pole_spolocne_xmin:
                                            pole_spolocne_xmin = index[0]
                                        if index[1] < pole_spolocne_ymin:
                                            pole_spolocne_ymin = index[1]

                                        if index[0] > pole_spolocne_xmax:
                                            pole_spolocne_xmax = index[0]
                                        if index[1] > pole_spolocne_ymax:
                                            pole_spolocne_ymax = index[1]                                        
                                    max_dlzka_pole_spolocne = ( (pole_spolocne_xmax-pole_spolocne_xmin)**2 + (pole_spolocne_ymax - pole_spolocne_ymin)**2 )**0.5
                                    #param vyjadrenie deliacej pramky cervenej
                                    dxc = pole_spolocne[-1][0]-pole_spolocne[0][0]
                                    dyc = pole_spolocne[-1][1]-pole_spolocne[0][1]
                                    #znormovanie vektora -> velkost=1
                                    # X = x + dxc*t
                                    # Y = y + dyc*t
                                    normc = (dxc**2+dyc**2)**0.5
                                    dxc = (dxc/normc)
                                    dyc = (dyc/normc)
                                    ndxc = -dyc
                                    ndyc = dxc                                

                                    #vykreslenie polii
                                    # plt.axis([0,w,0,h])
                                    # for index in pole1:
                                    #     plt.plot(index[1],w-index[0],',b')
                                    # plt.plot(pole1[0][1],w-pole1[0][0],'.b')
                                    # plt.plot(pole1[-1][1],w-pole1[-1][0],'.b')
                                    # for index in pole2:
                                    #     plt.plot(index[1],w-index[0],',g')
                                    # plt.plot(pole2[0][1],w-pole2[0][0],'.r')
                                    # plt.plot(pole2[-1][1],w-pole2[-1][0],'.r')
                                    # for idx in pole_spolocne:
                                    #     plt.plot(idx[1],w-idx[0],'.r')
                                    # plt.show()

                                    indexX1,indexY1, Px1,Py1, vzdialenost_bod_max1, vzdialenost_od_zac1 = max_vzdialenost(pole1[0][0],pole1[0][1],dx1,dy1,ndx1,ndy1,norm1,pole1)
                                    indexX2,indexY2, Px2,Py2, vzdialenost_bod_max2, vzdialenost_od_zac2 = max_vzdialenost(pole2[0][0],pole2[0][1],dx2,dy2,ndx2,ndy2,norm2,pole2)
                                    indexXc,indexYc, Pxc,Pyc, vzdialenost_bod_maxc, vzdialenost_od_zacc = max_vzdialenost(pole_spolocne[0][0],pole_spolocne[0][1],dxc,dyc,ndxc,ndyc,normc,pole_spolocne)
                                    # print(vzdialenost_bod_max1,vzdialenost_bod_max2,vzdialenost_bod_maxc)

                                    chyba1 = vzdialenost_bod_max1 / max_dlzka_pole1
                                    chyba2 = vzdialenost_bod_max2 / max_dlzka_pole2
                                    chyba_spolocne = vzdialenost_bod_maxc / max_dlzka_pole_spolocne * koeficient_chyby

                                    # v pripade znormovanej chyby celkovej mensej ako spolocne -> spoji dve polia !
                                    #inak ich necha ta....
                                    if chyba_spolocne <= chyba1 or chyba_spolocne <= chyba2:
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
        return merging(pole_new)
    else:
        return pole

def delenie_poly_koniec(pole_skupin_bodov,zobrazenie_delenia=False):
    global w,h,aver,zobraz_postupnost_bodov,min_vzdial

    treshold_strihania = 25*min_vzdial
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


def kolinear(pole):
    ''' Algoritmus spajania poomocou korelácií
     '''
    global aver,min_vzdial
    treshold_vzdialenost_priamok = 50*min_vzdial
    tolerancna_medz_so_smernicou = 30*min_vzdial
    tolerancna_medz_bez = -10*min_vzdial
    treshold_body = 50*min_vzdial
    
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

                                if sucin > 0.93:
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

#vykreslovacky
vykreslenie_kolmice_error = False
zobraz_postupnost_bodov = False
vykreslenie_stvorce = False
vykreslenie_kolmice_RG = False












#===========================================================================================







#                           107 merani 
pole_bodov_vsetko = pickle.load(open('data_python/data_namerane_vsetko.p','rb'))

# volny vyber merani z mnozinu [0:107] merani
pole_bodov2 = pole_bodov_vsetko[0:20]


#vytvorenie jedneho pola bodov
pole_bodov = []
for i in range(len(pole_bodov2)):
    for idx in pole_bodov2[i]:
        pole_bodov.append(idx)

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

#inicializacia max hodnot 
maxX,maxY = 0,0
for bods in pole_bodov:
    x,y = bods[0],bods[1]
    maxX = x if x > maxX else maxX
    maxY = y if y > maxY else maxY
w = h = 50*(maxX//50 + 2) if maxX > maxY else 50*(maxY//50 + 2)



#inicializacia tresholdu
median,aver,modus,max_vzdial,min_vzdial = adaptive_treshold(pole_bodov)
# print(median,aver,modus,max_vzdial,min_vzdial)

# vytvorena funkcia Split
idx0,idy0,idx1,idy1 = firs_last_max(pole_bodov)
rozdelene_polia = split((idx0,idy0),(idx1,idy1),pole_bodov,100)
vysledok_pole = []
vyber_polia(rozdelene_polia)

# merging
novsie_pole0 = merging(vysledok_pole)

farba = ['b','r','c','g','m','y','k']
i=0
plt.figure()
for idx in novsie_pole0:
    vykresli_pyplot_farba(idx,povodne_pole,farba[i],'.')
    i += 1    
    if i == len(farba):
        i=0
plt.title('Split&Merge')         
plt.show()



# ========================================================================================
novsie_pole = delenie_poly_koniec(novsie_pole0)

#filtracia malych skupin bodov < 3
for trieda_bodov in novsie_pole:
    if len(trieda_bodov) > 3:
        vysledok_pole_filt_totoz.append(trieda_bodov)
nove_pole = vysledok_pole_filt_totoz[:]

novsie_pole = kolinear(novsie_pole)


farba = ['b','r','c','g','m','y','k']
i=0

plt.figure()
for idx in novsie_pole:
    vykresli_pyplot_farba(idx,povodne_pole,farba[i],'.')
    i += 1    
    if i == len(farba):
        i=0
plt.title('Split&Merge + Postprocessing')         
plt.show()



