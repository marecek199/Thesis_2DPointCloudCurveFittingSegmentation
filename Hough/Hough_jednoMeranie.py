

import numpy as np
from PIL import Image
import pickle
import matplotlib.pyplot as plt
import pickle
import random
import sys
import statistics
# sys.path.append('D:\VUT\FSI\Ing\DP\Python\delenieKoniec')
# from delenie_funkcie import kolinear,delenie_poly_koniec

# =============================================================================================================

def vykresli_pyplot(vysvietene_pole,povodne_pole):
    global w,h
    plt.figure()
    for boddvoj in povodne_pole:
        x,y = boddvoj[1],w-boddvoj[0]
        plt.axis([0,w,0,h])
        plt.plot(x,y,',b')

    for boddvoj in vysvietene_pole:
        x,y = boddvoj[1],w-boddvoj[0]
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


# def vykresli(pole_bodov):
#     global data_farba
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
        
#     # Image.fromarray(data_farba,'RGB').show()
#     i=0
#     for boddvoj in pole_bodov:
#         x,y = boddvoj[0], boddvoj[1]
#         data_farba[x][y][0] = R_farba[i]
#         data_farba[x][y][1] = G_farba[i]
#         data_farba[x][y][2] = B_farba[i]
#         i+=1

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
    
    return idx0,idy0,idx1,idy1

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
        return pole_bodov

    pole_bodov[0], pole_bodov[k0] = pole_bodov[k0], pole_bodov[0]
    pole_bodov[-1], pole_bodov[k1] = pole_bodov[k1], pole_bodov[-1]
    
    return pole_bodov

def delenie_poly_koniec(pole_skupin_bodov,zobrazenie_delenia=False):
    global zobraz_postupnost_bodov, min_vzdial

    treshold_strihania = 20*min_vzdial
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

            else:
                if len(body_kamosov) == 0:
                    indexi_delenia.append(idx_bodu)

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
    global min_vzdial

    treshold_vzdialenost_priamok = 30*min_vzdial
    tolerancna_medz_so_smernicou = 20*min_vzdial
    tolerancna_medz_bez = -15*min_vzdial
    treshold_body = 40*min_vzdial
    
    working = True
    dlzka = len(pole)
    if dlzka == 1:
        working = False
    if working and dlzka>1:
        for idx in range(len(pole)):
            if working:
                for jdx in range(len(pole)):
                    if idx != jdx:    
                        if len(pole[idx]) == 0 or len(pole[jdx]) == 0:
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

                        else:
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
                                    try:
                                        vektX, vektY = bods[0]-bod[0][0], bods[1]-bod[0][1]
                                    except TypeError as e:
                                        print('Chyba kolinear')

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


def sorting_useciek(pole_useciek):

    for index1 in range(len(pole_useciek)):
        max = pole_useciek[index1][2]
        idx_max = index1
        for index2 in range(index1,len(pole_useciek)):
            if max < pole_useciek[index2][2]:
                max,idx_max = pole_useciek[index2][2],index2
        pole_useciek[index1], pole_useciek[idx_max] = pole_useciek[idx_max], pole_useciek[index1]

    return pole_useciek



def isLocalMax(Acum,index1,index2,velkost,data_f):
    max0,_ = np.shape(Acum)
    i0,j0,i1,j1= index1-velkost//2, index2-velkost//2, index1+velkost//2, index2+velkost//2
    pi0=pi1=pj0=pj1=0
    while i0<0:
        i0+=1
    while i1>max0:
        i1-=1
    while j0<0:
        j0+=1
    while j1>max0:
        j1-=1
    window = np.zeros((i1-i0,j1-j0),dtype=np.float32)
    dataaa = np.zeros((i1-i0,j1-j0,3), dtype=np.int8)
    # vykreslenie casti 
    for i in range(i0,i1):
        for j in range(j0,j1):
            window[i-i0][j-j0] = int(Acum[i][j])
            dataaa[i-i0][j-j0][1] = int(Acum[i][j])
    dataaa[index1-i0][index2-j0][0] = 255
    dataaa[index1-i0][index2-j0][1] = 0
    concl = True
    pocitadlo = 0
    for i in range(i0,i1):
        for j in range(j0,j1):
            if Acum[index1][index2] < window[i-i0][j-j0]:
                concl = False
            elif Acum[index1][index2] == window[i-i0][j-j0]:
                if window[i-i0][j-j0] == Acum[i][j]:
                    Acum[i][j] -= 1
            elif window[i-i0][j-j0] < 10:
                pocitadlo += 1

    if pocitadlo > 10:
        return concl
    return False



def isLocalMax2(Acum,index1,index2,velkost,data_f):
    max0,_ = np.shape(Acum)
    velkost += 0
    i0,j0,i1,j1= index1-velkost//2, index2-velkost//2, index1+velkost//2, index2+velkost//2
    while i0<0:
        i0+=1
    while i1>max0:
        i1-=1
    while j0<0:
        j0+=1
    while j1>max0:
        j1-=1
    window = np.zeros((i1-i0,j1-j0),dtype=np.float32)
    dataaa = np.zeros((i1-i0,j1-j0,3), dtype=np.int8)
    for i in range(i0,i1):
        for j in range(j0,j1):
            window[i-i0][j-j0] = int(Acum[i][j])
            dataaa[i-i0][j-j0][1] = int(Acum[i][j])
    dataaa[index1-i0][index2-j0][0] = 255
    dataaa[index1-i0][index2-j0][1] = 0
    concl = True
    maxim,indexI2,indexJ2 = 0,0,0
    for i in range(i0,i1):
        for j in range(j0,j1):
            if Acum[index1][index2] < window[i-i0][j-j0]:
                concl=False
                if maxim < window[i-i0][j-j0]:
                    maxim,indexJ2,indexI2 = window[i-i0][j-j0],i,j
            elif Acum[index1][index2] == window[i-i0][j-j0]:
                if window[i-i0][j-j0] == Acum[i][j]:
                    Acum[i][j] -= 1

    if concl:
        return Acum[index1][index2], j, i
    return maxim, indexJ2, indexI2


def vykresli_accum(accum,maxim,data_f_a):
    riad,stlp = np.shape(accum)
    imaged = np.zeros((riad,stlp),dtype=np.uint8)
    for riadok in range(riad):
        for stlpec in range(stlp):
            data_f_a[riadok][stlpec] = int(accum[riadok][stlpec]* 255 // maxim)




def vykresli_usecky(pole_useciek,x_r,y_r):
    global w,h
    pole_delenie_udaje = []
    data_vysledok = np.zeros((w,h), dtype=np.uint8)   
    for idx in pole_useciek:
        fi = idx[0]
        r = idx[1]
        #indexy bodov kolmice
        x = ( np.cos(fi)*r + x_r )
        y = ( np.sin(fi)*r + y_r )
        #vektor kolmice
        nx0 = np.sin(fi)*np.sign(r)
        ny0 = np.cos(fi)*np.sign(r)
        dlzka_norm = ( (nx0**2 + ny0**2)**(1/2) )

        nx = nx0/dlzka_norm
        ny = -ny0/dlzka_norm
        
        #vykreslovanie Hough-priamok
        dlzka_max = round( (w**2 + h**2)**(1/2) )
        for t in range(-dlzka_max,dlzka_max):
            x_new = (x + t*nx)
            y_new = (y + t*ny)
        
        pole_delenie_udaje.append([x,y,nx,ny])

    return data_vysledok,pole_delenie_udaje


def delenie_bodov(pole_bodov,pole_delenie_udaje):
    global pole_new

    if len(pole_delenie_udaje) > 0:
        x,y,nx,ny = pole_delenie_udaje[0]
        pole_delenie_udaje.pop(0)
        x0,y0,vektX,vektY = x,y,nx,ny
        normal = (vektX**2 + vektY**2)**0.5
        treshold = 70
        pomoc_pole = []
        druhe_pole = []

        for bod in pole_bodov:            
            rx, ry = bod[0] - x0, bod[1] - y0
            ndx, ndy = -vektY/normal ,vektX/normal
            vzdialenost_bod_priamka = abs(ndx*rx + ndy*ry)
            if vzdialenost_bod_priamka < treshold :
                pomoc_pole.append([bod[0],bod[1]])
            else:
                druhe_pole.append([bod[0],bod[1]])

        pole_new.append(pomoc_pole)
        delenie_bodov(druhe_pole,pole_delenie_udaje)

    else:
        if len(pole_bodov) > 1:
            pole_new.append(pole_bodov)


def houghTransfor(pole):
    global w,h
    n,m = 200,200
    M,N = w,h

    x_r, y_r = M//2, N//2
    d_fi = np.pi / m
    d_r = (M**2 + N**2)**(1/2) / n
    j0 = n//2

    Acum = np.zeros((m,n),dtype=np.float32)
    data_farba_accum = np.zeros((n,m,3), dtype=np.uint8)

    maxim = 0
    for data in pole:
        u,v = data[0], data[1]
        x, y = u-x_r, v-y_r
        for i in range(m):
            fi = d_fi * i
            r = x * np.cos(fi) + y * np.sin(fi)
            j = j0 + round(r/d_r)
            Acum[j][i] += 1
            if maxim<Acum[j][i]:
                maxim = Acum[j][i]
    vykresli_accum(Acum,maxim,data_farba_accum)
    # print('Maximalny pocet inkrementov -',str(maxim))

    lines = []
    w_pol = n // 2
    percento = 0.96
    doplnok = 1 - percento
    posl_percenta = int(w*percento)
    prve_percenta = int(w*doplnok)+1
    velkost_konv_matice = 20
    velkost_konv_matice2 = 25
    percento = maxim / len(pole)
    amin = int( 1 / 7 * maxim )
    
    for i in range(m):  
        for j in range(n):
            if Acum[j][i] >= amin:
                # # porovnanie prvych 5% stlpcov s ostatnymi na druhej strane
                concl1 = isLocalMax(Acum,j,i,velkost_konv_matice,data_farba_accum)
                if i < prve_percenta:
                    # obdve su maxima -> vybrat vacsie 
                    #pozera sa na druhu strana iba ak najde maximum-> vtedy sa rozhodne ci je to dost dobre 
                    # tak ze porovna aj druhu stranu alebo nie druha strana by mohla prehladat trocha vacsi priestor
                    if concl1:
                        i2,j2 = int(2*(w_pol - i) + i)-1, int(2*(w_pol - j) + j)-1
                        vysl2,j2,i2 = isLocalMax2(Acum,j2,i2,velkost_konv_matice2,data_farba_accum)
                        if Acum[j][i] >= vysl2:
                            ivysl,jvysl = i,j
                            
                        else:
                            ivysl,jvysl = i2,j2
                        fi = ivysl * d_fi
                        r = (jvysl-j0) * d_r
                        a = Acum[jvysl][ivysl]
                        L = [fi, r, a]
                        lines.append(L)
                        idx,jdx = i,j
                        data_farba_accum[jvysl][ivysl][0] = 255
                        data_farba_accum[jvysl][ivysl][1] = 0
                        # print(jvysl,ivysl)
                        # Image.fromarray(data_farba_accum,'RGB').show()

                elif i < posl_percenta and concl1:
                    fi = i * d_fi
                    r = (j-j0) * d_r
                    a = Acum[j][i]
                    L = [fi, r, a]
                    lines.append(L)
                    data_farba_accum[j][i][0] = 255
                    data_farba_accum[j][i][1] = 0                    


    return lines,x_r,y_r



#vykreslovacky 
zobraz_postupnost_bodov = False

# =============================================================================================================



#                           107 merani
pole_bodov_vsetko = pickle.load(open('data_python/data_namerane_vsetko.p','rb'))
#vybratie merania [0-106]
pole_bodov = pole_bodov_vsetko[63]
povodne_pole = pole_bodov[:]

#Pociatocne vytvorenie pola bodov -> ziskanie w,h
maxX,maxY = 0,0
for bods in pole_bodov:
    x,y = bods[0],bods[1]
    maxX = x if x > maxX else maxX
    maxY = y if y > maxY else maxY
w = h = 50*(maxX//50 + 2) if maxX > maxY else 50*(maxY//50 + 2)

# inicializacia tresholdu
aver,median,modus,max_vzdial,min_vzdial = adaptive_treshold(pole_bodov)

# HOUGH transformacia vstup - pole bodov, vystup - pole priamok,xr,yr(suradnice stredu)
new,xr,yr = houghTransfor(pole_bodov)

#zoradenie useciek podla hodnoty "votes"
new_usecky = sorting_useciek(new)
data,pole_delenie_udaje = vykresli_usecky(new_usecky,xr,yr)
pole_new=[]
delenie_bodov(pole_bodov,pole_delenie_udaje)



#Zobrazenie vysledku Hough funkcie
farba = ['b','g','r','c','m','y','k']
i=0
plt.figure()
for idx in pole_new:
    vykresli_pyplot_farba(idx,povodne_pole,farba[i],'.')
    i += 1    
    if i == len(farba):
        i=0
plt.title('Hough')        
plt.show()



# posprocessing ----------------------------------------------------------------------------------------

nove_pole = delenie_poly_koniec(pole_new)
novsie_pole = kolinear(nove_pole)


farba = ['b','g','r','c','m','y','k']
i=0
plt.figure()
for idx in novsie_pole:
    vykresli_pyplot_farba(idx,povodne_pole,farba[random.randrange(7)],'.')
    i += 1    
    if i == len(farba):
        i=0
plt.title('Hough + Postprocessing')        
plt.show()
