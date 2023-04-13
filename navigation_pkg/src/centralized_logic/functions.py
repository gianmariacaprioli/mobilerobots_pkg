import numpy as np
import math

###########################################################################

def comp_d_ab(ax,ay,bx,by):

    a = np.array([ ax , ay ])
    b = np.array([ bx , by ]) 
    d_ab = np.linalg.norm(a-b) 
    # distanza euclidea:
    # d_ab = sqrt((ax-bx)^2 + (ay-by)^2) 

    return d_ab

###########################################################################

def dRobot_comp(K,Pos_corr):

    # D_robot è una matrice KxK in cui il generico elemento 
    # D_robot[i,j] è la distanza eulidea fra il robot i-esimo 
    # e il robot j-esimo, considerando le rispettive
    # posizioni correnti nell'ambiente 
    D_robot = np.zeros( (K,K) ) # inizializzazione della matrice

    for i in range(0,K,1): # conto da 0 a K-1 con step 1 (K indica i robot)
        for j in range(0,K-1,1): # conto da 0 a K-1 con step 1 (K indica i robot)
            if(i!=j):

                ax = Pos_corr[i,0]
                ay = Pos_corr[i,1]
                bx = Pos_corr[j,0]
                by = Pos_corr[j,1]
                D_robot[i,j] = comp_d_ab(ax,ay,bx,by)

    return D_robot

###########################################################################

def func_f_theta (Nodes,task,b_temp,Pos_cand):

    # coordinate target
    ex = Nodes[task,0]  
    ey = Nodes[task,1]
    print("le coordinate del NODO TARGET FINALE sono: ("+ str(ex) + "," + str(ey) +")" )

    # coordinate candidato
    bx = Nodes[Pos_cand,0] 
    by = Nodes[Pos_cand,1]
    print("le coordinate del NODO CANDIDATO sono: ("+ str(bx) + "," + str(by) +")" )

    # coordinate nodo corrente
    px = Nodes[b_temp,0] 
    py = Nodes[b_temp,1]
    print("le coordinate del NODO CORRENTE  sono: ("+ str(px) + "," + str(py) +")" )

    v1=[bx-px, by-py]
    print("\nil primo vettore è: " + str(v1) )
    v2=[ex-px, ey-py]
    print("il secondo vettore è: " + str(v2) )

    theta = math.acos(np.inner(v1,v2)/(np.linalg.norm(v1)*np.linalg.norm(v2)))

    if (theta>-(math.pi)/2 and theta<(math.pi)/2):
        f_theta=1
    else:
        f_theta=0

    # if (theta!=math.pi):
    #     f_theta=1
    # else:
    #     f_theta=0
    # print("theta compreso è pari a: " + str(theta) + "\n")
    return f_theta

    

###########################################################################