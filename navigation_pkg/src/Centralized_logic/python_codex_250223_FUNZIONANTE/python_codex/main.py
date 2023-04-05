#!/usr/bin/env python3

# Python 3.11.0 64-bit

# pip install numpy 
# pip --version
# python.exe -m pip install --upgrade pip
import numpy as np

import centAct # azione centralizzata
# import decentAct # azione decentralizzata

###############################################################
# PARAMETERS
###############################################################

Lambda = 50 # concentration of pheromones carried by each AGV
Q_Lambda = 5 # threshold of congestion
T_s = 0.25 # sample time in [s] of the algorithm
D_tol = 0.21 # tolerance distance in [m]

###############################################################
# ENVIRONMENTAL MODEL
###############################################################

# N_nodes è il numero di intersezioni fra strade della rete, 
# quindi fra almeno due strade diverse

# Nodes è una matrice del tipo N_nodes x 2 in cui ogni nodo è inserito 
# in una riga e presenta nella prima colonna la sua coordinata 
# cartesiana sull'asse x e nella seconda colonna la sua coordinata
# cartesiana sull'asse y. L'indice di riga (da 0 a N_nodes-1) 
# rappresenta l'indice numerico identificativo del nodo.

# A_env è una matrice di adiacenza del tipo N_nodes x N_nodes, 
# presentante valore A_env[i,j]=0 se il nodo i non è interconnesso con
# il nodo j, valore A_env[i,j]=1 in caso contrario.
# Nel nostro caso la matrice non è simmetrica, in quanto stiamo
# lavorando con strade monodirezionali, quindi A_env[i,j] è diverso
# da A_env[j,i] in quanto solo uno dei due può essere non nullo.

N_nodes_16 = 16 # road intersections
N_nodes = 24

Nodes_16 = np.array([
[1,1],
[1,2],
[1,3],
[1,4],
[2,1],
[2,2],
[2,3],
[2,4], 
[3,1], #8
[3,2], #9
[3,3], #10
[3,4],
[4,1],
[4,2],
[4,3],
[4,4],
])

Nodes = np.array([ #contiene le coordinate cartesiane
[0,0], #0
[0,1],
[0,2],
[0,3],
[1,0],
[1,1],
[1,2],
[1,3],
[2,0],
[2,1],
[2,2],
[2,3],
[3,0],#12
[3,3],#13
[4,0],
[4,1],
[4,2],
[4,3],
[5,0],
[5,1],
[5,2],
[5,3],
[6,0],
[6,1],
[6,2],
[6,3],#25
])

A_env_16 = np.array([ 
#0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19 20 21 22 23 

[0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0],
[0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0], #8
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0],
[0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
])

A_env = np.array([
    #0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25
    [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], #0
    [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], #1
    [0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], #2
    [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], #3
    [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], #4
    [0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], #5
    [0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], #6
    [0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], #7
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], #8
    [0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], #9
    [0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], #10 
    [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], #11
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], #12
    #0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], #13
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0], #14
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0], #15
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0], #16
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], #17
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0], #18
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0], #19
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0], #20
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0], #21
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0], #22
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0], #23
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1], #24
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0]  #25
])

###############################################################
# SIMULATION SCENARIO
###############################################################

# K è il numero di robot della flotta

# Pos_corr è una matrice che rappresenta le posizioni correnti dei  
# K robot nella rete, indicando come i-esima riga della matrice 
# la posizione cartesiana del robot i-esimo, e in particolare 
# nella prima colonna la coordinata cartesiana x e nella
# seconda colonna la coordinata cartesiana y.

# task è un vettore riga di K elementi, in cui ogni elemento 
# i-esimo è il task (indice del nodo da raggiungere) dell'i-esimo
# robot della flotta (di K robot).

K = 3 # numero di robot della flotta 

task = np.array([ 
[18], 
[14], 
[22]
])
# Inizializzazione di task come 
# vettore di dimensioni 1xK (vettore riga).

Pos_corr1 = np.array([ 
[0, 1],
[2, 0],
[1, 2],
])

# Pos_corr2 = np.array([ 
# [1, 0],
# [2, 0],
# [3, 0],
# ])
# Pos_corr3 = np.array([ 
# [2, 0],
# [3, 0],
# [4, 0],
# ])
# Inizializzazione di Pos_corr come
# vettore di dimensioni Kx2 (matrice).

nextNode = np.array([ 
[1], # 1 1
[6], # 2 1
[8], # 3 1
])
# Inizializzazione di nextNode, considerando gli indici dei
# nodi di partenza (vedi Pos_corr)

###############################################################
# NODE SELECTION ALGORITHM (CENTRALIZED ACTION)
###############################################################

# nextNode è un vettore riga di K elementi, in cui il generico
# elemento i-esimo è il nodo (indice numerico identificativo) 
# verso cui deve dirigersi il robot i-esimo nel successivo time step.

#for i in range(0,3):
    # if i==0:
    #     print("######################### ZERO ITERAZIONE #########################")
    #     nextNode = centAct.nextNode_comp(A_env,Pos_corr1,N_nodes,Nodes,K,D_tol,nextNode,Lambda,task,Q_Lambda)
    #     print("\n########################################### IL NEXT NODE è: " + str(nextNode)+ " \n")
    # elif i == 1:
    #     print("######################### UNO ITERAZIONE #########################")
    #     nextNode = centAct.nextNode_comp(A_env,Pos_corr2,N_nodes,Nodes,K,D_tol,nextNode,Lambda,task,Q_Lambda)
    #     print("\nIL NEXT NODE è: " + str(nextNode)+ " \n")
    # else:
    #     print("######################### DUE ITERAZIONE #########################")
    #     nextNode = centAct.nextNode_comp(A_env,Pos_corr3,N_nodes,Nodes,K,D_tol,nextNode,Lambda,task,Q_Lambda)
    #     print("\nIL NEXT NODE è: " + str(nextNode)+ " \n")

    #################################################################################################################
    #################################################################################################################

    # sta un errore per il quale viene calcolata ugualmente la VP una volta assegnato il nodo successivo
    # dato che 1/0 in H2 non è ammissibile visto e considerato che la distanza tra il candidato 
    # il nodo target è 0 visto che coincidono.
    # fermo restando che per il robot 0 questo non succede, ma credo sia dovuto al fatto che viene assegnato
    # di default considerato che è l'unico nodo visitabile da 4 a 8
    #
    # va considerato inoltre il problema dell'uscita dal for che itera su tutti i candidati 
    #
    # ulteriore problema risulta essere il fatto che anche se 10 non è un nodo target per il robot 1
    # il calcolo di v1 restituisce -0. dall'iterazione 7 in poi, ovvero da quando l'algoritmo inizia a 
    # considerare quel nodo come papabile candidato.
    
    ################################################################################################################
    ################################################################################################################ 

for i in range(0,10):
        print("######################### " + str(i)+ " ITERAZIONE ########################################### !! #")
        nextNode = centAct.nextNode_comp(A_env,Pos_corr1,N_nodes,Nodes,K,D_tol,nextNode,Lambda,task,Q_Lambda)
        print("\n########################################### IL NEXT NODE è: " + str(nextNode)+ " \n")
        Pos_corr1[0]= Nodes[nextNode[0]]
        Pos_corr1[1]= Nodes[nextNode[1]]
        Pos_corr1[2]= Nodes[nextNode[2]]
        print("\n########################################### LA POSCOR è: " + str(Pos_corr1)+ " \n")
    #riga colonna colonna
###############################################################
# VARYING VELOCITY ALGORITHM (DECENTRALIZED ROBOT ACTION)
###############################################################

# nextVel è un vettore riga di K elementi, in cui il generico
# elemento i-esimo è la velocità del robot i-esimo nel 
# successivo time step.

# nextVel = np.zeros( (1,K) ) 
# Inizializzazione di nextVel come 
# vettore di dimensioni 1xK (vettore riga).

# nextVel = decentAct.nextVel_comp(...) 

###############################################################
# TRANSITION RULE (OVERALL ACTION)
###############################################################

# communication with ROS

###############################################################
# ENVIRONMENTAL REQUIREMENTS
###############################################################

# 1) Point (0,0) cannot be a road intersection, so it can't be 
# traveled by any robot of the fleet;

###############################################################
# FUTURE REMARKS
###############################################################

# include the mass of the robots in the decentralized actions

###############################################################
# NOTES
###############################################################

# Classe: lettera maiuscola iniziale
# funzione: lettera minuscola iniziale
# Matrice: lettera maiuscola iniziale
# vettore: lettera minuscola iniziale
# Costante: lettera maiuscola iniziale

# clear = it clears the terminal
# CTRL+K+C = insert comments belonging a lot of lines
# CTRL+K+U = delete comments belonging a lot of lines

###############################################################

# Istante 0: tutti i robot sono fermi in un incrocio*.
# L'azione centralizzata definisce per ogni robot la direzione
# di spostamento per l'istante di tempo successivo.
# L'azione decentralizzata di ogni robot definisce la velocità 
# di spostamento che dovrà essere adottata nel successivo
# istante di tempo.

# Istante m (dopo mxT_s secondi): tutti i robot si trovano in
# una posizione identificabile in termini di coordinate 
# cartesiane ma non si trovano generalmente in un incrocio.
# L'azione centralizzata prevede il mantenimento della direzione
# precedentemente computata, per i robot che non si trovano in 
# un incrocio (ad una distanza dall'incrocio con tolleranza d_tol).
# Prevede invece una nuova computazione della direzione di 
# spostamento per i robot che si trovano in un incrocio (ad una
# distanza dall'incrocio con tolleranza d_tol).

# L'insieme dei nodi candidati viene definito sulla base dell'adiacenza
# fra nodi di un grafo generato per rappresentare l'ambiente di lavoro.
# In particolare, si tratta di un grafo orientato e non pesato, 
# presentante come nodi gli incroci della rete e come archi le 
# interconnessioni stradali fra tali nodi. 
# Tale grafo viene rappresentato matematicamente mediante una matrice
# di adiacenza, che definisce i nodi adiacenti con degli 1 e i nodi
# non adiacenti con degli zeri, tenendo conto del verso di percorrenza
# di ogni tratto stradale (essendo il grafo orientato).

# L'azione centralizzata prevede la conoscenza dei seguenti dati:
# - posizione cartesiana corrente di ogni robot;
# - posizione cartesiana di ogni nodo candidato;
# - posizione cartesiana di ogni nodo di destinazione.

# L'azione decentralizzata prevede la conoscenza dei seguenti dati:
# - posizione cartesiana corrente di ogni robot;
# - posizio cartesiana di ogni nodo verso cui si stanno dirigendo i robot.

# * requisito di funzionamento