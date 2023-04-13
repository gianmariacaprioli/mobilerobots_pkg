import numpy as np
import math

def func_f_theta (Nodes,task,b_temp,Pos_cand):

    # coordinate target
    ex = Nodes[task,0]  
    ey = Nodes[task,1]
    print("le coordinate del NODO TARGET FINALE sono: ("+ str(ex) + "," + str(ey) +")" )

    # coordinate candidato
    bx = Nodes[b_temp,0] 
    by = Nodes[b_temp,1]
    print("le coordinate del NODO CANDIDATO sono: ("+ str(bx) + "," + str(by) +")" )

    # coordinate nodo corrente
    px = Nodes[Pos_cand,0] 
    py = Nodes[Pos_cand,1]
    print("le coordinate del NODO CORRENTE  sono: ("+ str(px) + "," + str(py) +")" )

    v1=[-1,1]
    print("\nil primo vettore è: " + str(v1) )
    v2=[1, 0]
    print("il secondo vettore è: " + str(v2) )

    theta = math.acos(np.inner(v1,v2)/(np.linalg.norm(v1)*np.linalg.norm(v2)))

    if (theta>=-(math.pi)/2 and theta<=(math.pi)/2):
        f_theta=1
    else:
        f_theta=0

    print("theta compreso è pari a: " + str(theta) + "\n")
    return f_theta

Nodes = np.array([
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

K = 3 # numero di robot della flotta 

task = np.array([ 
[8], 
[9], 
[10]
])

Pos_cand = np.array([[5],
[8]])
# Inizializzazione di task come 
# vettore di dimensioni 1xK (vettore riga).

Pos_corr = np.array([ 
[1, 1],
[2, 1],
[3, 1],
])
# Inizializzazione di Pos_corr come
# vettore di dimensioni Kx2 (matrice).

nextNode = np.array([ 
[0], # 1 1
[4], # 2 1
[8], # 3 1
])
j=1
b_temp = int(nextNode[j])


theta = func_f_theta(Nodes,task[1,0],b_temp,Pos_cand[0,0])
print(theta)
pippo=np.array([[2],[3]])
pluto = np.array([[2,1],[3,1]])
print(str(pippo))