import numpy as np
from functions import comp_d_ab
from functions import func_f_theta



###########################################################################

def nextNode_comp(A_env,Pos_corr,N_nodes,Nodes,K,D_tol,nextNode,Lambda,task,Q_Lambda): 

    for j in range(0,K,1): # conto da 0 a K-1 con step 1 (K indica i robot)
 ###       print("\n"+"############### SONO DENTRO L'ITERAZIONE DEL ROBOT N°"+ str(j)+" ###############")
        b_temp = int(nextNode[j]) # (0 4 8) tecnicamente il successivo nodo da raggiungere computato
                    # [j=0]   [j=1]
        # Pos_corr =[[ax,ay],[cx,cy]]
        ax = Pos_corr[j][0]
        ay = Pos_corr[j][1]
        bx = Nodes[b_temp,0]
        by = Nodes[b_temp,1]
        D_ab = comp_d_ab(ax,ay,bx,by)
        if(nextNode[j]==task[j]):
            print("IL NEXT NODE DEL ROBOT N° " + str(j) + " E' UGUALE AL TASK QUINDI VIENE INDIRIZZATO LI")
            continue
        elif(D_ab >= D_tol):

            print("Dab > Dtol per il robot \t"+str(j))
            continue
            
        else:
            Pos_cand = np.array([0]) # inizializzo il vettore riga delle posizioni candidate,
            indice=0
            for i in range(0,N_nodes,1): # conto da 0 a N_nodes-1 con step 1

                if (A_env[ b_temp , i ]==1): # verifica l'adiacenza fra nodi
                    if(indice > 0):
                        Pos_cand = np.append(Pos_cand,int(i))
                    else:
                        Pos_cand[indice]=i
                        indice = indice + 1

   ###         print("l'elenco dei possibili candidatiti è: "+ str(Pos_cand) + ", per il robot: " + str(j) + "-esimo")
   ###         print("#####################################################\n")
            if(len(Pos_cand)==1): 
                nextNode[j] = Pos_cand[0]

    ###            print("il prossimo nodo da visitare è il "+ str(Pos_cand) + ", per il robot: " + str(j) + "-esimo")
     ###           print("#####################################################\n")
            else:
                H1 = np.zeros( (len(Pos_cand),1) )
                H2 = np.zeros( (len(Pos_cand),1) )
                A = np.zeros( (len(Pos_cand),1) )
                f = np.zeros( (len(Pos_cand),1) )
                vP = np.zeros( (len(Pos_cand),1) )

                next_cand = 0
                y=0
                Sum = 0 
                vP_max = 0
                c=0 

                for m in range(0,len(Pos_cand),1): # conto da 0 a len(Pos_cand) con step 1
                    if(next_cand==1):
                        break
                    elif (func_f_theta (Nodes,task[j],b_temp,Pos_cand[m])==0):
                        c=m
    ###                    print("######################## Il nodo candidato "+str(Pos_cand[m])+" non è ammissibile perchè F[x]=0 ########################")
                        continue
                    else:
                        for n in range(0,K,1): # conto da 0 a K con step 1 
                            if (n!=j): 

                                d_ijk = comp_d_ab(Nodes[Pos_cand[m],0],Nodes[Pos_cand[m],1],Pos_corr[n,0],
                                Pos_corr[n,1])

                                # if (d_ijk == 0):
                                #     d_ijk =0.001
                                    
                                # print("la d_ijk tra nodo " + str(Pos_cand[m])+" e il robot " + str(n) +" è: " + str(d_ijk))

                                #Sum = Sum+(1/d_ijk)
                                Sum = Sum+(1/(1 + d_ijk))
       ###                         print("la Somma delle distanza all'iterazione " + str(n) + " è: " + str(Sum))
                                
                        tau = Lambda*Sum
     ###                   print("tau per il nodo " + str(Pos_cand[m]) + " è: " + str(tau) + "\n")
                        #print("task[j] è: " + str(task[int(j)]) + "\n")

                        ex = Nodes[task[j],0]
                        ey = Nodes[task[j],1] 
                        bx = Nodes[Pos_cand[m],0]
                        by = Nodes[Pos_cand[m],1] #il nodo dove mi trovo

                        if (comp_d_ab(ex,ey,bx,by) != 0):
                            H2[m] = 1/comp_d_ab(ex,ey,bx,by) #DIS TRA CAND E TARGET_FINALE
                        else:
     ###                       print("####################################### IL CANDIDATO PER IL ROBOT " +str(j) 
     ###                           + " è il nodo " + str(Pos_cand[m]) + "########################################")
                            nextNode[j] = Pos_cand[m]
                            next_cand = 1
                            break #torno nel ciclo j-esimo
                        px = Nodes[Pos_cand[m],0]
                        py = Nodes[Pos_cand[m],1] 
                        bx = Nodes[b_temp,0]
                        by = Nodes[b_temp,1]
                        H1[m] = 1/comp_d_ab(px,py,bx,by)
                        A[m] = Q_Lambda- (tau*H1[m])



                        f[m] = func_f_theta (Nodes,task[j],b_temp,Pos_cand[m])
     ###                   print("la funzione func_f_theta per il candidato " + str(Pos_cand[m]) + " è pari a " + str(f[m]) +"\n")
     ###                   print("####################################################### STO ANCORA CALCOLANDO IL FOR DI M #######################################################")
                if(next_cand==1):
                    continue
                else:
                    Sum_temp = 0
                    for z in range(0,len(Pos_cand),1):
                        
                        if z==c:
                            continue
                        Sum_temp = Sum_temp + ( A[z]*H1[z]*H2[z] )
                    y=0
                    for x in range(0,len(Pos_cand),1):
                        if x==c:
                            continue
     ###                   print("PER IL NODO " + str(Pos_cand[x]) + " le componenti di Vp sono:")
     ###                   print("-A[x] = " + str(A[x]))
     ###                   print("-H1[x] = " + str(H1[x]))
     ###                   print("-H2[x] = " + str(H2[x]))
      ###                  print("-f[x] = " + str(f[x]))
      ###                  print("-Sum_temp = " + str(Sum_temp))
                        vP = ( A[x]*H1[x]*H2[x]*f[x] ) / Sum_temp
     ###                   print("del nodo "+str(Pos_cand[x])+" vP è: " + str(vP))
                        if (vP > vP_max):
                            vP_max = vP 
                            y = x
     ###                       print("per il nodo " + str(Pos_cand[y]) +" VPMAX_temp è: " + str(vP_max)) 
                                          
                        nextNode[j] = Pos_cand[y]
     ###                   print("\n########### per il nodo " + str(Pos_cand[y]) +" VPMAX FINALE è: " + str(vP_max))  
        D_ab=0
    return nextNode

