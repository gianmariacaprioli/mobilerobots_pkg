import numpy as np
from functions import dRobot_comp
from functions import comp_d_ab
from functions import func_f_theta


###########################################################################

def nextNode_comp(A_env,Pos_corr,N_nodes,Nodes,K,D_tol,nextNode,Lambda,task,Q_Lambda): 

    for j in range(0,K,1): # conto da 0 a K-1 con step 1 (K indica i robot)

        print("\n"+"############### SONO DENTRO L'ITERAZIONE DEL ROBOT N°"+ str(j)+" ###############")
        # Calcola la distanza D_ab fra il robot considerato (j-esimo) e il 
        # nodo che nel time step precedente era il nodo da raggiungere
        b_temp = int(nextNode[j]) # (0 4 8) tecnicamente il successivo nodo da raggiungere computato
                                  # nell'iterazione precedente
        #print("il nodo "+str(b_temp)+" è il nodo successivo del robot "+ str(j)+"\n")

        ax = Pos_corr[j,0]
        ay = Pos_corr[j,1]
        #print("posizione iniziale è: " + str(ax)+" "+str(ay)) 

        bx = Nodes[b_temp,0]
        by = Nodes[b_temp,1]
        #print("posizione successiva è: " + str(bx)+" "+str(by))

        D_ab = comp_d_ab(ax,ay,bx,by)
        #print("la distanza euclidea è: "+str(D_ab)+"\n")

        if(nextNode[j]==task[j]):
            #chiamata ad una funzione che allerta ros della necessità di un nuovo task per il
            #robot j-esimo
            print("IL NEXT NODE DEL ROBOT N° " + str(j) + " E' UGUALE AL TASK QUINDI VIENE INDIRIZZATO LI")
            continue
        elif(D_ab > D_tol):

            print("Dab > Dtol per il robot \t"+str(j))
            # Nel caso in cui il robot non abbia ancora raggiunto il 
            # nodo designato nella precedente computazione della direzione,
            # cioè b_temp = nextNode[j], mantiene la direzione 
            # precedentemente calcolata, non aggiornando nextNode[j].
            #print("Dab>Dtop" + str(Pos_cand) + "per il robot: " + str(j) + "-esimo")
            continue
            
        else:

            # Nel caso in cui il robot j abbia raggiunto il nodo designato nella 
            # precedente computazione della direzione, con tolleranza D_tol, viene
            # individuato ogni nodo candidato e fra questi viene scelto il nodo 
            # successivo, effettuando la scelta sulla base del calcolo di una 
            # funzione P da massimizzare:

            Pos_cand = np.array([0]) # inizializzo il vettore riga delle posizioni candidate,
            # che presenterà un certo numero di righe pari al numero di nodi candidati,
            # con le posizioni intese in termini di indice di nodo 
            

            indice=0
            for i in range(0,N_nodes,1): # conto da 0 a N_nodes-1 con step 1

                if (A_env[ b_temp , i ]==1): # verifica l'adiacenza fra nodi
                    if(indice > 0):
                        Pos_cand = np.append(Pos_cand,int(i))
                    else:
                        Pos_cand[indice]=i
                        indice = indice + 1

            print("l'elenco dei possibili candidatiti è: "+ str(Pos_cand) + ", per il robot: " + str(j) + "-esimo")
            print("#####################################################\n")
            # Una volta individuati i nodi candidati, viene definito fra questi il successivo
            # nodo di destinazione per il robot considerato (j-esimo):

            if(len(Pos_cand)==1): 
                nextNode[j,0] = Pos_cand[0]

                print("il prossimo nodo da visitare è il "+ str(Pos_cand) + ", per il robot: " + str(j) + "-esimo")
                print("#####################################################\n")
                # se trova un solo candidato assegna direttamente nextNode[0,j]
                #print('candidato scelto:\t'+Pos_cand[0]+'\n')

            else:

                # Calcola vP per ogni nodo candidato nel vettore Pos_cand e sceglie 
                # come nodo successivo quello fra i candidati presentante un valore
                # più elevato di vP

                
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
                # inizializzazione del valore massimo di vP

                for m in range(0,len(Pos_cand),1): # conto da 0 a len(Pos_cand) con step 1
                    if(next_cand==1):
                        break
                    elif (func_f_theta (Nodes,task[j,0],b_temp,Pos_cand[m])==0):
                         c=m
                         print("######################## Il nodo candidato "+str(Pos_cand[m])+" non è ammissibile perchè F[x]=0 ########################")
                         continue
                    else:
                        for n in range(0,K,1): # conto da 0 a K con step 1 

                            # distanza tra rob 0 e nodo 5
                            # distanza tra rob 2 e nodo 5
                            
                            # distanza tra rob 0 e nodo 8                          
                            # distanza tra rob 2 e nodo 8

                            # itero su tutti i K robot con l'indice n per calcolare la distanza d_ijk, la quale 
                            # deve  essere valutata tra il robot n-esimo e il nodo candidato m-esimo
                        
                            if (n!=j): 

                                d_ijk = comp_d_ab(Nodes[Pos_cand[m],0],Nodes[Pos_cand[m],1],Pos_corr[n,0],
                                Pos_corr[n,1])

                                # if (d_ijk == 0):
                                #     d_ijk =0.001
                                    
                                # print("la d_ijk tra nodo " + str(Pos_cand[m])+" e il robot " + str(n) +" è: " + str(d_ijk))

                                #Sum = Sum+(1/d_ijk)
                                Sum = Sum+(1/(1 + d_ijk))
                                print("la Somma delle distanza all'iterazione " + str(n) + " è: " + str(Sum))
                                
                        tau = Lambda*Sum
                        print("tau per il nodo " + str(Pos_cand[m]) + " è: " + str(tau) + "\n")
                        #print("task[j] è: " + str(task[int(j)]) + "\n")

                        ex = Nodes[task[j],0]
                        ey = Nodes[task[j],1] 
                        bx = Nodes[Pos_cand[m],0]
                        by = Nodes[Pos_cand[m],1] #il nodo dove mi trovo

                        # #MODIFICA EFFETTUATA DA JIM IN DATA 170223
                        # # se il nodo candidato è uguale al nodo target la distanza è nulla effettua un'operazione
                        # # proibita (1/0) dando errore
                        # # ho risolto facendo un controllo sull'operazione ed invertendo successivamente

                        if (comp_d_ab(ex,ey,bx,by) != 0):
                            H2[m] = 1/comp_d_ab(ex,ey,bx,by) #DIS TRA CAND E TARGET_FINALE
                        else:
                            print("####################################### IL CANDIDATO PER IL ROBOT " +str(j) 
                                + " è il nodo " + str(Pos_cand[m]) + "########################################")
                            nextNode[j,0] = Pos_cand[m]
                            next_cand = 1
                            break #torno nel ciclo j-esimo
                        # comp_d_ab è un vettore di len(Pos_cand) elementi, in cui il generico  
                        # elemento d_CE[m] è la distanza eulidea distanza euclidea fra 
                        # nodo candidato m-esimo e task del robot j-esimo

                        px = Nodes[Pos_cand[m],0]
                        py = Nodes[Pos_cand[m],1] 
                        bx = Nodes[b_temp,0]
                        by = Nodes[b_temp,1]
                        H1[m] = 1/comp_d_ab(px,py,bx,by)
                        # comp_d_ab è un vettore di len(Pos_cand) elementi, in cui il generico  
                        # elemento d_PC[m] è la distanza eulidea distanza euclidea fra 
                        # nodo corrente del robot j-esimo e nodo candidato i-esimo

                        A[m] = Q_Lambda- (tau*H1[m])
                        #A[m] =(tau*H1[m])


                        f[m] = func_f_theta (Nodes,task[j,0],b_temp,Pos_cand[m])
                        print("la funzione func_f_theta per il candidato " + str(Pos_cand[m]) + " è pari a " + str(f[m]) +"\n")
                        print("####################################################### STO ANCORA CALCOLANDO IL FOR DI M #######################################################")
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
                        print("PER IL NODO " + str(Pos_cand[x]) + " le componenti di Vp sono:")
                        print("-A[x] = " + str(A[x]))
                        print("-H1[x] = " + str(H1[x]))
                        print("-H2[x] = " + str(H2[x]))
                        print("-f[x] = " + str(f[x]))
                        print("-Sum_temp = " + str(Sum_temp))


                        vP = ( A[x]*H1[x]*H2[x]*f[x] ) / Sum_temp
                        print("del nodo "+str(Pos_cand[x])+" vP è: " + str(vP))

                        if (vP > vP_max):
                            vP_max = vP 
                            y = x
                            print("per il nodo " + str(Pos_cand[y]) +" VPMAX_temp è: " + str(vP_max)) 
                                                
                        nextNode[j,0] = Pos_cand[y]

                # IL PROBLEMA PENSIAMO ESSERE LA POSIZIONE DEI CICLI E IL CALCOLO DELLE H1/H2 CHE DEVE ESSERE ACCORPATO
                # IN QUALCHE MANIERA
                        print("\n########### per il nodo " + str(Pos_cand[y]) +" VPMAX FINALE è: " + str(vP_max))  
        D_ab=0
    return nextNode