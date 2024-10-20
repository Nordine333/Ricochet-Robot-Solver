import numpy as np
from FonctionsInstances import *
from RicochetRobotSolverA_H1 import *

class RicochetRobotSolverA_H2(RicochetRobotSolverA_H1):
    def __init__(self, taille, nbRobots, maxProf, cellulesInit, verticaux, horizontaux):
        super().__init__(taille, nbRobots, maxProf, cellulesInit, verticaux, horizontaux)
        self.TableH2 = self.CalculerTableH2()
        
    def CalculerTableH2(self): # on va fill le table heuristique 2 (realiser en modifiant de la fctn pour engendrer les voisins)
        posR0 = np.where(self.cellulesInit == 1)
        H2 = self.cellulesInit.copy()
        H2.fill(-1) #met -1 partout
        H2[posR0[0],posR0[1]] = 0 #pos depart à 0
        directions = ["Nord","Sud","Est","Ouest"]
        posi = 0
        posj = 0

        Q = [posR0] #queue des positions à explorer
        while len(Q) > 0:
            posR = Q.pop(0)
            valR = H2[posR[0],posR[1]]
            for move in directions:
                if move == "Nord":
                    posi = -1 #volontairement pas optimiser pour faciliter comprehension
                    posj = 0                
                elif move == "Est":
                    posi = 0
                    posj = 1               
                elif move == "Sud":
                    posi = 1
                    posj = 0                
                elif move == "Ouest":
                    posi = 0
                    posj = -1  
                else:
                    print("Erreur sur les directions")

                nextPos = self.GetBorneH2(posR,posi,posj)

                #avance dans la direction associée
                if move == "Sud":
                    for i in range(int(posR[0]), int(nextPos[0])+1): 
                        if H2[i,int(nextPos[1])] < 0:
                            H2[i,int(nextPos[1])] = valR + 1
                            Q.append([i,nextPos[1]])
                elif move == "Nord" :
                    for i in range(int(nextPos[0]), int(posR[0]) +1): 
                        if H2[i,int(nextPos[1])] < 0:
                            H2[i,int(nextPos[1])] = valR + 1
                            Q.append([i,nextPos[1]])
                elif move == "Est":
                    for j in range(int(posR[1]), int(nextPos[1])+1):  
                        if H2[int(nextPos[0]), j] < 0:
                            H2[int(nextPos[0]), j] = valR + 1
                            Q.append([nextPos[0],j])

                elif move == "Ouest":
                    for j in range(int(nextPos[1]), int(posR[1])+1): 
                        if H2[int(nextPos[0]), j] < 0:
                            H2[int(nextPos[0]), j] = valR + 1
                            Q.append([nextPos[0],j])
                else:
                    print("Erreur sur les directions 2")

        return H2


    def GetBorneH2(self,posR,posi,posj): #avance jusqu'a cogné mais j'ai relaché la contrainte des colissions robots
        nextPos = [posR[0],posR[1]]
        obstacleAtteint = False
        while not obstacleAtteint:
            #verification      
            if nextPos[0]+posi < 0 or nextPos[0]+posi > np.shape(self.verticaux)[0]-1:
                if posi > 0 : #=> on avance a droite on s'arette a la fin
                    nextPos[0] = posi*(np.shape(self.cellules)[0]-1)
                if posi < 0 :#=> on avance a gauche on s'arette au debut
                    nextPos[0] = 0
                obstacleAtteint = True
                break

            if nextPos[1]+posj < 0 or nextPos[1]+posj > np.shape(self.horizontaux)[1]-1:
                if posj > 0 : #=> on descend on s'arette a la fin
                    nextPos[1] = posj*(np.shape(self.cellules)[1]-1)
                if posj < 0 : #=> on monte on s'arette au debut
                    nextPos[1] = 0
                obstacleAtteint = True
                break

            if posj == 0 : #si on se deplace verticalement
                if posi == 1: #on descend
                    if self.horizontaux[nextPos[0],nextPos[1]] == 1:
                        obstacleAtteint = True
                        break
                else: #posi == -1 : on monte
                    if self.horizontaux[nextPos[0]-1,nextPos[1]] == 1:
                        obstacleAtteint = True
                        break

            if posi == 0 : #si on se deplace horizontalement
                if posj == 1: #on descend
                    if self.verticaux[nextPos[0],nextPos[1]] == 1:
                        obstacleAtteint = True
                        break
                else: #posi == -1 : on monte
                    if self.verticaux[nextPos[0],nextPos[1]-1] == 1:
                        obstacleAtteint = True
                        break

            nextPos[0] = nextPos[0]+posi
            nextPos[1] = nextPos[1]+posj

        return nextPos

    def Voisins(self,n): #engendre les voisins du noeud n
        S = []
        directions = ["Nord","Sud","Est","Ouest"]
        posi = 0
        posj = 0
        for numR in range(self.nbRobots): # n[0] == posR1 ...
            for move in directions:
                if move == "Nord":
                    posi = -1 #volontairement pas optimiser pour faciliter comprehension
                    posj = 0                
                elif move == "Est":
                    posi = 0
                    posj = 1               
                elif move == "Sud":
                    posi = 1
                    posj = 0                
                elif move == "Ouest":
                    posi = 0
                    posj = -1  
                else:
                    print("Erreur sur les directions")

                nextPos = self.Traverser(n,numR,posi,posj) #meme fonction que recherche en largeur
                
                posRobots = n.pos.copy() #copie position robots du noeud pere
                
                posRobots[numR] = nextPos #cree voisin avec nouvelle position du robot NumR
                
                gm = n.g + 1
                if gm > n.g + 1:
                    continue
                fm = gm + self.TableH2[posRobots[0][0],posRobots[0][1]]
                
                etat_fils = NoeudAlgoA(posRobots,gm,fm)
                
                    
                #En theorie on doit verifier si on a des noeuds qui repasse par le meme chemin en etant plus rapide
                # càd moins de coups mais ca consomme trop car on doit faire une double boucle explicite :
                
                #verifie si on a deja le noeud dans F ou qu'il n'est pas plus optimal
                #for node in self.F:
                    #if etat_fils.pos == node.pos:
                        #if etat_fils.g >= node.g: #si on a pas un chemin plus optimal (+ de coup pour atteindr la même pos)
                            #continue #on passe a l'iteration suivante
                
                #verifie si on a deja le noeud dans O ou qu'il n'est pas plus optimal
                #for node in self.O:
                    #if etat_fils.pos == node.pos:
                        #if etat_fils.g >= node.g: #si on a pas un chemin plus optimal (+ de coup pour atteindr la même pos)
                            #continue #on passe a l'iteration suivante
                
                #Donc a la place je garde ma methode precedente :
                
                #Verifie si on a deja le noeud dans F et dans O = doublon
                if etat_fils.pos in self.F or etat_fils.pos in [node.pos for node in self.O]:
                    continue
                
                etat_fils.chemin = n.chemin.copy()
                etat_fils.chemin.append(n.pos) #ajoute les positions des robots du pere dans le chemin du fils (solution)


                #ajout du nouvel etat généré
                S.append(etat_fils)

        return S

        
    def Recherche(self):
        posDepart = []
        for i in range(self.nbRobots):
            posInitRobot = np.where(self.cellules == i+1) #pos initiale des robot
            posDepart.append([posInitRobot[0],posInitRobot[1]])
                    
        g = 0
        h = self.TableH2[posDepart[0][0],posDepart[0][1]]# on donne position robot cible depart et il calcule le table heuristique2
        f = g + h
        
        NoeudDepart = NoeudAlgoA(posDepart,g,f)
        
        self.O = [NoeudDepart] #couple pos [[posR1],..,[posRk]], g (nbCoups) , f 

        while len(self.O) > 0 and not self.trouve:
            indice = self.ChoixNoeud()
            n = self.O.pop(indice)

            for i in range(self.nbRobots):
                 # swap ancienne position du robot et nouvelle
                ni = np.where(self.cellules == i+1)
                self.cellules[ni[0], ni[1]] = 0
                self.cellules[n.pos[i][0], n.pos[i][1]] = i+1
            
            #affichage des etapes intermédiaires
            #showgrid(self.taille,self.cellules,self.verticaux,self.horizontaux)

            #ajout du noeud ouvert aux noeuds fermées
            #self.F.append(n)
            self.F.append(n.pos)

            posR_but = n.pos[0] #position du robot devant atteindre la cible

            if posR_but[0] == self.target[0] and posR_but[1] == self.target[1]:
                self.trouve = True
                n.chemin.append(n.pos)
                #print("Solution trouvé")
                return n.chemin, n.g, self.trouve #n.g == nbCoups
            else: #n.g == nbCoups
                if n.g > self.maxProf: #si profondeur plus grande que borneSuperieur on abandonne la branche (la soluce opti pas la)
                    continue
                S = self.Voisins(n) 
                if len(S) > 0:
                    for next_etat in S:
                        self.O.append(next_etat)

        #print("Pas de solution trouvé")
        return [], 0, self.trouve