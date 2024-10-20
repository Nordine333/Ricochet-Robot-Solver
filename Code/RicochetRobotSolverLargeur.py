import numpy as np
from FonctionsInstances import *
from RicochetRobotSolverPronfondeur import *

class RicochetRobotSolverLargeur(RicochetRobotSolverPronfondeur):
    def __init__(self, taille, nbRobots, maxProf, cellulesInit, verticaux, horizontaux):
        super().__init__(taille, nbRobots, maxProf, cellulesInit, verticaux, horizontaux)

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

                nextPos = self.Traverser(n,numR,posi,posj) #retourne uniquement une position

                posRobots = n.pos.copy() #copie position robots du noeud pere
                etat_fils = Noeud(posRobots,n.nbCoups+1)
                etat_fils.pos[numR] = nextPos #cree voisin avec nouvelle position du robot NumR
                
                #verifie si on a deja le noeud dans F et dans O = doublon
                if etat_fils.pos in self.F or etat_fils.pos in [node.pos for node in self.O]:
                    continue
                
                etat_fils.chemin = n.chemin.copy()
                etat_fils.chemin.append(n.pos) #ajoute les positions des robots du pere dans le chemin du fils

                #ajout du nouvel etat généré
                S.append(etat_fils)

        return S


    def Traverser(self,n,numR,posi,posj): # gere les collisions avec les robots (q5)
        nextPos = [n.pos[numR][0],n.pos[numR][1]]
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

            if self.cellules[nextPos[0]+posi,nextPos[1]+posj] > 0: #robot touché
                obstacleAtteint = True
                break


            #cible touché (par autre robots que celui souhaité)
            if numR != 0 and self.cellules[nextPos[0]+posi,nextPos[1]+posj] == -1: #on s'arette pour pas ecraser la cible
                obstacleAtteint = True
                break


            nextPos[0] = nextPos[0]+posi
            nextPos[1] = nextPos[1]+posj

        return nextPos

    def Recherche(self): #q5
        
        posDepart = []
        for i in range(self.nbRobots):
            posInitRobot = np.where(self.cellules == i+1) #pos initiale des robot
            posDepart.append([posInitRobot[0],posInitRobot[1]])
            
        NoeudDepart = Noeud(posDepart,0)
        
        self.O = [NoeudDepart] #couple pos [[posR1],..,[posRk]], evalDuNoeud (nbCoups)
        
        while len(self.O) > 0 and not self.trouve:
            n = self.O.pop(0) #recherche largeur => choix noeud (principe de la queue), sinon on peut faire argmin(nbCoups)
            nbCoups = n.nbCoups
            
            # swap ancienne position du robot et nouvelle
            for numR in range(self.nbRobots):
                posNumR = np.where(self.cellules == numR+1) #numR+1 car on commence à 0
                self.cellules[posNumR[0], posNumR[1]] = 0
                self.cellules[n.pos[numR][0], n.pos[numR][1]] = numR+1
                
            #affichage des etapes intermediaires
            #showgrid(self.taille,self.cellules,self.verticaux,self.horizontaux)

            #ajout du noeud ouvert aux noeuds fermées 
            # uniquement besoin des positions des robots car parcours en largeur balaye horizontalement donc 
            # pas besoin de verifier si on a les memes positons avec un nb de coups inferieur (nbCoups constan a chaqu etge)
            self.F.append(n.pos)

            posR_but = n.pos[0] #position du robot 1 devant atteindre cible
            if posR_but[0] == self.target[0] and posR_but[1] == self.target[1]:
                self.trouve = True
                n.chemin.append(n.pos)
                #print("Solution trouvé")
                return n.chemin, n.nbCoups, self.trouve
            else:
                if n.nbCoups > self.maxProf: #si profondeur plus grande que borneSuperieur on abandonne la branche 
                                             #(la soluce opti n'est pas dedans)
                    continue
                S = self.Voisins(n) 
                if len(S) > 0:
                    for next_etat in S:
                        self.O.append(next_etat)

        #print("Pas de solution trouvé")
        return [], 0, self.trouve #chemin, nbCoups, bool trouve
    
    def PrintSoluce(self,chemin):
        cellule_formate = self.cellulesInit.copy()
        indices = np.where(np.isin(cellule_formate, [1, 2, 3, 4, 5])) #retire anciennes positions robots (retire 1,2,3,4,5 de la grille)
        cellule_formate[indices] = 0

        for elem in chemin:
            cellule_affichee = cellule_formate.copy()
            for i in range(self.nbRobots):
                cellule_affichee[elem[i][0],elem[i][1]] = i+1 # robot1 => val cellule = 1 (0+1)
            showgrid(self.taille,cellule_affichee,self.verticaux,self.horizontaux)
