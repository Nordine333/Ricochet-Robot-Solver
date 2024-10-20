import numpy as np
from FonctionsInstances import *

class Noeud:
    def __init__(self, pos, nbCoups):
        self.pos = pos
        self.nbCoups = nbCoups
        self.chemin = []

class RicochetRobotSolverPronfondeur:
    def __init__(self, taille, nbRobots, maxProf, cellulesInit, verticaux, horizontaux):
        self.taille = taille
        self.nbRobots = nbRobots
        self.maxProf = maxProf
        self.cellulesInit = cellulesInit.copy()
        self.cellules = cellulesInit.copy()
        self.verticaux = verticaux
        self.horizontaux = horizontaux
        self.O = []
        self.F = []
        self.trouve = False
        self.target = np.where(cellulesInit == -1)

    def ChoixNoeud(self): #choisi le prochain noeud à ouvrir selon son evaluation
        indMin = 0
        evalMax = -1
        for i in range(len(self.O)):
            n = self.O[i]
            if n.nbCoups > evalMax:
                indMin = i
                evalMax = n.nbCoups
        return indMin

    def Traverser(self,n,posi,posj): #avance dans la direction jusqu'a toucher un obstacle
        nextPos = [n.pos[0],n.pos[1]] #nextPos est incrementé dans la direction jusqu'a toucher un obstacle
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


            nextPos[0] = nextPos[0]+posi
            nextPos[1] = nextPos[1]+posj

        return nextPos

    def Voisins(self,n): #engendre les voisins du noeud n
        S = []
        if n.nbCoups + 1 <= self.maxProf:
            directions = ["Nord","Sud","Est","Ouest"]
            posi = 0
            posj = 0
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
                    print("Erreur sur les directions choisies.")

                nextPos = self.Traverser(n,posi,posj)
                
                doublon = False

                #vérifie les doublons dans F
                for node in self.F:
                    if node == nextPos:
                        doublon = True
                        break

                #vérifie les doublons dans O
                if not doublon:
                    for node in self.O:
                        if node.pos == nextPos :
                            doublon = True
                            break

                if doublon:
                    continue

                else:
                    #ajout du nouvel etat généré
                    etat_fils = Noeud(nextPos,n.nbCoups+1)
                    etat_fils.chemin = n.chemin.copy()
                    etat_fils.chemin.append(n.pos)
                    S.append(etat_fils)
        return S

    def Recherche(self): #q4 (estime borne sup : ne gere pas collision avec robots)
        posDepart = np.where(self.cellules == 1)
        NoeudDepart = Noeud(posDepart,0)
        
        self.O = [NoeudDepart] #couple pos, evalDuNoeud (nbCoups)
        while len(self.O) > 0 and not self.trouve:
            
            indice = self.ChoixNoeud()
            n = self.O.pop(indice)
            
            oldPosR = np.where(self.cellules == 1)
            
            #Met a jour les cellules (sinon le robot se cogne contre d'ancienne position)
            self.cellules[oldPosR[0],oldPosR[1]] = 0
            self.cellules[n.pos[0],n.pos[1]] = 1
            
            #affichage des etapes intermediaires
            #showgrid(self.taille,self.cellules,self.verticaux,self.horizontaux)

            self.F.append(n.pos)

            if n.pos[0] == self.target[0] and n.pos[1] == self.target[1]:
                self.trouve = True
                #print("Solution trouvé")
                n.chemin.append(n.pos) #rajoute dernier position evidente (sur la cible)
                return n.chemin, n.nbCoups, self.trouve
            else:
                S = self.Voisins(n) 
                if len(S) > 0:
                    for next_etat in S:
                        self.O.append(next_etat)

        #print("Pas de solution trouvé")
        return [], self.maxProf, self.trouve
    
    def PrintSoluce(self, chemin):
        cellule_formate = self.cellulesInit.copy()
        indices = np.where(np.isin(cellule_formate, [1])) #retire l'ancienne position du robot R1
        cellule_formate[indices] = 0

        for elem in chemin:
            cellule_affichee = cellule_formate.copy()
            cellule_affichee[elem[0],elem[1]] = 1 # R1 prend la position du chemin i ème noeud du chemin
            showgrid(self.taille,cellule_affichee,self.verticaux,self.horizontaux)
    