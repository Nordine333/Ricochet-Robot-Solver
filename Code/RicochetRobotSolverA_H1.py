import numpy as np
from FonctionsInstances import *
from RicochetRobotSolverLargeur import *

class NoeudAlgoA: 
    def __init__(self, pos, g , f):
        self.pos = pos
        self.g = g #eq nb coups
        self.f = f
        self.chemin = []
        
        
class RicochetRobotSolverA_H1(RicochetRobotSolverLargeur):
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

                nextPos = self.Traverser(n,numR,posi,posj) #meme fonction que recherche en largeur
                
                posRobots = n.pos.copy() #copie position robots du noeud pere
                
                posRobots[numR] = nextPos #cree voisin avec nouvelle position du robot NumR
                
                gm = n.g + 1
                if gm > n.g + 1:
                    continue
                fm = gm + self.CalculerH1(posRobots[0])
                
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


    def CalculerH1(self,posR):
        if posR[0] == self.target[0] and posR[1] != self.target[1]:
            return 1
        elif posR[0] != self.target[0] and posR[1] == self.target[1]:
            return 1
        else:
            if posR[0] == self.target[0] and posR[1] == self.target[1]:
                return 0
            else:
                return 2

    def ChoixNoeud(self):
        indice = 0
        evalF = 1000
        evalG = -1000
        noeud = self.O[0]
        for i in range(len(self.O)):
            if self.O[i].f < evalF: #boucle classique de recherche du noeud ayant le plus petit f et le plus grand G
                indice = i
                noeud = self.O[i]
                evalF = self.O[i].f
                evalG = self.O[i].g
            elif self.O[i].f == evalF:
                if self.O[i].g > evalG:
                    indice = i
                    noeud = self.O[i]
                    evalF = self.O[i].f
                    evalG = self.O[i].g
            else:
                None

        return indice


    def Recherche(self):
        posDepart = []
        for i in range(self.nbRobots):
            posInitRobot = np.where(self.cellules == i+1) #pos initiale des robot
            posDepart.append([posInitRobot[0],posInitRobot[1]])
        
        g = 0
        h = self.CalculerH1(posDepart[0]) #calcul h de robot bleu (avec sa pos initiale)
        f = g + h
        
        NoeudDepart = NoeudAlgoA(posDepart,g,f)
        
        self.O = [NoeudDepart] #couple pos [[posR1],..,[posRk]], g (nbCoups) , f 
        self.trouve = False

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