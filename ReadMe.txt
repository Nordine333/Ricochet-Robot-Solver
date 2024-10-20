Ce code a été réalisé en utilisant Jupyter Notebook et Python.
Il utilise la bibliothèque NumPy (notamment pour représenter les instances) et la bibliothèque Matplotlib pour l'affichage de graphes.

Nous avons un fichier FonctionsInstances, qui fournit deux méthodes :

- generateRandomInstances : Cette méthode génère une instance en prenant en paramètre n la taille de la grille et k le nombre de robots. 
  Elle retourne les grilles représentant l'instance sous forme de trois tableaux numpy : cellules, verticaux et horizontaux.

- showgrid : Cette fonction affiche des instances. 
  Elle prend en paramètre n la taille de la grille et k le nombre de robots, ainsi que cellules, verticaux et horizontaux.

C'est un fichier .py.

Nous avons 4 classes qui implémentent différentes méthodes de recherche pour le jeu Ricochet Robots :

- RicochetRobotSolverProfondeur implémente la méthode de recherche en profondeur (chemin non optimal).

- RicochetRobotSolverLargeur implémente la méthode de recherche en largeur (chemin optimal).

- RicochetRobotSolverA_H1 (respectivement RicochetRobotSolverA_H2) implémente respectivement la méthode de recherche pour l'algorithme A* avec l'heuristique n°1
 (respectivement l'heuristique n°2) (chemin optimal).

Chaque classe prend lors de la construction les paramètres suivant :
- n, entier qui represente la taille de la grille (carré de taille n*n)
- k, entier qui represente le nombre de robots de l'instance
- bornesSupCoups, entier qui represente la limite de coups, 
 si le noeud à developper depasse cette limite nous ne le developperons pas (pour eviter les boucles infinies)
- cellules, verticaux et horizontaux (provenant de showgrid) qui caractèrise l'instance utilisée

Ces classes sont des fichiers .py.

Nous avons également 2 fichiers ComparaisonsMethodes et TestMethodes permettant de tester nos implémentations :

- TestMethodes va générer une instance et ensuite exécuter les différentes méthodes de recherche. Si une solution est trouvée, 
 elle affichera le chemin de la solution (l'évolution de la grille après chaque coup).

- ComparaisonsMethodes va générer des instances de paramètres variés, exécutera les méthodes de recherche et 
 réalisera des graphes montrant les performances des méthodes. Il y a une partie dédiée à l'analyse de la méthode de recherche en largeur, 
 puis une partie qui exécutera toutes les méthodes sur les mêmes instances dans le but de comparer leurs performances.

Ce sont des fichiers .ipynb.

Noureddine SIDKI - Cours RP Sorbonne Université - 2024
