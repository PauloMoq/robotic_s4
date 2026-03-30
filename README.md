# PROJET : SIMULATION ROBOT MOBILE (MVC)

## AUTEUR
Paul HURDEBOURCQ (PauloMoq)

## FORMATION
Master 2 Data&IA (Alternance)

## ANNEE
2024-2026

# TO-DO
## QOL :
- Clean up affichage (print sur la fenêtre moches + jeu fini)
- Opti dessin des oeufs
- Son de collecte
- Skin de robots

## TECH : 
- Ajouter une sortie au labyrinthe 
- Robot doit trouver la sortie après la collecte des 3 oeufs
- 2 Robots avec déplacements différents (2eme Robot en A* ?)

## RENDU :
- Diapo
- Clean code
- Clean README

---

## INSTALLATION ET LANCEMENT

- Il faut avoir Python installé.
- Il faut installer la bibliothèque Pygame : tapez `pip install pygame` dans votre terminal.
- Pour lancer le jeu : tapez `python main.py` à la racine du dossier.

---

## COMMANDES DU ROBOT

Le robot réagit différemment selon le moteur choisi dans le code :

### Pour le Moteur Différentiel :
- Flèches HAUT / BAS : Avancer et reculer.
- Flèches GAUCHE / DROITE : Tourner sur place.

### Pour le Moteur Omnidirectionnel :
- Flèches : Se déplacer dans toutes les directions (X et Y).
- Touches Q et D : Tourner sur soi-même.

---

## Système
- Touche ECHAP : Quitter proprement.
- CTRL + C : Quitter depuis le terminal.

---

## ARCHITECTURE DU CODE (MVC)

Le projet est séparé en 3 parties pour que le code soit propre :

### LE MODELE (`robot/model`)
C'est le cerveau et la physique. Il calcule la position du robot, gère les collisions avec les obstacles rouges et fait revenir le robot de l'autre côté de l'écran s'il sort (wrap-around).

### LA VUE (`robot/vue`)
C'est ce qu'on voit à l'écran. Elle dessine la grille, les obstacles et le robot en transformant les mètres en pixels.

### LE CONTROLEUR (`robot/controler`)
C'est le lien avec le clavier. Il lit les touches pressées et donne les ordres de vitesse au robot.

---

## POINTS CLÉS DU TP
- Collisions : Le robot se bloque contre les obstacles ronds.
- Monde infini : Si on sort à droite, on revient à gauche.
- Modularité : On peut changer de moteur ou de vue sans tout recoder, c'est l'avantage du MVC.