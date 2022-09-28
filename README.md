# inf443-jf — « Le crime de l'Orient Express »

Projet final de Sacha et Gaspard pour le cours d’informatique graphique INF443


# Faire fonctionner le projet

Pour compiler le projet, il faut faire comme pour une scène de la CGP, notre scène est enregistrée sous le dossier projet_Braun_DeFommervault au fond du dossier scenes. 

Pour ce qui est de la navigation dans la scène, on peut utiliser la souris et les flèches directionnelles.

On peut générer des flocons en enlevant les commentaires devant les deux instructions pour générer des flocon dans la fonction.
display() de scene.cpp
C'est tres couteux en memoire (tourne à 7fds sur mon petit ordinateur), meme si l'utilisation de billboard avec plusieurs flocons permet reduire les calculs.

L'animation du train est générative : on donne en entrée la vitesse initiale et tout s'actualise en fonction de la vitesse accumulée et de la pente du terrain.

Le fichier ne se lance pas sur l'ordinateur de Gaspard, le lac apparait au dessus du terrain. N'hesitez pas à me contacter si vous avez aussi ce soucis.