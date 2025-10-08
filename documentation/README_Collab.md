# Guide de collaboration — Projet Teensy 4.1

Bienvenue dans le projet.  
Ce guide explique comment récupérer le code, travailler dessus et partager les modifications avec l’équipe à l’aide de Git et GitHub.

---

## 1. Cloner le dépôt

Commence par récupérer le projet sur ta machine :

    git clone https://github.com/H2EcoInnov/prototype-control-system.git
    cd nom-du-projet

---

## 2. Créer ta branche de travail ( Optionnel )
Ce n’est pas forcément nécessaire si peu de personnes travaillent simultanément sur le code et qu’il n’y a pas de changements majeurs, mais cela reste une bonne pratique.
Crée une nouvelle branche pour ta fonctionnalité ou ton correctif (afin d’éviter les conflits sur la branche principale) :

    git checkout -b feature/ma-fonction

---

## 3. Travailler et sauvegarder ton code

Modifie le code, ajoute ou supprime des fichiers, puis enregistre ton travail localement :

    git add .
    git commit -m "Ajout de la nouvelle fonctionnalité"

Fais des commits clairs et réguliers, c’est plus simple pour suivre l’évolution du projet.

---

## 4. Récupérer les dernières modifications de l’équipe

Avant de pousser ton travail, mets-toi à jour avec la branche principale :

    git pull origin main

En cas de conflit, Git te demandera de les résoudre manuellement (VS Code aide beaucoup pour ça).

---

## 5. Envoyer ta branche sur GitHub

Pousse ton travail sur le dépôt distant :

    git push -u origin feature/ma-fonction

Tu peux ensuite créer une Pull Request sur GitHub pour proposer la fusion dans `main`,  
ou simplement fusionner directement si vous travaillez en confiance.

---

## 6. Compiler et exécuter sur Teensy 4.1

Le projet utilise PlatformIO.  
Pour compiler et téléverser le code sur la Teensy :

    pio run --target upload

Une fois le téléversement terminé, la Teensy redémarre et exécute le programme.

---

## Résumé des commandes utiles

    git clone <url>          # Récupérer le projet
    git checkout -b <nom>    # Créer une branche
    git add .                # Ajouter les fichiers modifiés
    git commit -m "<msg>"    # Enregistrer les changements
    git pull origin main     # Mettre à jour avec le dépôt
    git push -u origin <nom> # Envoyer ta branche sur GitHub
