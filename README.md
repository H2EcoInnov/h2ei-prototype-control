# prototype-control-system

Ce dépôt contient le code de contrôle du prototype H2 Eco'Innov pour la saison 2024-2025.

Il est destiné aux trois cartes de commande du prototype :

- La main card : carte centrale responsable de la majorité des opérations. Elle gère l’asservissement de la pile à combustible, les règles de sécurité et la communication avec les autres cartes. Elle est également liée à l’arrêt d’urgence, entièrement câblé en logique pure.

- La carte pilote : elle transmet les entrées du volant à la main card et affiche des informations au pilote.

- La carte télémétrie (optionnelle et prévue dans un second temps) : elle permet de traiter différentes données liées à la gestion et aux performances du prototype, et assure la communication avec l’extérieur.

Le code est structuré comme un projet PlatformIO, ce qui facilite sa gestion et la collaboration. L’environnement de travail repose donc sur l’extension PlatformIO de VS Code.
Une fois le projet ouvert dans VS Code avec l’extension PlatformIO, il suffit de connecter la carte Teensy 4.1 via USB, puis de lancer la compilation et le téléversement avec la commande :
```
PlatformIO: Upload
```
ou via le raccourci Ctrl + Alt + U.

Il est prévu de fournir régulièrement des versions compatibles avec l’Arduino IDE, mais le fichier situé dans /src/main.cpp peut être utilisé dans ce dernier en copiant son contenu dans un fichier .ino, à condition de bien gérer les bibliothèques nécessaires et d’adapter la structure si besoin.

Pour éditer et collaborer sur ce code, vous pouvez cloner ce dépôt avec git et la commande :
```
git clone https://github.com/H2EcoInnov/prototype-control-system.git
```
Pour plus d'information sur la collaboration sur des projets de code vus pouvez suivre ce tutoriel sur git : https://www.w3schools.com/git/ ou lire le READ ME Git présent dans : /documentation/README_Git.md

Les collaborateurs du projet resteront autant que possible disponibles pour assurer la transmission des informations. Leurs contacts sont disponibles ci-dessous :
- Corentin Royet, corentin.royet@proton.me
