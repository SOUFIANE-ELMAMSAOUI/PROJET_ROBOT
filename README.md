# 🤖 Robot STM32 avec micro-ROS
- Un robot autonome basé sur STM32 avec communication ROS2, capable de navigation autonome, contrôle manuel et suivi de cible par caméra.
## Fonctionnalités
### Modes de fonctionnement

- Mode Manuel : Contrôle direct via commandes ROS
- Mode Aléatoire : Navigation autonome avec évitement d'obstacles
- Mode Suivi Caméra : Suivi de cible détectée par caméra

### Capteurs embarqués

- Capteur VL53L0X : Mesure de distance laser (obstacle arrière)
- Capteurs IR : Détection d'obstacles frontaux (gauche/droite)
- Encodeurs : Mesure de vitesse des moteurs
- Écran LCD Grove : Affichage du mode actuel

## 📡 Communication ROS2
### Topics de commande

- /command/mode : Changement de mode (0=Manuel, 1=Aléatoire, 2=Caméra)
- /command/move : Commandes de mouvement (f=avancer, b=reculer, l=gauche, r=droite, s=stop)
- /camera/src_xy : Coordonnées de la cible (format x,y)

### Topics de données

- /sensor/motor_speed : Vitesse et consigne des moteurs
- /sensor/receive_obstacle : État des détecteurs d'obstacles
- /diagnostic/sensor_data : Données brutes des capteurs
- /diagnostic/analysis : Analyses et alertes
