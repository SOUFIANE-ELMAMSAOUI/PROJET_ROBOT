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

## ⚙️ Configuration technique
### Paramètres capteurs
- #define IR_THRESHOLD 2000      // Seuil détection IR
- #define VL53_THRESHOLD 200     // Seuil distance VL53 (mm)
- #define w_cam 260              // Largeur image caméra
- #define h_cam 150              // Hauteur image caméra
 
### Asservissement moteurs
- Période d'échantillonnage : 5ms
- Contrôleur PI séparé pour chaque moteur
- Consigne par défaut : 500

### ROS Domain

- Domain ID : 11 (configurable via ROS_DOMAIN_ID)

## 🔧 Architecture logicielle
### Tâches FreeRTOS

- robot_control_task : Logique principale de contrôle
- microros_task : Communication ROS2
- displayTask : Gestion de l'affichage LCD
- sensor_data_collector_task : Collecte des données capteurs
- data_analyzer_task : Analyse et diagnostic

### Synchronisation

- Sémaphores binaires pour protection des données partagées
- Priorités configurées pour temps réel

## Affichage série
- Debug disponible via UART2 (115200 bauds) pour informations détaillées.
  
## 🚨 Sécurité
- Arrêt d'urgence automatique en cas d'obstacle (mode manuel)
- Timeouts sur les communications
- Surveillance de l'intégrité des données capteurs
- Protection mémoire avec allocation dynamique contrôlée
