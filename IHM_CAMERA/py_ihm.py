#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import sys
import numpy as np
from datetime import datetime

from PyQt5 import QtCore
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QColor, QPixmap, QImage, QPen, QFont, QBrush, QPainter
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QLineEdit,
    QPushButton, QVBoxLayout, QComboBox, QHBoxLayout, QGridLayout, 
    QGraphicsScene, QGraphicsView, QGraphicsRectItem, QSplitter,
    QSlider, QFrame, QRadioButton, QButtonGroup, QGraphicsTextItem,
    QSizePolicy, QSpacerItem, QTextEdit, QScrollArea
)

########################################################################

class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)

        # Configuration de base de la fenêtre
        self.setWindowTitle("Interface de Contrôle du Robot Mobile - TEMPS RÉEL avec DIAGNOSTIC")
        self.setMinimumSize(1600, 1000)
        self.setStyleSheet("""
            QMainWindow { background-color: #1e1e2e; color: #ffffff; }
            QFrame { background-color: #2d2d44; border-radius: 4px; padding: 6px; }
            QLabel { font-size: 12px; font-weight: bold; margin-bottom: 2px; color: #ffffff; }
            QLineEdit { 
                font-size: 12px; 
                padding: 4px; 
                border: 1px solid #555; 
                border-radius: 3px; 
                background-color: #333344; 
                color: #ffffff;
                min-height: 20px;
            }
            QPushButton { 
                font-size: 12px; 
                padding: 6px; 
                border: none; 
                border-radius: 3px; 
                background-color: #3498db; 
                color: white;
                min-height: 25px; 
            }
            QPushButton:hover { background-color: #2980b9; }
            QPushButton#btnEmergency { 
                background-color: #e74c3c; 
                font-weight: bold; 
                padding: 8px; 
                font-size: 14px;
                min-height: 40px;
            }
            QPushButton#btnEmergency:hover { background-color: #c0392b; }
            QPushButton#btnUp { background-color: #2ecc71; min-height: 40px; min-width: 40px; font-size: 16px; }
            QPushButton#btnDown { background-color: #e67e22; min-height: 40px; min-width: 40px; font-size: 16px; }
            QPushButton#btnLeft { background-color: #f1c40f; min-height: 40px; min-width: 40px; font-size: 16px; }
            QPushButton#btnRight { background-color: #3498db; min-height: 40px; min-width: 40px; font-size: 16px; }
            QPushButton#btnStop { background-color: #95a5a6; min-height: 40px; min-width: 40px; font-size: 16px; }
            QComboBox { 
                font-size: 12px; 
                padding: 4px; 
                border: 1px solid #555; 
                border-radius: 3px; 
                background-color: #3498db; 
                color: white;
                min-height: 25px;
            }
            QComboBox::drop-down {
                subcontrol-origin: padding;
                subcontrol-position: top right;
                width: 20px;
                border-left: 1px solid #555;
                background-color: #2980b9;
            }
            QComboBox QAbstractItemView {
                background-color: #333344;
                color: white;
                selection-background-color: #3498db;
                border: none;
                outline: none;
                padding: 0px;
                margin: 0px;
            }
            QRadioButton { 
                font-size: 12px; 
                color: #ffffff; 
                spacing: 6px;
            }
            QRadioButton::indicator {
                width: 14px;
                height: 14px;
                border: 2px solid #3498db;
                border-radius: 7px;
                background-color: #1e1e2e;
            }
            QRadioButton::indicator:checked {
                background-color: #3498db;
                border: 2px solid #3498db;
                border-radius: 7px;
                width: 14px;
                height: 14px;
            }
            QSlider::groove:horizontal { 
                border: 1px solid #555; 
                background: #333344; 
                height: 8px; 
                border-radius: 4px; 
            }
            QSlider::handle:horizontal { 
                background: #3498db; 
                border: 1px solid #777; 
                width: 16px; 
                height: 16px;
                margin: -4px 0; 
                border-radius: 8px; 
            }
            QGraphicsView { 
                border: 1px solid #555; 
                background-color: #000;
            }
            QTextEdit {
                background-color: #1a1a2e;
                color: #ffffff;
                border: 1px solid #555;
                border-radius: 3px;
                font-family: 'Courier New', monospace;
                font-size: 11px;
                padding: 5px;
            }
        """)

        # Création du widget principal
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Layout principal 
        main_grid = QGridLayout(central_widget)
        main_grid.setSpacing(8)
        main_grid.setContentsMargins(8, 8, 8, 8)

        # COLONNE GAUCHE: Mode + Données + Contrôle + Caméra
        left_column = QVBoxLayout()
        left_column.setSpacing(8)

        # LIGNE 1: Mode + Données + Contrôle 
        top_row_layout = QHBoxLayout()
        top_row_layout.setSpacing(8)

        # --- 1. Section Mode de Fonctionnement ---
        mode_frame = QFrame()
        mode_frame.setMaximumWidth(300)
        mode_layout = QVBoxLayout(mode_frame)
        mode_layout.setContentsMargins(15, 15, 15, 15)
        
        mode_title = QLabel("Mode de Fonctionnement")
        mode_title.setStyleSheet("font-size: 14px; color: #3498db;")
        mode_layout.addWidget(mode_title)
        
        # Options de mode avec Radio Buttons
        self.mode_group = QButtonGroup(self)
        
        self.rb_manual = QRadioButton("Manuel")
        self.rb_manual.setChecked(True)
        self.mode_group.addButton(self.rb_manual, 0)
        mode_layout.addWidget(self.rb_manual)
        
        self.rb_random = QRadioButton("Aléatoire")
        self.mode_group.addButton(self.rb_random, 1)
        mode_layout.addWidget(self.rb_random)
        
        self.rb_tracking = QRadioButton("Tracking (Suivi de couleur)")
        self.mode_group.addButton(self.rb_tracking, 2)
        mode_layout.addWidget(self.rb_tracking)
        
        # Connecter le changement de mode
        self.mode_group.buttonClicked.connect(self.on_mode_change)
        
        # Sélection de couleur pour le tracking
        self.tracking_config = QWidget()
        tracking_layout = QVBoxLayout(self.tracking_config)
        tracking_layout.setContentsMargins(0, 0, 0, 0)
        
        tracking_label = QLabel("Couleur à suivre:")
        tracking_layout.addWidget(tracking_label)
        
        color_options = QHBoxLayout()
        self.color_buttons = []
        
        colors = [
            ("#ff0000", "Rouge"),
            ("#00ff00", "Vert"),
            ("#0000ff", "Bleu"),
            ("#ffff00", "Jaune"),
            ("#ff00ff", "Magenta"),
            ("#00ffff", "Cyan")
        ]
        
        for color_hex, color_name in colors:
            color_btn = QPushButton()
            color_btn.setFixedSize(30, 30)
            color_btn.setStyleSheet(f"background-color: {color_hex}; border-radius: 16px;")
            color_btn.setToolTip(color_name)
            color_btn.clicked.connect(lambda checked, c=color_name: self.on_color_selected(c))
            color_options.addWidget(color_btn)
            self.color_buttons.append(color_btn)
        
        tracking_layout.addLayout(color_options)
        mode_layout.addWidget(self.tracking_config)
        self.tracking_config.setVisible(False)
        
        mode_layout.addStretch(1)
        top_row_layout.addWidget(mode_frame)
        
        # --- 2. Section Données du Robot ---
        data_frame = QFrame()
        data_frame.setMaximumWidth(350)
        data_layout = QVBoxLayout(data_frame)
        data_layout.setContentsMargins(15, 15, 15, 15)
        
        data_title = QLabel("Données du Robot")
        data_title.setStyleSheet("font-size: 14px; color: #3498db;")
        data_layout.addWidget(data_title)
        
        # Vitesse mesurée
        measured_speed_label = QLabel("Vitesse mesurée:")
        data_layout.addWidget(measured_speed_label)
        
        self.measured_speed_value = QLineEdit()
        self.measured_speed_value.setReadOnly(True)
        self.measured_speed_value.setText("0 tr/min")
        data_layout.addWidget(self.measured_speed_value)
        
        # Consigne de vitesse
        target_speed_label = QLabel("Consigne de vitesse:")
        data_layout.addWidget(target_speed_label)
        
        self.target_speed_value = QLineEdit()
        self.target_speed_value.setReadOnly(True)
        self.target_speed_value.setText("500 tr/min")
        data_layout.addWidget(self.target_speed_value)
        
        # Vitesse du robot en unités physiques
        robot_speed_label = QLabel("Vitesse du robot:")
        data_layout.addWidget(robot_speed_label)
        
        speed_layout = QGridLayout()
        speed_layout.setColumnStretch(0, 1)
        speed_layout.setColumnStretch(1, 2)
        
        speed_mps_label = QLabel("m/s:")
        speed_layout.addWidget(speed_mps_label, 0, 0)
        
        self.speed_mps_value = QLineEdit()
        self.speed_mps_value.setReadOnly(True)
        self.speed_mps_value.setText("0.00 m/s")
        speed_layout.addWidget(self.speed_mps_value, 0, 1)
        
        speed_kmh_label = QLabel("km/h:")
        speed_layout.addWidget(speed_kmh_label, 1, 0)
        
        self.speed_kmh_value = QLineEdit()
        self.speed_kmh_value.setReadOnly(True)
        self.speed_kmh_value.setText("0.00 km/h")
        speed_layout.addWidget(self.speed_kmh_value, 1, 1)
        
        data_layout.addLayout(speed_layout)
        
        # État des obstacles
        obstacle_label = QLabel("État des obstacles:")
        data_layout.addWidget(obstacle_label)
        
        self.obstacle_value = QLineEdit()
        self.obstacle_value.setReadOnly(True)
        self.obstacle_value.setText("Aucun obstacle détecté")
        data_layout.addWidget(self.obstacle_value)
        
        # Indicateurs visuels d'obstacles
        obstacle_indicators = QHBoxLayout()
        obstacle_indicators.setSpacing(10)
        
        # Obstacle avant
        self.front_obstacle_button = QPushButton("AVANT: OK")
        self.front_obstacle_button.setFixedHeight(35)
        self.front_obstacle_button.setStyleSheet("""
            QPushButton {
                background-color: #2ecc71;
                color: white;
                font-size: 12px;
                font-weight: bold;
                border: none;
                border-radius: 3px;
                padding: 5px;
                text-align: center;
            }
        """)
        self.front_obstacle_button.setEnabled(False)
        obstacle_indicators.addWidget(self.front_obstacle_button)
        
        # Obstacle arrière
        self.rear_obstacle_button = QPushButton("ARRIÈRE: OK")
        self.rear_obstacle_button.setFixedHeight(35)
        self.rear_obstacle_button.setStyleSheet("""
            QPushButton {
                background-color: #2ecc71;
                color: white;
                font-size: 12px;
                font-weight: bold;
                border: none;
                border-radius: 3px;
                padding: 5px;
                text-align: center;
            }
        """)
        self.rear_obstacle_button.setEnabled(False)
        obstacle_indicators.addWidget(self.rear_obstacle_button)
        
        data_layout.addLayout(obstacle_indicators)
        data_layout.addStretch(1)
        top_row_layout.addWidget(data_frame)
        
        # 3. Section Contrôle de Mouvement
        control_frame = QFrame()
        control_frame.setMaximumWidth(350)
        control_layout = QVBoxLayout(control_frame)
        control_layout.setContentsMargins(15, 15, 15, 15)
        
        control_title = QLabel("Contrôle Manuel")
        control_title.setStyleSheet("font-size: 14px; color: #3498db;")
        control_layout.addWidget(control_title)
        
        # Contrôle de consigne de vitesse
        speed_control_label = QLabel("Consigne de vitesse (TEMPS RÉEL):")
        control_layout.addWidget(speed_control_label)
        
        speed_input_layout = QHBoxLayout()
        self.speed_input = QLineEdit()
        self.speed_input.setPlaceholderText("0-1000")
        speed_input_layout.addWidget(self.speed_input)
        
        self.speed_set_button = QPushButton("Appliquer")
        self.speed_set_button.clicked.connect(self.on_speed_set)
        speed_input_layout.addWidget(self.speed_set_button)
        
        control_layout.addLayout(speed_input_layout)
        
        # Slider de consigne (0-1000 tr/min)
        slider_layout = QVBoxLayout()
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setMinimum(0)
        self.speed_slider.setMaximum(1000)
        self.speed_slider.setValue(500)
        self.speed_slider.setTickPosition(QSlider.TicksBelow)
        self.speed_slider.setTickInterval(100)
        self.speed_slider.valueChanged.connect(self.on_slider_changed)
        slider_layout.addWidget(self.speed_slider)
        
        slider_labels = QHBoxLayout()
        slow_label = QLabel("0")
        slow_label.setStyleSheet("font-size: 10px; font-weight: normal;")
        slider_labels.addWidget(slow_label)
        
        slider_labels.addStretch(1)
        
        fast_label = QLabel("1000 tr/min")
        fast_label.setStyleSheet("font-size: 10px; font-weight: normal;")
        slider_labels.addWidget(fast_label)
        
        slider_layout.addLayout(slider_labels)
        control_layout.addLayout(slider_layout)
        
        # Contrôle de direction avec boutons
        direction_layout = QGridLayout()
        
        # Bouton avant
        self.btn_forward = QPushButton("▲")
        self.btn_forward.setObjectName("btnUp")
        self.btn_forward.setFixedSize(40, 40)
        self.btn_forward.clicked.connect(lambda: self.on_movement_command("f"))
        direction_layout.addWidget(self.btn_forward, 0, 1)
        
        # Bouton gauche
        self.btn_left = QPushButton("◄")
        self.btn_left.setObjectName("btnLeft")
        self.btn_left.setFixedSize(40, 40)
        self.btn_left.clicked.connect(lambda: self.on_movement_command("l"))
        direction_layout.addWidget(self.btn_left, 1, 0)
        
        # Bouton stop
        self.btn_stop = QPushButton("■")
        self.btn_stop.setObjectName("btnStop")
        self.btn_stop.setFixedSize(40, 40)
        self.btn_stop.clicked.connect(lambda: self.on_movement_command("s"))
        direction_layout.addWidget(self.btn_stop, 1, 1)
        
        # Bouton droite
        self.btn_right = QPushButton("►")
        self.btn_right.setObjectName("btnRight")
        self.btn_right.setFixedSize(40, 40)
        self.btn_right.clicked.connect(lambda: self.on_movement_command("r"))
        direction_layout.addWidget(self.btn_right, 1, 2)
        
        # Bouton arrière
        self.btn_backward = QPushButton("▼")
        self.btn_backward.setObjectName("btnDown")
        self.btn_backward.setFixedSize(40, 40)
        self.btn_backward.clicked.connect(lambda: self.on_movement_command("b"))
        direction_layout.addWidget(self.btn_backward, 2, 1)
        
        # Centrer les boutons de direction
        direction_container = QHBoxLayout()
        direction_container.addStretch(1)
        direction_container.addLayout(direction_layout)
        direction_container.addStretch(1)
        
        control_layout.addLayout(direction_container)
        control_layout.addSpacing(15)
        
        # Bouton d'arrêt d'urgence
        self.emergency_button = QPushButton("ARRÊT D'URGENCE")
        self.emergency_button.setObjectName("btnEmergency")
        self.emergency_button.setMinimumHeight(50)
        self.emergency_button.clicked.connect(self.on_emergency_stop)
        control_layout.addWidget(self.emergency_button)
        
        control_layout.addStretch(1)
        top_row_layout.addWidget(control_frame)

        left_column.addLayout(top_row_layout)

        # === PARTIE CAMÉRA ===
        camera_frame = QFrame()
        camera_layout = QVBoxLayout(camera_frame)
        camera_layout.setContentsMargins(15, 15, 15, 15)
        
        camera_header = QHBoxLayout()
        
        camera_title = QLabel("Vision Caméra")
        camera_title.setStyleSheet("font-size: 14px; color: #3498db;")
        camera_header.addWidget(camera_title)
        
        camera_header.addStretch(1)
        
        camera_layout.addLayout(camera_header)
        
        # Vue caméra
        self.camera_scene = QGraphicsScene()
        self.camera_view = QGraphicsView(self.camera_scene)
        self.camera_view.setMinimumHeight(350)
        self.camera_view.setRenderHint(QPainter.Antialiasing)
        self.camera_view.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.camera_view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        
        camera_layout.addWidget(self.camera_view)
        
        left_column.addWidget(camera_frame)

        # === COLONNE DROITE: Diagnostic ===
        diagnostic_frame = QFrame()
        diagnostic_layout = QVBoxLayout(diagnostic_frame)
        diagnostic_layout.setContentsMargins(15, 15, 15, 15)
        
        diagnostic_title = QLabel("🔍 Diagnostic & Analyse Temps Réel")
        diagnostic_title.setStyleSheet("font-size: 14px; color: #e67e22; font-weight: bold;")
        diagnostic_layout.addWidget(diagnostic_title)
        
        # === Section 1: Données Collectées ===
        collected_title = QLabel("📊 Données Collectées")
        collected_title.setStyleSheet("color: #3498db; font-weight: bold; font-size: 13px; margin-top: 10px;")
        diagnostic_layout.addWidget(collected_title)
        
        # VL53L0X
        # vl53_layout = QHBoxLayout()
        # vl53_label = QLabel("📏 VL53L0X:")
        # vl53_label.setMinimumWidth(100)
        # vl53_layout.addWidget(vl53_label)
        # self.vl53_distance_value = QLineEdit()
        # self.vl53_distance_value.setReadOnly(True)
        # self.vl53_distance_value.setText("En attente...")
        # vl53_layout.addWidget(self.vl53_distance_value)
        # diagnostic_layout.addLayout(vl53_layout)
        
        # IR sensors
        ir_layout = QHBoxLayout()
        ir_label = QLabel("📡 IR G/D:")
        ir_label.setMinimumWidth(100)
        ir_layout.addWidget(ir_label)
        self.ir_left_value = QLineEdit()
        self.ir_left_value.setReadOnly(True)
        self.ir_left_value.setText("0")
        ir_layout.addWidget(self.ir_left_value)
        self.ir_right_value = QLineEdit()
        self.ir_right_value.setReadOnly(True)
        self.ir_right_value.setText("0")
        ir_layout.addWidget(self.ir_right_value)
        diagnostic_layout.addLayout(ir_layout)
        
        # Motors
        motor_layout = QHBoxLayout()
        motor_label = QLabel("⚙️ Moteurs:")
        motor_label.setMinimumWidth(100)
        motor_layout.addWidget(motor_label)
        self.motor_left_value = QLineEdit()
        self.motor_left_value.setReadOnly(True)
        self.motor_left_value.setText("0 tr/min")
        motor_layout.addWidget(self.motor_left_value)
        self.motor_right_value = QLineEdit()
        self.motor_right_value.setReadOnly(True)
        self.motor_right_value.setText("0 tr/min")
        motor_layout.addWidget(self.motor_right_value)
        diagnostic_layout.addLayout(motor_layout)
        
        # Timestamp
        timestamp_layout = QHBoxLayout()
        timestamp_label = QLabel("🕒 Collecte:")
        timestamp_label.setMinimumWidth(100)
        timestamp_layout.addWidget(timestamp_label)
        self.timestamp_value = QLineEdit()
        self.timestamp_value.setReadOnly(True)
        self.timestamp_value.setText("Aucune donnée")
        timestamp_layout.addWidget(self.timestamp_value)
        diagnostic_layout.addLayout(timestamp_layout)
        
        # === Section 2: Alertes ===
        alerts_title = QLabel("⚠️ Alertes & Analyses")
        alerts_title.setStyleSheet("color: #e74c3c; font-weight: bold; font-size: 13px; margin-top: 15px;")
        diagnostic_layout.addWidget(alerts_title)
        
        # État actuel
        status_layout = QHBoxLayout()
        status_label = QLabel("État:")
        status_label.setMinimumWidth(100)
        status_layout.addWidget(status_label)
        self.current_status_display = QLineEdit()
        self.current_status_display.setReadOnly(True)
        self.current_status_display.setText("Système normal")
        self.current_status_display.setStyleSheet("background-color: #2ecc71; color: white; font-weight: bold;")
        status_layout.addWidget(self.current_status_display)
        diagnostic_layout.addLayout(status_layout)
        
        # Historique des alertes
        history_label = QLabel("📋 Historique (5 dernières):")
        history_label.setStyleSheet("color: #f39c12; font-weight: bold; margin-top: 10px;")
        diagnostic_layout.addWidget(history_label)
        
        self.alerts_history = QTextEdit()
        self.alerts_history.setMinimumHeight(150)
        self.alerts_history.setPlainText("Aucune alerte pour le moment.\n")
        diagnostic_layout.addWidget(self.alerts_history)
        
        # Bouton effacer
        clear_history_btn = QPushButton("🗑️ Effacer")
        clear_history_btn.setMaximumHeight(30)
        clear_history_btn.clicked.connect(self.clear_alerts_history)
        diagnostic_layout.addWidget(clear_history_btn)
        
        # === Section 3: Statistiques ===
        stats_title = QLabel("📈 Statistiques")
        stats_title.setStyleSheet("color: #2ecc71; font-weight: bold; font-size: 13px; margin-top: 15px;")
        diagnostic_layout.addWidget(stats_title)
        
        # Statistiques en format compact
        stats_grid = QGridLayout()
        
        self.total_alerts_label = QLabel("Total alertes: 0")
        self.total_alerts_label.setStyleSheet("font-size: 11px;")
        stats_grid.addWidget(self.total_alerts_label, 0, 0)
        
        self.obstacles_detected_label = QLabel("Obstacles: 0")
        self.obstacles_detected_label.setStyleSheet("font-size: 11px;")
        stats_grid.addWidget(self.obstacles_detected_label, 0, 1)
        
        self.motor_speed_warnings_label = QLabel("Vitesse moteurs: 0")
        self.motor_speed_warnings_label.setStyleSheet("font-size: 11px;")
        stats_grid.addWidget(self.motor_speed_warnings_label, 1, 0)
        
        self.data_age_warnings_label = QLabel("Données anciennes: 0")
        self.data_age_warnings_label.setStyleSheet("font-size: 11px;")
        stats_grid.addWidget(self.data_age_warnings_label, 1, 1)
        
        diagnostic_layout.addLayout(stats_grid)
        
        # Bouton reset
        reset_stats_btn = QPushButton("🔄 Reset")
        reset_stats_btn.setMaximumHeight(30)
        reset_stats_btn.clicked.connect(self.reset_statistics)
        diagnostic_layout.addWidget(reset_stats_btn)
        
        diagnostic_layout.addStretch(1)

        # === ASSEMBLAGE FINAL DU LAYOUT PRINCIPAL ===       
        left_widget = QWidget()
        left_widget.setLayout(left_column)
        
        # Ajouter au grid principal
        main_grid.addWidget(left_widget, 0, 0)      # Colonne gauche
        main_grid.addWidget(diagnostic_frame, 0, 1) # Colonne droite 
        
        # Définir les proportions des colonnes
        main_grid.setColumnStretch(0, 2)  
        main_grid.setColumnStretch(1, 1)  

        # Variables pour les éléments graphiques de la caméra
        self.no_signal_text = None
        self.tracking_status = None
        
        # Variables de l'état du robot
        self.current_mode = 0
        self.selected_color = "Bleu"
        self.current_speed = 500
        self.last_sent_speed = -1
        self.front_obstacle = False
        self.rear_obstacle = False
        self.is_tracking_active = False
        self.tracking_x = 0
        self.tracking_y = 0
        self.camera_view_mode = 0
        
        # Timer pour le slider
        self.slider_timer = QTimer()
        self.slider_timer.setSingleShot(True)
        self.slider_timer.timeout.connect(self.send_pending_speed_command)
        self.pending_speed = None
        self.is_user_moving_slider = False
        
        # Variables pour le diagnostic
        self.diagnostic_data = {
            # 'vl53_distance': 0,
            'ir_left': 0,
            'ir_right': 0,
            'motor_left': 0,
            'motor_right': 0,
            'timestamp': 0
        }
        
        # Compteurs pour statistiques
        self.stats = {
            'total_alerts': 0,
            'obstacles_detected': 0,
            'motor_speed_warnings': 0,
            'data_age_warnings': 0
        }
        
        # Historique des alertes
        self.alerts_history_list = []
        
        # Créer les éléments initiaux de la caméra
        self.create_camera_elements()

        # Initialisation de ROS2
        try:
            rclpy.init(args=None)
            self.node = Node('py_ihm_node')
            
            self.node.get_logger().info("Robot ID: 10")
            
            # Publishers
            self.mode_publisher = self.node.create_publisher(String, '/command/mode', 10)
            self.movement_publisher = self.node.create_publisher(String, '/command/move', 10)
            self.publisher_tracking_color = self.node.create_publisher(String, '/command/tracking_color', 10)
           
            # Subscribers existants
            self.subscription_speed = self.node.create_subscription(String, '/sensor/motor_speed', self.movement_speed_callback, 10)
            self.subscription_obstacle = self.node.create_subscription(String, '/sensor/receive_obstacle', self.obstacle_callback, 10)
            self.subscription_camera = self.node.create_subscription(Image, '/camera/src_frame', self.img_callback, 10)
            self.subscription_camera_xy = self.node.create_subscription(String, '/camera/src_xy', self.on_tracking_data, 10)

            # Nouveaux subscribers pour le diagnostic
            self.subscription_sensor_data = self.node.create_subscription(String, '/diagnostic/sensor_data', self.sensor_data_callback, 10)
            self.subscription_analysis = self.node.create_subscription(String, '/diagnostic/analysis', self.analysis_callback, 10)

            print("ROS2 initialisé avec succès (avec diagnostic)")
        except Exception as e:
            print(f"Erreur lors de l'initialisation ROS2: {e}")
            self.node = None

        # Timer pour les événements ROS
        if self.node:
            self.timer = QTimer()
            self.timer.timeout.connect(self.onTimerTick)
            self.timer.start(100)
        else:
            self.timer = None
        
        # Envoyer le mode initial
        if self.node:
            self.send_mode_command(0)
            import time
            time.sleep(0.1)
            self.send_speed_target_command(self.current_speed)
            print(f"*** INITIALISATION: Mode=0, Consigne={self.current_speed} ***")

    # === MÉTHODES POUR LE DIAGNOSTIC ===
    
    def sensor_data_callback(self, msg):
        """Callback pour recevoir les données des capteurs collectées par le STM32"""
        try:
            data_parts = msg.data.split(',')
            for part in data_parts:
                if '=' in part:
                    key, value = part.split('=')
                    key = key.strip()
                    value = int(value.strip())
                    
                    if key in self.diagnostic_data:
                        self.diagnostic_data[key] = value
            
            self.update_diagnostic_display()
            
        except Exception as e:
            print(f"Erreur parsing données capteurs: {e}")
    
    def analysis_callback(self, msg):
        """Callback pour recevoir les analyses du STM32"""
        try:
            analysis_text = msg.data
            self.add_alert_to_history(analysis_text)
            self.update_current_status(analysis_text)
            self.update_statistics(analysis_text)
            
        except Exception as e:
            print(f"Erreur parsing analyse: {e}")
    
    def update_diagnostic_display(self):
        """Met à jour l'affichage des données collectées"""
        # self.vl53_distance_value.setText(f"{self.diagnostic_data['vl53_distance']} mm")
        self.ir_left_value.setText(str(self.diagnostic_data['ir_left']))
        self.ir_right_value.setText(str(self.diagnostic_data['ir_right']))
        self.motor_left_value.setText(f"{self.diagnostic_data['motor_left']} tr/min")
        self.motor_right_value.setText(f"{self.diagnostic_data['motor_right']} tr/min")
        
        if self.diagnostic_data['timestamp'] > 0:
            timestamp_ms = self.diagnostic_data['timestamp']
            current_time = datetime.now().strftime("%H:%M:%S")
            self.timestamp_value.setText(f"{current_time} (STM32: {timestamp_ms}ms)")
        
    def add_alert_to_history(self, alert_text):
        """Ajoute une alerte à l'historique"""
        current_time = datetime.now().strftime("%H:%M:%S")
        formatted_alert = f"[{current_time}] {alert_text}"
        
        self.alerts_history_list.append(formatted_alert)
        
        # Garder seulement les 5 dernières
        if len(self.alerts_history_list) > 5:
            self.alerts_history_list = self.alerts_history_list[-5:]
        
        self.alerts_history.setPlainText('\n'.join(reversed(self.alerts_history_list)))
        
        cursor = self.alerts_history.textCursor()
        cursor.movePosition(cursor.End)
        self.alerts_history.setTextCursor(cursor)
    
    def update_current_status(self, analysis_text):
        """Met à jour le statut actuel"""
        if "TRÈS PROCHE" in analysis_text or "WARNING" in analysis_text:
            self.current_status_display.setText("⚠️ ALERTE: " + analysis_text[:30] + "...")
            self.current_status_display.setStyleSheet("background-color: #e74c3c; color: white; font-weight: bold;")
        elif "Obstacle détecté" in analysis_text or "Capteur IR actif" in analysis_text:
            self.current_status_display.setText("⚠️ ATTENTION: " + analysis_text[:30] + "...")
            self.current_status_display.setStyleSheet("background-color: #f39c12; color: white; font-weight: bold;")
        elif "Différence vitesse" in analysis_text:
            self.current_status_display.setText("⚠️ INFO: " + analysis_text[:30] + "...")
            self.current_status_display.setStyleSheet("background-color: #3498db; color: white; font-weight: bold;")
        else:
            QTimer.singleShot(5000, self.reset_status_to_normal)
    
    def reset_status_to_normal(self):
        """Remet le statut à normal"""
        self.current_status_display.setText("✅ Système normal")
        self.current_status_display.setStyleSheet("background-color: #2ecc71; color: white; font-weight: bold;")
    
    def update_statistics(self, analysis_text):
        """Met à jour les statistiques"""
        self.stats['total_alerts'] += 1
        
        if "Obstacle" in analysis_text:
            self.stats['obstacles_detected'] += 1
        if "vitesse moteurs" in analysis_text:
            self.stats['motor_speed_warnings'] += 1
        if "données anciennes" in analysis_text:
            self.stats['data_age_warnings'] += 1
        
        self.total_alerts_label.setText(f"Total alertes: {self.stats['total_alerts']}")
        self.obstacles_detected_label.setText(f"Obstacles: {self.stats['obstacles_detected']}")
        self.motor_speed_warnings_label.setText(f"Vitesse moteurs: {self.stats['motor_speed_warnings']}")
        self.data_age_warnings_label.setText(f"Données anciennes: {self.stats['data_age_warnings']}")
    
    def clear_alerts_history(self):
        """Efface l'historique des alertes"""
        self.alerts_history_list.clear()
        self.alerts_history.setPlainText("Historique effacé.\n")
    
    def reset_statistics(self):
        """Remet les statistiques à zéro"""
        self.stats = {
            'total_alerts': 0,
            'obstacles_detected': 0,
            'motor_speed_warnings': 0,
            'data_age_warnings': 0
        }
        self.update_statistics("")
        
    # === MÉTHODES EXISTANTES ===

    def create_camera_elements(self):
        """Crée les éléments graphiques de base pour la caméra"""
        self.camera_scene.clear()
        self.no_signal_text = None
        self.tracking_status = None
        
        self.no_signal_text = self.camera_scene.addText("En attente du flux vidéo...")
        self.no_signal_text.setDefaultTextColor(QColor(255, 255, 255))
        font = QFont()
        font.setPointSize(14)
        self.no_signal_text.setFont(font)
        self.no_signal_text.setPos(100, 150)
        
        self.tracking_status = self.camera_scene.addText("Tracking: En attente de cible")
        self.tracking_status.setDefaultTextColor(QColor(255, 255, 0))
        self.tracking_status.setFont(font)
        self.tracking_status.setPos(50, 50)
        self.tracking_status.setVisible(self.current_mode == 2)

    def on_mode_change(self, button):
        mode_id = self.mode_group.id(button)
        self.current_mode = mode_id
        
        self.tracking_config.setVisible(mode_id == 2)
        
        try:
            if self.tracking_status and not self.tracking_status.scene() is None:
                self.tracking_status.setVisible(mode_id == 2)
        except RuntimeError:
            self.create_camera_elements()
        
        self.send_mode_command(mode_id)
        print(f"Mode changé: {mode_id}")

    def on_color_selected(self, color):
        self.selected_color = color
        if self.node:
            msg = String()
            msg.data = color
            self.publisher_tracking_color.publish(msg)
            print(f"Couleur envoyée : {color}")

    def on_speed_set(self):
        try:
            speed = int(self.speed_input.text())
            if 0 <= speed <= 1000:
                self.current_speed = speed
                self.speed_slider.blockSignals(True)
                self.speed_slider.setValue(speed)
                self.speed_slider.blockSignals(False)
                
                self.send_speed_target_command(speed)
                print(f"*** BOUTON APPLIQUÉ: {speed} ***")
            else:
                print("La consigne doit être entre 0 et 1000")
        except ValueError:
            print("Veuillez entrer une valeur numérique valide")

    def on_slider_changed(self, value):
        self.current_speed = value
        self.speed_input.setText(str(value))
        self.is_user_moving_slider = True
        
        self.pending_speed = value
        self.slider_timer.stop()
        self.slider_timer.start(50)

    def send_pending_speed_command(self):
        if self.pending_speed is not None:
            self.send_speed_target_command(self.pending_speed)
            print(f"*** SLIDER TEMPS RÉEL: {self.pending_speed} ***")
            self.pending_speed = None
            QTimer.singleShot(500, lambda: setattr(self, 'is_user_moving_slider', False))

    def on_movement_command(self, direction):
        self.send_movement_command(direction, self.current_speed)
        direction_names = {"f": "avant", "b": "arrière", "l": "gauche", "r": "droite", "s": "stop"}
        print(f"Mouvement: {direction_names.get(direction, direction)}")

    def on_emergency_stop(self):
        self.send_movement_command("s", 0)
        print("ARRÊT D'URGENCE activé")

    def on_camera_mode_changed(self, index):
        self.camera_view_mode = index
        print(f"Mode caméra changé: {index}")

    def send_mode_command(self, mode):
        if self.node:
            msg = String()
            msg.data = str(mode)
            self.mode_publisher.publish(msg)

    def send_speed_target_command(self, speed):
        if self.node:
            msg = String()
            msg.data = f"v{speed}"
            self.movement_publisher.publish(msg)
            print(f"*** COMMANDE VITESSE ENVOYEE: '{msg.data}' (consigne: {speed}) ***")

    def send_movement_command(self, direction, speed=None):
        if self.node:
            if speed is None:
                speed = self.current_speed
                
            msg = String()
            msg.data = f"{direction}{speed}"
            self.movement_publisher.publish(msg)
            print(f"*** COMMANDE MOUVEMENT ENVOYEE: '{msg.data}' (direction: {direction}, vitesse: {speed}) ***")

    def movement_speed_callback(self, msg):
        message_text = msg.data
        D = 0.098
        
        try:
            measured_speed_str = message_text.split("speed=#")[1].split(",")[0]
            target_speed_str = message_text.split("consigne=#")[1]

            measured_speed = float(measured_speed_str.strip())
            target_speed = float(target_speed_str.strip())

            speed_km = measured_speed * np.pi * D / 1000 * 60
            speed_m = speed_km / 3.6

            self.measured_speed_value.setText(f'{measured_speed:.0f} tr/min')
            self.target_speed_value.setText(f'{target_speed:.0f} tr/min')
            self.speed_kmh_value.setText(f'{speed_km:.2f} km/h')
            self.speed_mps_value.setText(f'{speed_m:.2f} m/s')
            
            if (not self.is_user_moving_slider and 
                not self.slider_timer.isActive() and 
                abs(self.current_speed - target_speed) > 20):
                
                self.speed_slider.blockSignals(True)
                self.speed_slider.setValue(int(target_speed))
                self.speed_slider.blockSignals(False)
                
                self.current_speed = int(target_speed)
                self.speed_input.setText(str(self.current_speed))
            
        except (IndexError, ValueError) as e:
            print(f"Erreur lors de l'analyse des données de vitesse : {e}")

    def obstacle_callback(self, msg):
        message_text = msg.data
        
        try:
            if "front_obstacle=#" in message_text and "rear_obstacle=#" in message_text:
                front_str = message_text.split("front_obstacle=#")[1].split(",")[0]
                rear_str = message_text.split("rear_obstacle=#")[1]
                
                front_obstacle_detected = int(front_str.strip())
                rear_obstacle_detected = int(rear_str.strip())
                
                self.front_obstacle = (front_obstacle_detected == 1)
                self.rear_obstacle = (rear_obstacle_detected == 1)
                
                if self.front_obstacle and self.rear_obstacle:
                    self.obstacle_value.setText("Obstacles détectés avant ET arrière")
                elif self.front_obstacle:
                    self.obstacle_value.setText("Obstacle détecté à l'AVANT")
                elif self.rear_obstacle:
                    self.obstacle_value.setText("Obstacle détecté à l'ARRIÈRE") 
                else:
                    self.obstacle_value.setText("Aucun obstacle détecté")
            
            self.update_obstacle_indicators()
            
        except Exception as e:
            print(f"Erreur lors du traitement des données d'obstacle: {e}")

    def update_obstacle_indicators(self):
        front_style = """
            QPushButton {
                background-color: %s;
                color: white;
                font-size: 12px;
                font-weight: bold;
                border: none;
                border-radius: 3px;
                padding: 5px;
            }
        """
        
        if self.front_obstacle:
            self.front_obstacle_button.setStyleSheet(front_style % "#e74c3c")
            self.front_obstacle_button.setText("AVANT: OBSTACLE")
        else:
            self.front_obstacle_button.setStyleSheet(front_style % "#2ecc71")
            self.front_obstacle_button.setText("AVANT: OK")
        
        if self.rear_obstacle:
            self.rear_obstacle_button.setStyleSheet(front_style % "#e74c3c")
            self.rear_obstacle_button.setText("ARRIÈRE: OBSTACLE")
        else:
            self.rear_obstacle_button.setStyleSheet(front_style % "#2ecc71")
            self.rear_obstacle_button.setText("ARRIÈRE: OK")

    def img_callback(self, msg):
        try:
            width = msg.width
            height = msg.height
            image = QImage(msg.data, width, height, QImage.Format_RGB888)
            
            pixmap = QPixmap.fromImage(image)
            
            self.camera_scene.clear()
            self.no_signal_text = None
            self.tracking_status = None
            
            self.camera_scene.addPixmap(pixmap)
            
            self.camera_scene.setSceneRect(0, 0, width, height)
            self.camera_view.fitInView(self.camera_scene.sceneRect(), Qt.KeepAspectRatio)
            
            if self.current_mode == 2 and self.is_tracking_active:
                target_size = 20
                target_rect = QGraphicsRectItem(
                    self.tracking_x - target_size/2, 
                    self.tracking_y - target_size/2, 
                    target_size, 
                    target_size
                )
                target_rect.setPen(QPen(QColor(255, 0, 0), 2))
                target_rect.setBrush(QBrush(QColor(255, 0, 0, 128)))
                self.camera_scene.addItem(target_rect)
                
                tracking_text = QGraphicsTextItem(f"Cible: ({self.tracking_x}, {self.tracking_y})")
                tracking_text.setDefaultTextColor(QColor(255, 255, 0))
                tracking_text.setPos(10, 10)
                self.camera_scene.addItem(tracking_text)
            
            if self.current_mode == 2:
                self.tracking_status = self.camera_scene.addText(
                    f"Tracking: {'Cible détectée' if self.is_tracking_active else 'En attente de cible'}"
                )
                self.tracking_status.setDefaultTextColor(QColor(255, 255, 0))
                font = QFont()
                font.setPointSize(12)
                self.tracking_status.setFont(font)
                self.tracking_status.setPos(10, height - 40)
            
        except Exception as e:
            print(f"Erreur lors du traitement de l'image: {e}")
            self.create_camera_elements()

    def on_tracking_data(self, msg):
        try:
            coords = msg.data
            if coords == "0,0":
                self.is_tracking_active = False
            else:
                x, y = map(int, coords.split(','))
                self.tracking_x = x
                self.tracking_y = y
                self.is_tracking_active = True
                
            print(f"Données tracking: {coords}, Active: {self.is_tracking_active}")
        except Exception as e:
            print(f"Erreur lors du traitement des données de tracking: {e}")

    def onTimerTick(self):
        try:
            if self.node:
                rclpy.spin_once(self.node, timeout_sec=0.1)
        except Exception as e:
            print(f"Erreur ROS: {e}")

    def closeEvent(self, event):
        try:
            if hasattr(self, 'slider_timer') and self.slider_timer:
                self.slider_timer.stop()
            if hasattr(self, 'timer') and self.timer:
                self.timer.stop()
            if hasattr(self, 'node') and self.node:
                self.node.destroy_node()
                rclpy.shutdown()
        except Exception as e:
            print(f"Erreur lors de la fermeture: {e}")
        event.accept()

########################################################################
def main(args=None):
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()
########################################################################
