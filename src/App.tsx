import React, { useState, useEffect, useRef } from 'react';
import { Camera, Cpu, Wifi, Eye, Zap, Shield, Play, Github, ChevronDown, Menu, X, ArrowRight, ExternalLink } from 'lucide-react';

interface MousePosition {
  x: number;
  y: number;
}

const RobotProjectSite: React.FC = () => {
  const [activeSection, setActiveSection] = useState<string>('hero');
  const [isMenuOpen, setIsMenuOpen] = useState<boolean>(false);
  const [mousePosition, setMousePosition] = useState<MousePosition>({ x: 0, y: 0 });
  const [activeCodeTab, setActiveCodeTab] = useState<string>('stm32');
  const heroRef = useRef<HTMLElement>(null);
  const [scrollY, setScrollY] = useState<number>(0);

  useEffect(() => {
    const handleScroll = () => setScrollY(window.scrollY);
    const handleMouseMove = (e: MouseEvent) => setMousePosition({ x: e.clientX, y: e.clientY });
    
    window.addEventListener('scroll', handleScroll);
    window.addEventListener('mousemove', handleMouseMove);
    
    return () => {
      window.removeEventListener('scroll', handleScroll);
      window.removeEventListener('mousemove', handleMouseMove);
    };
  }, []);

  const FloatingParticles: React.FC = () => {
    return (
      <div className="fixed inset-0 pointer-events-none overflow-hidden">
        {[...Array(50)].map((_, i) => (
          <div
            key={i}
            className="absolute w-1 h-1 bg-blue-400 rounded-full opacity-20 animate-pulse"
            style={{
              left: `${Math.random() * 100}%`,
              top: `${Math.random() * 100}%`,
              animationDelay: `${Math.random() * 3}s`,
              animationDuration: `${2 + Math.random() * 4}s`
            }}
          />
        ))}
      </div>
    );
  };

  const Navigation: React.FC = () => (
    <nav className="fixed top-0 left-0 right-0 z-50 bg-black/10 backdrop-blur-xl border-b border-white/10">
      <div className="max-w-7xl mx-auto px-6 py-4">
        <div className="flex justify-between items-center">
          <div className="flex items-center space-x-3">
            <div className="w-10 h-10 bg-gradient-to-r from-blue-500 to-purple-600 rounded-xl flex items-center justify-center">
              <Cpu className="w-6 h-6 text-white" />
            </div>
            <span className="text-xl font-bold text-white">RobotLab</span>
          </div>
          
          <div className="hidden md:flex items-center space-x-8">
            {['Accueil', 'Projet', 'Architecture', 'Démo', 'Code'].map((item) => (
              <a
                key={item}
                href={`#${item.toLowerCase()}`}
                className="text-white/80 hover:text-white transition-all duration-300 hover:scale-105"
              >
                {item}
              </a>
            ))}
          </div>
          
          <button
            onClick={() => setIsMenuOpen(!isMenuOpen)}
            className="md:hidden text-white"
          >
            {isMenuOpen ? <X /> : <Menu />}
          </button>
        </div>
      </div>
    </nav>
  );

  const Hero: React.FC = () => (
    <section ref={heroRef} className="relative min-h-screen flex items-center justify-center overflow-hidden">
      <div 
        className="absolute inset-0 bg-gradient-robot-hero"
        style={{
          transform: `translateY(${scrollY * 0.5}px)`
        }}
      />
      
      <div className="absolute inset-0 opacity-30">
        <div className="absolute top-1/4 left-1/4 w-96 h-96 bg-blue-500 rounded-full filter blur-3xl animate-pulse" />
        <div className="absolute bottom-1/4 right-1/4 w-96 h-96 bg-purple-500 rounded-full filter blur-3xl animate-pulse delay-1000" />
      </div>

      <div className="relative z-10 text-center max-w-6xl mx-auto px-6">
        <div className="mb-8 inline-flex items-center space-x-2 bg-white/10 backdrop-blur-sm rounded-full px-6 py-3 border border-white/20">
          <Zap className="w-5 h-5 text-yellow-400" />
          <span className="text-white/90">Projet Innovant STM32 + ROS2</span>
        </div>
        
        <h1 className="text-5xl md:text-7xl font-bold text-white mb-6 leading-tight">
          Robot Autonome
          <span className="block bg-gradient-to-r from-blue-400 to-purple-400 text-gradient-blue-purple">
            Multi-Modes
          </span>
        </h1>
        
        <p className="text-xl md:text-2xl text-white/80 mb-12 max-w-4xl mx-auto leading-relaxed">
          Système embarqué temps réel avec FreeRTOS, vision par ordinateur, 
          et interface de contrôle avancée via ROS2
        </p>
        
        <div className="flex flex-col sm:flex-row gap-6 justify-center items-center">
          <button className="group bg-gradient-to-r from-blue-500 to-purple-600 text-white px-8 py-4 rounded-xl font-semibold text-lg transition-all duration-300 hover:scale-105 hover:shadow-2xl hover:shadow-blue-500/25 flex items-center space-x-3">
            <Play className="w-6 h-6 group-hover:scale-110 transition-transform" />
            <span>Voir la démo</span>
          </button>
          <button className="border-2 border-white/30 text-white px-8 py-4 rounded-xl font-semibold text-lg hover:bg-white/10 transition-all duration-300 flex items-center space-x-3">
            <Github className="w-6 h-6" />
            <span>Code source</span>
          </button>
        </div>
      </div>
      
      <div className="absolute bottom-10 left-1/2 transform -translate-x-1/2 animate-bounce">
        <ChevronDown className="w-8 h-8 text-white/60" />
      </div>
    </section>
  );

  const ProjectOverview: React.FC = () => (
    <section className="py-20 bg-slate-50 relative overflow-hidden">
      <div className="max-w-7xl mx-auto px-6">
        <div className="text-center mb-16">
          <h2 className="text-4xl md:text-5xl font-bold text-slate-900 mb-6">
            Vision du Projet
          </h2>
          <p className="text-xl text-slate-600 max-w-3xl mx-auto">
            Développement d'un robot autonome multi-modes avec architecture distribuée 
            et communication temps réel via ROS2
          </p>
        </div>

        <div className="grid grid-cols-1 lg:grid-cols-2 gap-12 items-center mb-20">
          <div className="space-y-8">
            <div className="bg-white rounded-2xl p-8 shadow-xl border border-slate-200 hover:shadow-2xl transition-all duration-500 hover:scale-[1.02]">
              <div className="w-14 h-14 bg-gradient-to-r from-blue-500 to-purple-600 rounded-xl flex items-center justify-center mb-6">
                <Cpu className="w-8 h-8 text-white" />
              </div>
              <h3 className="text-2xl font-bold text-slate-900 mb-4">STM32 + FreeRTOS</h3>
              <ul className="space-y-3 text-slate-600">
                <li className="flex items-center space-x-3">
                  <div className="w-2 h-2 bg-green-500 rounded-full" />
                  <span>5+ tâches synchronisées avec sémaphores</span>
                </li>
                <li className="flex items-center space-x-3">
                  <div className="w-2 h-2 bg-green-500 rounded-full" />
                  <span>Asservissement PI des moteurs (5ms)</span>
                </li>
                <li className="flex items-center space-x-3">
                  <div className="w-2 h-2 bg-green-500 rounded-full" />
                  <span>Communication micro-ROS</span>
                </li>
                <li className="flex items-center space-x-3">
                  <div className="w-2 h-2 bg-green-500 rounded-full" />
                  <span>Diagnostic temps réel</span>
                </li>
              </ul>
            </div>

            <div className="bg-white rounded-2xl p-8 shadow-xl border border-slate-200 hover:shadow-2xl transition-all duration-500 hover:scale-[1.02]">
              <div className="w-14 h-14 bg-gradient-to-r from-green-500 to-teal-600 rounded-xl flex items-center justify-center mb-6">
                <Camera className="w-8 h-8 text-white" />
              </div>
              <h3 className="text-2xl font-bold text-slate-900 mb-4">Vision & Navigation</h3>
              <ul className="space-y-3 text-slate-600">
                <li className="flex items-center space-x-3">
                  <div className="w-2 h-2 bg-green-500 rounded-full" />
                  <span>OpenCV pour tracking couleur</span>
                </li>
                <li className="flex items-center space-x-3">
                  <div className="w-2 h-2 bg-green-500 rounded-full" />
                  <span>Navigation autonome intelligente</span>
                </li>
                <li className="flex items-center space-x-3">
                  <div className="w-2 h-2 bg-green-500 rounded-full" />
                  <span>Évitement d'obstacles multi-capteurs</span>
                </li>
              </ul>
            </div>
          </div>

          <div className="relative">
            <div className="bg-gradient-to-r from-slate-900 to-slate-700 rounded-3xl p-8 text-white">
              <h3 className="text-3xl font-bold mb-8">Images du Robot</h3>
              <div className="grid grid-cols-2 gap-4">
                {[
                  { id: 1, title: "Vue d'ensemble", image: "/images/robot-vue-ensemble.jpg" },
                  { id: 2, title: "Capteurs avant", image: "/images/robot-capteurs.jpg" },
                  { id: 3, title: "Architecture", image: "/images/robot-architecture.jpg" },
                  { id: 4, title: "Tests mobile", image: "/images/robot-test.jpg" }
                ].map((item) => (
                  <div
                    key={item.id}
                    className="robot-image-container group hover:scale-105 transition-all duration-300 cursor-pointer shadow-lg"
                  >
                    <img 
                      src={item.image} 
                      alt={item.title}
                      loading="lazy"
                    />
                    <div className="absolute inset-0 bg-black/30 group-hover:bg-black/20 transition-all duration-300" />
                    <div className="absolute bottom-0 left-0 right-0 p-3 z-10">
                      <p className="text-white font-medium text-sm bg-black/50 rounded px-2 py-1 backdrop-blur-sm">
                        {item.title}
                      </p>
                    </div>
                  </div>
                ))}
              </div>
            </div>
          </div>
        </div>
      </div>
    </section>
  );

  const ModesSection: React.FC = () => (
    <section className="py-20 bg-slate-900 relative overflow-hidden">
      <div className="absolute inset-0 bg-[radial-gradient(circle_at_50%_50%,rgba(59,130,246,0.1),transparent)]" />
      
      <div className="max-w-7xl mx-auto px-6 relative z-10">
        <div className="text-center mb-16">
          <h2 className="text-4xl md:text-5xl font-bold text-white mb-6">
            Modes de Fonctionnement
          </h2>
          <p className="text-xl text-slate-300 max-w-3xl mx-auto">
            Trois modes intelligents pour répondre à différents scénarios d'utilisation
          </p>
        </div>

        <div className="grid grid-cols-1 md:grid-cols-3 gap-8">
          {[
            {
              icon: <Wifi className="w-8 h-8" />,
              title: "Mode Manuel",
              description: "Contrôle à distance via interface PyQt5 avec protection anti-collision",
              features: ["Commande directionnelle", "Réglage vitesse temps réel", "Détection obstacles", "Feedback visuel"],
              color: "from-blue-500 to-cyan-500"
            },
            {
              icon: <Zap className="w-8 h-8" />,
              title: "Mode Aléatoire", 
              description: "Navigation autonome avec algorithme d'évitement intelligent",
              features: ["Exploration autonome", "Évitement automatique", "Changement direction", "Sécurité intégrée"],
              color: "from-purple-500 to-pink-500"
            },
            {
              icon: <Eye className="w-8 h-8" />,
              title: "Mode Tracking",
              description: "Suivi de cible par vision artificielle OpenCV avancée",
              features: ["Détection couleur HSV", "Calcul barycentre", "Suivi temps réel", "6 couleurs supportées"],
              color: "from-green-500 to-teal-500"
            }
          ].map((mode, index) => (
            <div
              key={index}
              className="group bg-white/5 backdrop-blur-sm rounded-2xl p-8 border border-white/10 hover:border-white/20 transition-all duration-500 hover:scale-105 hover:bg-white/10"
            >
              <div className={`w-16 h-16 bg-gradient-to-r ${mode.color} rounded-2xl flex items-center justify-center mb-6 group-hover:scale-110 transition-transform duration-300`}>
                {mode.icon}
              </div>
              
              <h3 className="text-2xl font-bold text-white mb-4">{mode.title}</h3>
              <p className="text-slate-300 mb-6 leading-relaxed">{mode.description}</p>
              
              <ul className="space-y-3">
                {mode.features.map((feature, i) => (
                  <li key={i} className="flex items-center space-x-3 text-slate-400">
                    <div className={`w-2 h-2 bg-gradient-to-r ${mode.color} rounded-full`} />
                    <span>{feature}</span>
                  </li>
                ))}
              </ul>
              
              <button className="mt-6 w-full bg-white/10 hover:bg-white/20 text-white py-3 rounded-xl transition-all duration-300 font-medium">
                En savoir plus
              </button>
            </div>
          ))}
        </div>
      </div>
    </section>
  );

  const ArchitectureSection: React.FC = () => (
    <section className="py-20 bg-gradient-to-br from-slate-50 to-blue-50">
      <div className="max-w-7xl mx-auto px-6">
        <div className="text-center mb-16">
          <h2 className="text-4xl md:text-5xl font-bold text-slate-900 mb-6">
            Architecture Système
          </h2>
          <p className="text-xl text-slate-600 max-w-3xl mx-auto">
            Architecture distribuée avec communication ROS2 entre trois composants principaux
          </p>
        </div>

        <div className="relative">
          <div className="bg-white rounded-3xl p-8 shadow-2xl border border-slate-200">
            <div className="grid grid-cols-1 lg:grid-cols-3 gap-8 items-center">
              
              <div className="text-center">
                <div className="bg-gradient-to-r from-blue-500 to-purple-600 rounded-2xl p-6 text-white mb-4 hover:scale-105 transition-transform duration-300">
                  <Cpu className="w-12 h-12 mx-auto mb-4" />
                  <h3 className="text-xl font-bold">PC Host</h3>
                  <p className="text-blue-100 text-sm">Interface IHM PyQt5</p>
                </div>
                <ul className="text-left space-y-2 text-sm text-slate-600">
                  <li>• Interface graphique temps réel</li>
                  <li>• Contrôle modes & vitesse</li>
                  <li>• Affichage diagnostic</li>
                  <li>• Visualisation caméra</li>
                </ul>
              </div>

              <div className="hidden lg:flex flex-col items-center space-y-4">
                <ArrowRight className="w-8 h-8 text-slate-400" />
                <div className="text-center">
                  <div className="bg-slate-100 rounded-lg px-4 py-2 text-sm font-medium text-slate-700">
                    ROS2 Topics
                  </div>
                  <div className="text-xs text-slate-500 mt-1">WiFi Network</div>
                </div>
                <ArrowRight className="w-8 h-8 text-slate-400 transform rotate-180" />
              </div>

              <div className="text-center">
                <div className="bg-gradient-to-r from-green-500 to-teal-600 rounded-2xl p-6 text-white mb-4 hover:scale-105 transition-transform duration-300">
                  <Camera className="w-12 h-12 mx-auto mb-4" />
                  <h3 className="text-xl font-bold">Raspberry Pi 4</h3>
                  <p className="text-green-100 text-sm">Vision & Traitement</p>
                </div>
                <ul className="text-left space-y-2 text-sm text-slate-600">
                  <li>• Acquisition webcam USB</li>
                  <li>• Traitement OpenCV</li>
                  <li>• Détection couleur HSV</li>
                  <li>• Node ROS2 camera</li>
                </ul>
              </div>
            </div>

            <div className="mt-8 flex justify-center">
              <div className="text-center max-w-md">
                <div className="bg-gradient-to-r from-orange-500 to-red-600 rounded-2xl p-6 text-white mb-4 hover:scale-105 transition-transform duration-300">
                  <Shield className="w-12 h-12 mx-auto mb-4" />
                  <h3 className="text-xl font-bold">STM32 Nucleo F411</h3>
                  <p className="text-orange-100 text-sm">Contrôle Temps Réel</p>
                </div>
                <div className="grid grid-cols-2 gap-4 text-left text-sm text-slate-600">
                  <div>
                    <p>• FreeRTOS (5+ tâches)</p>
                    <p>• Asservissement PI</p>
                    <p>• Encodeurs moteurs</p>
                  </div>
                  <div>
                    <p>• Capteurs IR + VL53L0X</p>
                    <p>• LCD Grove</p>
                    <p>• micro-ROS</p>
                  </div>
                </div>
              </div>
            </div>
          </div>

          <div className="grid grid-cols-2 md:grid-cols-4 gap-6 mt-12">
            {[
              { label: "Fréquence", value: "200Hz", desc: "Échantillonnage capteurs" },
              { label: "Latence", value: "<5ms", desc: "Commande moteurs" },
              { label: "Précision", value: "±2%", desc: "Asservissement vitesse" },
              { label: "Topics ROS2", value: "8+", desc: "Communication active" }
            ].map((stat, index) => (
              <div key={index} className="bg-white rounded-xl p-6 text-center shadow-lg border border-slate-200">
                <div className="text-3xl font-bold text-slate-900 mb-2">{stat.value}</div>
                <div className="text-lg font-medium text-slate-700 mb-1">{stat.label}</div>
                <div className="text-sm text-slate-500">{stat.desc}</div>
              </div>
            ))}
          </div>
        </div>
      </div>
    </section>
  );

  const CodeShowcase: React.FC = () => (
    <section className="py-20 bg-slate-900">
      <div className="max-w-7xl mx-auto px-6">
        <div className="text-center mb-16">
          <h2 className="text-4xl md:text-5xl font-bold text-white mb-6">
            Code & Implémentation
          </h2>
          <p className="text-xl text-slate-300 max-w-3xl mx-auto">
            Aperçu du code temps réel multi-plateformes avec synchronisation avancée
          </p>
        </div>

        <div className="bg-slate-800 rounded-2xl overflow-hidden border border-slate-700">
          <div className="flex border-b border-slate-700">
            {[
              { id: 'stm32', label: 'STM32 (C)', icon: <Cpu className="w-4 h-4" /> },
              { id: 'python', label: 'IHM (Python)', icon: <Eye className="w-4 h-4" /> },
              { id: 'cpp', label: 'Vision (C++)', icon: <Camera className="w-4 h-4" /> }
            ].map((tab) => (
              <button
                key={tab.id}
                onClick={() => setActiveCodeTab(tab.id)}
                className={`flex items-center space-x-2 px-6 py-4 font-medium transition-all duration-300 ${
                  activeCodeTab === tab.id
                    ? 'bg-slate-700 text-white border-b-2 border-blue-500'
                    : 'text-slate-400 hover:text-white hover:bg-slate-750'
                }`}
              >
                {tab.icon}
                <span>{tab.label}</span>
              </button>
            ))}
          </div>

          <div className="p-6">
            {activeCodeTab === 'stm32' && (
              <div className="text-slate-300 font-mono text-sm leading-relaxed">
                <div className="text-green-400 mb-2">// STM32 - Tâche contrôle robot avec FreeRTOS</div>
                <pre className="text-slate-300 overflow-x-auto">
{`static void robot_control_task(void *pvParameters)
{
    int speedL = 0, speedR = 0;
    float upL, upR, uiL = 0.0, uiR = 0.0;
    
    for (;;) {
        // Lecture capteurs avec protection critique
        speedL = quadEncoder_GetSpeedL();
        speedR = quadEncoder_GetSpeedR();
        mes_vl53 = VL53L0X_readRangeContinuousMillimeters();
        captDistIR_Get(tab_ir);
        
        // Détection obstacles multi-capteurs
        obstacle_detected = (tab_ir[0] > IR_THRESHOLD || 
                           tab_ir[1] > IR_THRESHOLD || 
                           mes_vl53 < VL53_THRESHOLD);
        
        // Asservissement PI optimisé
        errL = consigne_moteur_gauche - speedL;
        upL = Kp_L * errL;
        uiL = uiL + Kp_L * Ki_L * errL;
        motorLeft_SetDuty(upL + uiL + 100);
        
        vTaskDelay(SAMPLING_PERIOD_ms); // 5ms précis
    }
}`}
                </pre>
              </div>
            )}

            {activeCodeTab === 'python' && (
              <div className="text-slate-300 font-mono text-sm leading-relaxed">
                <div className="text-green-400 mb-2"># Interface IHM PyQt5 - Contrôle temps réel</div>
                <pre className="text-slate-300 overflow-x-auto">
{`class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Control - TEMPS RÉEL")
        
        # Publishers ROS2 optimisés
        self.mode_publisher = self.node.create_publisher(
            String, '/command/mode', 10)
        
        # Subscribers diagnostic avancé
        self.subscription_sensor_data = self.node.create_subscription(
            String, '/diagnostic/sensor_data', 
            self.sensor_data_callback, 10)
    
    def on_speed_set(self):
        speed = int(self.speed_input.text())
        if 0 <= speed <= 1000:
            self.send_speed_target_command(speed)
    
    def sensor_data_callback(self, msg):
        # Traitement diagnostic temps réel
        self.update_diagnostic_display()
        self.add_alert_to_history(msg.data)`}
                </pre>
              </div>
            )}

            {activeCodeTab === 'cpp' && (
              <div className="text-slate-300 font-mono text-sm leading-relaxed">
                <div className="text-green-400 mb-2">// Node ROS2 - Vision OpenCV avancée</div>
                <pre className="text-slate-300 overflow-x-auto">
{`class CameraTrackingNode : public rclcpp::Node
{
private:
    cv::Mat createColorMask(const cv::Mat& hsv, const std::string& color)
    {
        cv::Mat mask;
        if (color == "Rouge") {
            cv::Mat mask1, mask2;
            cv::inRange(hsv, red_lower1, red_upper1, mask1);
            cv::inRange(hsv, red_lower2, red_upper2, mask2);
            mask = mask1 | mask2;
        }
        
        // Filtrage morphologique optimisé
        cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_ELLIPSE, cv::Size(3,3));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        
        return mask;
    }
    
    void timer_callback()
    {
        cv::Mat frame, hsv;
        cap_ >> frame;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        
        cv::Mat mask = createColorMask(hsv, current_color_);
        cv::Moments m = cv::moments(mask, true);
        
        if (m.m00 > 1000) {
            int cx = static_cast<int>(m.m10 / m.m00);
            int cy = static_cast<int>(m.m01 / m.m00);
            xy_msg.data = std::to_string(cx) + "," + std::to_string(cy);
        }
        
        xy_pub_->publish(xy_msg);
    }
};`}
                </pre>
              </div>
            )}
          </div>
        </div>
      </div>
    </section>
  );

  const Footer: React.FC = () => (
    <footer className="py-12 bg-black text-white">
      <div className="max-w-7xl mx-auto px-6">
        <div className="grid grid-cols-1 md:grid-cols-4 gap-8 mb-8">
          <div>
            <div className="flex items-center space-x-3 mb-4">
              <div className="w-10 h-10 bg-gradient-to-r from-blue-500 to-purple-600 rounded-xl flex items-center justify-center">
                <Cpu className="w-6 h-6 text-white" />
              </div>
              <span className="text-xl font-bold">RobotLab</span>
            </div>
            <p className="text-slate-400 leading-relaxed">
              Projet innovant de robotique autonome avec architecture distribuée STM32 + ROS2
            </p>
          </div>
          
          <div>
            <h4 className="font-semibold mb-4">Technologies</h4>
            <ul className="space-y-2 text-slate-400">
              <li>STM32 + FreeRTOS</li>
              <li>Raspberry Pi 4</li>
              <li>ROS2 Humble</li>
              <li>OpenCV</li>
              <li>PyQt5</li>
            </ul>
          </div>
          
          <div>
            <h4 className="font-semibold mb-4">Fonctionnalités</h4>
            <ul className="space-y-2 text-slate-400">
              <li>Navigation autonome</li>
              <li>Vision artificielle</li>
              <li>Contrôle temps réel</li>
              <li>Interface graphique</li>
              <li>Diagnostic avancé</li>
            </ul>
          </div>
          
          <div>
            <h4 className="font-semibold mb-4">Liens</h4>
            <div className="flex space-x-4">
              <button className="w-10 h-10 bg-slate-800 rounded-lg flex items-center justify-center hover:bg-slate-700 transition-colors">
                <Github className="w-5 h-5" />
              </button>
              <button className="w-10 h-10 bg-slate-800 rounded-lg flex items-center justify-center hover:bg-slate-700 transition-colors">
                <ExternalLink className="w-5 h-5" />
              </button>
            </div>
          </div>
        </div>
        
        <div className="border-t border-slate-800 pt-8 text-center">
          <p className="text-slate-400">
            © 2024 Projet Robot Autonome. Technologies: STM32 • Raspberry Pi • ROS2 • OpenCV • PyQt5 • FreeRTOS
          </p>
        </div>
      </div>
    </footer>
  );

  return (
    <div className="min-h-screen bg-slate-900 overflow-x-hidden">
      <FloatingParticles />
      <Navigation />
      <Hero />
      <ProjectOverview />
      <ModesSection />
      <ArchitectureSection />
      <CodeShowcase />
      <Footer />
    </div>
  );
};

export default RobotProjectSite;