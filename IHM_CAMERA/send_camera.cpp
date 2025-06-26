#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <map>
#include <string>

struct ColorRange {
    cv::Scalar lower;
    cv::Scalar upper;
    std::string name;
};

class CameraTrackingNode : public rclcpp::Node
{
public:
    CameraTrackingNode() : Node("camera_tracking_node"), current_color_("Bleu")
    {
        // Initialisation des couleurs HSV
        initColorRanges();
        
        // Publishers - CORRIGÉ : Utiliser uniquement ROS2
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/src_frame", 10);
        xy_pub_ = this->create_publisher<std_msgs::msg::String>("/camera/src_xy", 10);
        
        // Subscriber pour recevoir la couleur sélectionnée depuis l'IHM
        color_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/command/tracking_color", 10,
            std::bind(&CameraTrackingNode::color_callback, this, std::placeholders::_1));
        
        // Timer to read camera frame - OPTIMISÉ : 50ms au lieu de 33ms
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), // 20 FPS au lieu de 30 FPS pour réduire la charge
            std::bind(&CameraTrackingNode::timer_callback, this));
        
        // Initialiser la caméra
        cap_.open(0);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Erreur : Impossible d'ouvrir la caméra");
            rclcpp::shutdown();
            return;
        } else {
            RCLCPP_INFO(this->get_logger(), "Caméra initialisée avec succès.");
            RCLCPP_INFO(this->get_logger(), "Couleur de tracking par défaut : %s", current_color_.c_str());
            RCLCPP_INFO(this->get_logger(), "Communication uniquement via ROS2 - Pas d'UART direct");
        }
        
        // Configuration caméra pour de meilleures performances
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 320);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 240);
        cap_.set(cv::CAP_PROP_FPS, 20);
    }

private:
    void initColorRanges()
    {
        // Définition des plages HSV pour chaque couleur - OPTIMISÉ
        color_ranges_["Rouge"] = {cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), "Rouge"};
        color_ranges_["Rouge2"] = {cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), "Rouge"}; // Rouge wraparound
        color_ranges_["Vert"] = {cv::Scalar(40, 50, 50), cv::Scalar(80, 255, 255), "Vert"};
        color_ranges_["Bleu"] = {cv::Scalar(100, 150, 50), cv::Scalar(130, 255, 255), "Bleu"};
        color_ranges_["Jaune"] = {cv::Scalar(20, 100, 100), cv::Scalar(30, 255, 255), "Jaune"};
        color_ranges_["Magenta"] = {cv::Scalar(140, 100, 100), cv::Scalar(170, 255, 255), "Magenta"};
        color_ranges_["Cyan"] = {cv::Scalar(80, 100, 100), cv::Scalar(100, 255, 255), "Cyan"};
        
        RCLCPP_INFO(this->get_logger(), "Couleurs initialisées : Rouge, Vert, Bleu, Jaune, Magenta, Cyan");
    }
    
    void color_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string new_color = msg->data;
        if (color_ranges_.find(new_color) != color_ranges_.end()) {
            current_color_ = new_color;
            RCLCPP_INFO(this->get_logger(), "Couleur de tracking changée : %s", current_color_.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Couleur non reconnue : %s", new_color.c_str());
        }
    }
    
    cv::Mat createColorMask(const cv::Mat& hsv, const std::string& color)
    {
        cv::Mat mask;
        
        if (color == "Rouge") {
            // Le rouge nécessite deux masques car il est à cheval sur 0° et 180°
            cv::Mat mask1, mask2;
            cv::inRange(hsv, color_ranges_["Rouge"].lower, color_ranges_["Rouge"].upper, mask1);
            cv::inRange(hsv, color_ranges_["Rouge2"].lower, color_ranges_["Rouge2"].upper, mask2);
            mask = mask1 | mask2;
        } else if (color_ranges_.find(color) != color_ranges_.end()) {
            cv::inRange(hsv, color_ranges_[color].lower, color_ranges_[color].upper, mask);
        } else {
            // Couleur par défaut (Bleu) si non trouvée
            cv::inRange(hsv, color_ranges_["Bleu"].lower, color_ranges_["Bleu"].upper, mask);
        }
        
        // Amélioration du masque avec des opérations morphologiques - OPTIMISÉ
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)); // Réduit de 5x5 à 3x3
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
        
        return mask;
    }
    
    cv::Scalar getColorBGR(const std::string& color)
    {
        // Couleurs BGR pour l'affichage sur l'image
        std::map<std::string, cv::Scalar> bgr_colors = {
            {"Rouge", cv::Scalar(0, 0, 255)},
            {"Vert", cv::Scalar(0, 255, 0)},
            {"Bleu", cv::Scalar(255, 0, 0)},
            {"Jaune", cv::Scalar(0, 255, 255)},
            {"Magenta", cv::Scalar(255, 0, 255)},
            {"Cyan", cv::Scalar(255, 255, 0)}
        };
        
        if (bgr_colors.find(color) != bgr_colors.end()) {
            return bgr_colors[color];
        }
        return cv::Scalar(255, 0, 0); // Bleu par défaut
    }

    void timer_callback()
    {
        cv::Mat frame;
        cap_ >> frame;
        if (frame.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Image vide capturée !");
            return;
        }

        // BGR → RGB pour ROS - OPTIMISÉ : Publier seulement si nécessaire
        cv::Mat frame_rgb;
        cv::cvtColor(frame, frame_rgb, cv::COLOR_BGR2RGB);
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", frame_rgb).toImageMsg();
        image_pub_->publish(*msg);

        // Traitement d'image pour le tracking - OPTIMISÉ
        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        
        // Créer le masque pour la couleur sélectionnée
        cv::Mat mask = createColorMask(hsv, current_color_);
        
        // Calcul des moments
        cv::Moments m = cv::moments(mask, true);
        
        std_msgs::msg::String xy_msg;
        std::string direction = "Aucune détection";
        
        if (m.m00 > 1000) { // Seuil minimum pour éviter le bruit
            int cx = static_cast<int>(m.m10 / m.m00);
            int cy = static_cast<int>(m.m01 / m.m00);
            
            // COMMUNICATION ROS2 UNIQUEMENT - Format attendu par la STM32
            xy_msg.data = std::to_string(cx) + "," + std::to_string(cy);
            direction = "Cible détectée";
            
            // Debug optionnel - seulement si activé
            #ifdef DEBUG_VISUAL
            cv::Scalar display_color = getColorBGR(current_color_);
            cv::circle(frame, cv::Point(cx, cy), 15, display_color, 3);
            cv::circle(frame, cv::Point(cx, cy), 5, cv::Scalar(255, 255, 255), -1);
            cv::putText(frame, "(" + std::to_string(cx) + "," + std::to_string(cy) + ")",
                       cv::Point(cx + 20, cy - 20), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                       display_color, 2);
            #endif
            
        } else {
            // Aucune détection
            xy_msg.data = "0,0";
            direction = "Pas de détection";
        }

        // PUBLIER UNIQUEMENT VIA ROS2 - PLUS D'UART
        xy_pub_->publish(xy_msg);

        // Log périodique optimisé
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "Tracking %s - %s", current_color_.c_str(), direction.c_str());

        // Affichage désactivé par défaut pour les performances
        // Décommenter si nécessaire pour le debug
        /*
        cv::putText(frame, "Couleur: " + current_color_, cv::Point(10, 30),
                   cv::FONT_HERSHEY_SIMPLEX, 0.8, getColorBGR(current_color_), 2);
        cv::putText(frame, direction, cv::Point(10, 60),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
        cv::imshow("Camera Tracking - " + current_color_, frame);
        cv::waitKey(1);
        */
    }

    // Variables membres
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr xy_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr color_sub_;
    cv::VideoCapture cap_;
    
    std::string current_color_;
    std::map<std::string, ColorRange> color_ranges_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "=== Démarrage Camera Tracking Node (ROS2 Only) ===");
    
    rclcpp::spin(std::make_shared<CameraTrackingNode>());
    rclcpp::shutdown();
    return 0;
}

