#include "main.h"
#include "motorCommand.h"
#include "quadEncoder.h"
#include "captDistIR.h"
#include "VL53L0X.h"
#include "groveLCD.h"

#define ARRAY_LEN 100
#define SAMPLING_PERIOD_ms 5
#define FIND_MOTOR_INIT_POS 0

//################################################
#define EX1 1
#define EX2 2
#define EX3 3

#define SYNCHRO_EX EX1
//################################################
// PARAMETRE A MODIFIER EN FONCTION DU N° ROBOT
#define ROS_DOMAIN_ID 11
//################################################

// Modes de fonctionnement
#define MODE_MANUEL 0
#define MODE_ALEATOIRE 1
#define MODE_SUIVI_CAMERA 2

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

extern I2C_HandleTypeDef hi2c1;

//###################ASSERVISSEMENT#################
#define Te_L 5
#define tau_L 40
#define Ti_L (0.1*tau_L)
#define Ki_L (Te_L/Ti_L)
#define Kp_L 0.01

#define Te_R 5
#define tau_R 40
#define Ti_R  (0.1*tau_R)
#define Ki_R  (Te_R/Ti_R)
#define Kp_R  0.01

int consigne_moteur_gauche = 0;
int consigne_moteur_droit = 0;

// Paramètres de détection d'obstacles
#define IR_THRESHOLD 2000
#define VL53_THRESHOLD 200

// Paramètres caméra
#define w_cam 260
#define h_cam 150

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

// Déclaration des objets synchronisants
xSemaphoreHandle xSemaphore = NULL;
xQueueHandle qh = NULL;
//###############task synchro################
xSemaphoreHandle xDataSemaphore = NULL;
struct SensorData {
    uint16_t vl53_distance;
    int ir_sensors[2];
    int motor_speeds[2];
    uint32_t timestamp;
} shared_sensor_data;

// Variables globales pour les publishers de diagnostic
rcl_publisher_t* g_publisherSensorData = NULL;
rcl_publisher_t* g_publisherAnalysis = NULL;
std_msgs__msg__String* g_sensor_data_msg = NULL;
std_msgs__msg__String* g_analysis_msg = NULL;
// ===================================================================

//###############task synchro################
struct AMessage
{
    char command;
    int data;
};

//~~~~~~~~~~~~~~~~~~~~~~~~~ État du robot ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
enum {
    AVANCER,
    TOURNER_GAUCHE,
    TOURNER_DROITE,
    RECULER,
    ARRET
} etat_robot = ARRET;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Variables globales
int obstacle_detected = 0;
// coté IHM
int obstacle_detected_avant =0;
int obstacle_detected_arriere=0;
/////////////
int currentMode = MODE_MANUEL;  // Mode par défaut
int lastMode = -1;

// Variables pour commandes ROS
char command_received = 's';
int speed_received = 0;

// Variables caméra
float values_xy[2] = {0.0, 0.0};

// Variable de consigne globale (contrôlable via ROS)
int consigne_globale = 500;  // Consigne par défaut

// Variable de vitesse mesurée pour publication
int vitesse_mesuree = 0;

uint16_t mes_vl53 = 0;

int Left_first_index_reached = 0;
int Right_first_index_reached = 0;

// Déclarations des fonctions
void microros_task(void *argument);

char process_command_data(const char* data) {
    if (data == NULL || strlen(data) == 0) {
        return '\0';
    }

    char first_char = data[0];

    // Check for valid movement commands
    if (first_char == 'f' || first_char == 'b' ||
        first_char == 'l' || first_char == 'r' ||
        first_char == 's') {
        return first_char;
    }

    // Check for velocity-only command (v + number)
    if (first_char == 'v') {
        return '\0';  // Indicate this is a velocity-only command
    }

    return '\0';  // Unknown command
}

/**
 * Parse speed/velocity value from ROS message string
 * Expected format: "f500" or "v500" etc.
 * Returns the numeric value or 0 if no valid number found
 */
int process_vitess_data(const char* data) {
    if (data == NULL || strlen(data) <= 1) {
        return 0;
    }

    // Skip the first character (command) and parse the number
    const char* number_start = data + 1;

    // Convert string to integer
    int value = 0;
    int i = 0;

    // Simple string to integer conversion
    while (number_start[i] >= '0' && number_start[i] <= '9') {
        value = value * 10 + (number_start[i] - '0');
        i++;
    }

    return value;
}

/**
 * Parse camera coordinates from ROS message string
 * Expected format: "x=130.5,y=75.2" or "130.5,75.2"
 * Fills the xy_values array with [x, y] coordinates
 */
void get_xy(const char* data, float* xy_values) {
    if (data == NULL || xy_values == NULL) {
        xy_values[0] = 0.0;
        xy_values[1] = 0.0;
        return;
    }

    // Initialize to zero
    xy_values[0] = 0.0;
    xy_values[1] = 0.0;

    // Try to parse format "x=value,y=value"
    if (strstr(data, "x=") != NULL && strstr(data, "y=") != NULL) {
        sscanf(data, "x=%f,y=%f", &xy_values[0], &xy_values[1]);
    }
    // Try to parse simple format "value,value"
    else if (strchr(data, ',') != NULL) {
        sscanf(data, "%f,%f", &xy_values[0], &xy_values[1]);
    }
    // Try to parse single value as x coordinate
    else {
        sscanf(data, "%f", &xy_values[0]);
        xy_values[1] = 0.0;
    }
}

//========================================================================

#if SYNCHRO_EX == EX1

int tab_speed_L[100];
int tab_speed_R[100];
int tab_cde[100];

// Tâche unique pour le contrôle du robot - basée sur la logique testée de main.c
static void robot_control_task(void *pvParameters)
{
    printf("=== ROBOT CONTROL TASK STARTED ===\n\r");

    int speedL = 0;
    int speedR = 0;
    int errL, errR;
    float upL, upR;
    static float uiL = 0.0;
    static float uiR = 0.0;
    static int i = 0;
    static int j = 0;
    int tab_ir[2];
    int runtime_counter = 0;
    int previous_state = ARRET;
    enum {AVANCER, TOURNER_GAUCHE, TOURNER_DROITE, RECULER, ARRET} current_etat = ARRET;
    int consigne_locale = 500;  // Consigne locale pour la tâche
    static int debug_counter = 0; // Pour limiter l'affichage debug

    for (;;)
    {
        debug_counter++;

        // Lecture des capteurs
        speedL = quadEncoder_GetSpeedL();
        speedR = quadEncoder_GetSpeedR();
        vitesse_mesuree = (speedL + speedR) / 2;  // Moyenne pour publication ROS

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
        mes_vl53 = VL53L0X_readRangeContinuousMillimeters();


        captDistIR_Get(tab_ir);

        // Détection d'obstacles - logique simple et testée
        obstacle_detected = (tab_ir[0] > IR_THRESHOLD || tab_ir[1] > IR_THRESHOLD || mes_vl53 < VL53_THRESHOLD);

        // Détection obstacle pour IHM
        obstacle_detected_avant = (tab_ir[0] > IR_THRESHOLD || tab_ir[1] > IR_THRESHOLD );
        obstacle_detected_arriere = mes_vl53 < VL53_THRESHOLD;

        //=================== LOGIQUE DE CONTRÔLE SELON LE MODE ===================
        switch (currentMode) {
            case MODE_MANUEL:
                // Mode Manuel - Exécuter les commandes reçues avec protection d'obstacle
                switch (command_received) {
                    case 'f':
                        current_etat = AVANCER;
                        if (speed_received > 0) {
                            consigne_moteur_gauche = speed_received;
                            consigne_moteur_droit = speed_received;
                        } else {
                            consigne_moteur_gauche = consigne_globale;
                            consigne_moteur_droit = consigne_globale;
                        }
                        break;
                    case 'b':
                        current_etat = RECULER;
                        break;
                    case 'l':
                        current_etat = TOURNER_GAUCHE;
                        break;
                    case 'r':
                        current_etat = TOURNER_DROITE;
                        break;
                    case 's':
                    default:
                        current_etat = ARRET;
                        break;
                }
                break;

            case MODE_ALEATOIRE:
                // Mode Aléatoire - Logique testée et fonctionnelle du main.c original
                runtime_counter++;
                if (runtime_counter >= 30000) {
                    current_etat = ARRET;
                } else {
                    if (tab_ir[0] > IR_THRESHOLD && tab_ir[1] > IR_THRESHOLD) {
                        current_etat = RECULER;
                    }
                    else if (tab_ir[0] > IR_THRESHOLD) {
                        current_etat = TOURNER_GAUCHE;
                    }
                    else if (tab_ir[1] > IR_THRESHOLD) {
                        current_etat = TOURNER_DROITE;
                    }
                    else if (mes_vl53 < VL53_THRESHOLD && current_etat == RECULER) {
                        current_etat = AVANCER;
                    }
                    else if (current_etat == TOURNER_GAUCHE || current_etat == TOURNER_DROITE) {
                        if (tab_ir[0] <= IR_THRESHOLD && tab_ir[1] <= IR_THRESHOLD) {
                            current_etat = AVANCER;
                        }
                    }
                    else if (current_etat != RECULER) {
                        current_etat = AVANCER;  // État par défaut
                    }
                }
                consigne_moteur_gauche = consigne_globale;
                consigne_moteur_droit = consigne_globale;
                break;

            case MODE_SUIVI_CAMERA:
                printf("Je suis dans le mode camera");
                // Mode Suivi Caméra
                if (obstacle_detected) {
                    current_etat = ARRET; // Arrêt si obstacle
                } else if (values_xy[0] == 0.0 && values_xy[1] == 0.0) {
                    current_etat = ARRET; // Pas de cible détectée
                } else if (values_xy[0] > (w_cam /2 - 40) && values_xy[0] < (w_cam /2 + 40)) {
                    current_etat = AVANCER; // Cible au centre, avancer
                    consigne_moteur_gauche = consigne_globale;
                    consigne_moteur_droit = consigne_globale;
                } else if (values_xy[0] > w_cam /2 +40) {
                    current_etat = TOURNER_DROITE; // Cible à droite
                } else {
                    current_etat = TOURNER_GAUCHE; // Cible à gauche
                }
                break;
        }

        // Debug périodique (toutes les 2 secondes environ)
        if (debug_counter % 400 == 0) {
            printf("*** ROBOT STATE: Mode=%d, Etat=%d, ConsigneG=%d, ConsigneD=%d, VitesseL=%d, VitesseR=%d ***\n\r",
                   currentMode, current_etat, consigne_moteur_gauche, consigne_moteur_droit, speedL, speedR);
        }

        // Reset de l'intégrateur si changement d'état
        if (current_etat != previous_state) {
            uiL = 0.0;
            uiR = 0.0;
            previous_state = current_etat;
            printf("*** CHANGEMENT ETAT: %d ***\n\r", current_etat);
        }

        // Commandes moteur selon l'état - LOGIQUE TESTÉE ET FONCTIONNELLE
        switch (current_etat) {
            case AVANCER:
                // Mode manuel : vérifier obstacle pendant l'avancée
                if (currentMode == MODE_MANUEL && obstacle_detected) {
                    // Obstacle détecté en mode manuel, arrêter les moteurs
                    uiL = 0.0;
                    uiR = 0.0;
                    motorLeft_SetDuty(100);
                    motorRight_SetDuty(100);
                } else {
                    // Pas d'obstacle ou mode automatique, avancer normalement avec asservissement séparé
                    errL = consigne_moteur_gauche - speedL;
                    errR = consigne_moteur_droit - speedR;

                    upL = Kp_L * errL;
                    upR = Kp_R * errR;

                    uiL = uiL + Kp_L * Ki_L * errL;
                    uiR = uiR + Kp_R * Ki_R * errR;

                    motorLeft_SetDuty(upL + uiL + 100);
                    motorRight_SetDuty(upR + uiR + 100);

                    if (i<100){
                        tab_speed_L[i] = speedL;
                        i++;
                    }
                    if (j<100){
                        tab_speed_R[j] = speedR;
                        j++;
                    }
                }
                break;

            case TOURNER_GAUCHE:
                if (currentMode == MODE_MANUEL && obstacle_detected) {
                    // Obstacle détecté en mode manuel, arrêter les moteurs
                    motorLeft_SetDuty(100);
                    motorRight_SetDuty(100);
                } else {
                    // Tourner à gauche : moteur droit plus rapide
                    motorLeft_SetDuty(100);
                    motorRight_SetDuty(130);
                }
                break;

            case TOURNER_DROITE:
                if (currentMode == MODE_MANUEL && obstacle_detected) {
                    // Obstacle détecté en mode manuel, arrêter les moteurs
                    motorLeft_SetDuty(100);
                    motorRight_SetDuty(100);
                } else {
                    // Tourner à droite : moteur gauche plus rapide
                    motorLeft_SetDuty(130);
                    motorRight_SetDuty(100);
                }
                break;

            case RECULER:
                if (currentMode == MODE_MANUEL && obstacle_detected || currentMode == MODE_ALEATOIRE && mes_vl53<VL53_THRESHOLD) {
                    // Obstacle détecté en mode manuel, arrêter les moteurs
                    motorLeft_SetDuty(100);
                    motorRight_SetDuty(100);
                } else {
                    // Reculer
                    motorLeft_SetDuty(80);
                    motorRight_SetDuty(80);
                }
                break;

            case ARRET:
            default:
                // Arrêt complet
                uiL = 0.0;
                uiR = 0.0;
                motorLeft_SetDuty(100);
                motorRight_SetDuty(100);
                break;
        }

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
        vTaskDelay(SAMPLING_PERIOD_ms);
    }
}

static void displayTask(void *pvParameters)
{

    // Initialisation LCD
    groveLCD_begin(16,2,0);
    HAL_Delay(100);
    groveLCD_display();
    groveLCD_setCursor(0,0);
    groveLCD_term_printf("Robot Ready");

    for(;;) {
        if (currentMode != lastMode) {
            groveLCD_display();
            groveLCD_setCursor(0,0);
            groveLCD_term_printf("                "); // Clear line
            groveLCD_setCursor(0,0);

            switch (currentMode) {
                case MODE_MANUEL:
                    groveLCD_term_printf("Mode Manuel");
                    break;
                case MODE_ALEATOIRE:
                    groveLCD_term_printf("Mode Aleatoire");
                    break;
                case MODE_SUIVI_CAMERA:
                    groveLCD_term_printf("Mode Suivi");
                    break;
                default:
                    groveLCD_term_printf("Mode: ERROR");
                    break;
            }
            lastMode = currentMode;
        }
        vTaskDelay(500);
    }
}

//###############task synchro################
//COLLECTEUR DE DONNÉES CAPTEURS (Producer) - MODIFIÉ POUR IHM
static void sensor_data_collector_task(void *pvParameters)
{
    printf("=== SENSOR DATA COLLECTOR TASK STARTED SUCCESSFULLY ===\n\r");

    // Attendre un peu pour que micro-ROS soit prêt
    vTaskDelay(2000);
    printf("=== SENSOR COLLECTOR: Début de la boucle principale ===\n\r");

    for (;;)
    {
        printf("SENSOR COLLECTOR: Lecture capteurs...\n\r");

        // Collecte des données des capteurs
        int tab_ir[2];
        captDistIR_Get(tab_ir);
        uint16_t current_vl53;

        if(xSemaphoreTake(xDataSemaphore, portMAX_DELAY)==pdTRUE)
        {
        	current_vl53 = mes_vl53;
        	xSemaphoreGive(xDataSemaphore);

        }

        // Préparation des données à partager
        struct SensorData temp_data;
        temp_data.vl53_distance = current_vl53;
        temp_data.ir_sensors[0] = tab_ir[0];
        temp_data.ir_sensors[1] = tab_ir[1];
        temp_data.motor_speeds[0] = quadEncoder_GetSpeedL();
        temp_data.motor_speeds[1] = quadEncoder_GetSpeedR();
        temp_data.timestamp = HAL_GetTick();

        printf("COLLECTOR: VL53=%d, IR=[%d,%d], Motors=[%d,%d]\n\r",
               temp_data.vl53_distance,
               temp_data.ir_sensors[0], temp_data.ir_sensors[1],
               temp_data.motor_speeds[0], temp_data.motor_speeds[1]);

        if (xSemaphoreTake(xDataSemaphore, portMAX_DELAY) == pdTRUE)
        {
            shared_sensor_data = temp_data;

            // Publier vers IHM seulement si micro-ROS est prêt
            if (g_publisherSensorData != NULL && g_sensor_data_msg != NULL) {
                sprintf(g_sensor_data_msg->data.data,
                        "vl53=%d,ir_left=%d,ir_right=%d,motor_left=%d,motor_right=%d,timestamp=%lu",
                        temp_data.vl53_distance,
                        temp_data.ir_sensors[0],
                        temp_data.ir_sensors[1],
                        temp_data.motor_speeds[0],
                        temp_data.motor_speeds[1],
                        temp_data.timestamp);
                g_sensor_data_msg->data.size = strlen(g_sensor_data_msg->data.data);
                rcl_publish(g_publisherSensorData, g_sensor_data_msg, NULL);
                printf("COLLECTOR: Message publié vers ROS\n\r");
            } else {
                printf("COLLECTOR: Publishers pas encore prêts\n\r");
            }

            xSemaphoreGive(xDataSemaphore);
        } else {
            printf("COLLECTOR: ERREUR - Impossible de prendre le sémaphore\n\r");
        }

        vTaskDelay(50);
    }
}

//ANALYSEUR DE DONNÉES (Consumer) - MODIFIÉ POUR IHM
static void data_analyzer_task(void *pvParameters)
{
    printf("=== DATA ANALYZER TASK STARTED SUCCESSFULLY ===\n\r");

    // Attendre un peu pour que micro-ROS soit prêt
    vTaskDelay(3000);
    printf("=== ANALYZER: Début de la boucle principale ===\n\r");

    struct SensorData local_data;
    uint32_t analysis_counter = 0;

    for (;;)
    {
        printf("ANALYZER: Cycle %lu\n\r", analysis_counter);

        if (xSemaphoreTake(xDataSemaphore, portMAX_DELAY) == pdTRUE)
        {
            local_data = shared_sensor_data;
            xSemaphoreGive(xDataSemaphore);
            printf("ANALYZER: Données récupérées avec succès\n\r");
        } else {
            printf("ANALYZER: ERREUR - Impossible de prendre le sémaphore\n\r");
        }

        analysis_counter++;

        if (analysis_counter % 20 == 0) // toutes les 2s
        {
            printf("=== ANALYSIS #%lu ===\n\r", analysis_counter/20);

            // Analyse de la distance
            if (local_data.vl53_distance < 100) {
                printf("ANALYZER: OBSTACLE TRÈS PROCHE! Distance=%d mm\n\r", local_data.vl53_distance);

                if (g_publisherAnalysis != NULL && g_analysis_msg != NULL) {
                    sprintf(g_analysis_msg->data.data, "OBSTACLE TRÈS PROCHE! Distance=%d mm", local_data.vl53_distance);
                    g_analysis_msg->data.size = strlen(g_analysis_msg->data.data);
                    rcl_publish(g_publisherAnalysis, g_analysis_msg, NULL);
                }

            } else if (local_data.vl53_distance < 200) {
                printf("ANALYZER: Obstacle détecté à %d mm\n\r", local_data.vl53_distance);

                if (g_publisherAnalysis != NULL && g_analysis_msg != NULL) {
                    sprintf(g_analysis_msg->data.data, "Obstacle détecté à %d mm", local_data.vl53_distance);
                    g_analysis_msg->data.size = strlen(g_analysis_msg->data.data);
                    rcl_publish(g_publisherAnalysis, g_analysis_msg, NULL);
                }
            }

            // Analyse des capteurs IR
            int ir_max = (local_data.ir_sensors[0] > local_data.ir_sensors[1]) ?
                         local_data.ir_sensors[0] : local_data.ir_sensors[1];
            if (ir_max > IR_THRESHOLD) {
                printf("ANALYZER: Capteur IR actif - Valeur max: %d\n\r", ir_max);

                if (g_publisherAnalysis != NULL && g_analysis_msg != NULL) {
                    sprintf(g_analysis_msg->data.data, "Capteur IR actif - Valeur max: %d", ir_max);
                    g_analysis_msg->data.size = strlen(g_analysis_msg->data.data);
                    rcl_publish(g_publisherAnalysis, g_analysis_msg, NULL);
                }
            }

            // Analyse des vitesses moteurs
            int speed_diff = abs(local_data.motor_speeds[0] - local_data.motor_speeds[1]);
            if (speed_diff > 50) {
                printf("ANALYZER: Différence vitesse moteurs importante: %d\n\r", speed_diff);

                if (g_publisherAnalysis != NULL && g_analysis_msg != NULL) {
                    sprintf(g_analysis_msg->data.data, "Différence vitesse moteurs importante: %d", speed_diff);
                    g_analysis_msg->data.size = strlen(g_analysis_msg->data.data);
                    rcl_publish(g_publisherAnalysis, g_analysis_msg, NULL);
                }
            }

            // Analyse temporelle
            uint32_t data_age = HAL_GetTick() - local_data.timestamp;
            if (data_age > 1000) {
                printf("ANALYZER: WARNING - Données anciennes (%lu ms)\n\r", data_age);

                if (g_publisherAnalysis != NULL && g_analysis_msg != NULL) {
                    sprintf(g_analysis_msg->data.data, "WARNING - Données anciennes (%lu ms)", data_age);
                    g_analysis_msg->data.size = strlen(g_analysis_msg->data.data);
                    rcl_publish(g_publisherAnalysis, g_analysis_msg, NULL);
                }
            }
        }

        vTaskDelay(100);
    }
}

//###############task synchro################

//========================================================
#elif SYNCHRO_EX == EX2

static void task_C( void *pvParameters )
{
    for (;;)
    {
        printf("TASK C \n\r");
        xSemaphoreTake( xSemaphore, portMAX_DELAY );
    }
}

static void task_D( void *pvParameters )
{
    for (;;)
    {
        printf("TASK D \n\r");
        xSemaphoreGive( xSemaphore );
    }
}

//========================================================
#elif SYNCHRO_EX == EX3

static void task_E( void *pvParameters )
{
    struct AMessage pxMessage;
    pxMessage.command='a';
    pxMessage.data=10;
    vTaskDelay(1000);

    for (;;)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
        printf("TASK E \r\n");
        xQueueSend( qh, ( void * ) &pxMessage, portMAX_DELAY );
        xSemaphoreTake( xSemaphore, portMAX_DELAY );

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
        vTaskDelay(SAMPLING_PERIOD_ms);
    }
}

static void task_F(void *pvParameters)
{
    struct AMessage pxRxedMessage;

    for (;;)
    {
        xQueueReceive( qh, &( pxRxedMessage ), portMAX_DELAY );
        printf("TASK F \r\n");
        xSemaphoreGive( xSemaphore );
    }
}
#endif

//=========================================================================
int main(void)
{
    int ret=0;

    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();

    motorCommand_Init();
    quadEncoder_Init();
    captDistIR_Init();

    HAL_Delay(500);

    // Affichage via UART2 sur Terminal série
    printf("=== Robot STM32 avec micro-ROS ===\n\r");

    VL53L0X_init();

    ret = VL53L0X_validateInterface();
    if(ret == 0) {
        printf("VL53L0X OK\n\r");
    } else {
        printf("!! PROBLEME VL53L0X !!\n\r");
    }
    VL53L0X_startContinuous(0);

    HAL_Delay(50);

#if FIND_MOTOR_INIT_POS
    int16_t speed=0;
    // RECHERCHE DE LA POSITION INITIALE
    printf("Initialisation encodeurs...\n\r");
    Left_first_index_reached = 0;
    while( Left_first_index_reached != 1 ) {
        motorLeft_SetDuty(130);
        HAL_Delay(SAMPLING_PERIOD_ms);
        speed = quadEncoder_GetSpeedL();
    }

    Right_first_index_reached = 0;
    while( Right_first_index_reached != 1 ) {
        motorRight_SetDuty(130);
        HAL_Delay(SAMPLING_PERIOD_ms);
        speed = quadEncoder_GetSpeedR();
    }

    motorLeft_SetDuty(100);
    motorRight_SetDuty(100);
    HAL_Delay(200);
    printf("Encodeurs initialisés\n\r");
#endif

    // ============ INITIALISATION FREERTOS ============
    osKernelInitialize();
    printf("Kernel FreeRTOS initialisé\n\r");

    // ============ CRÉATION DES OBJETS DE SYNCHRONISATION D'ABORD ============
    printf("Création des objets de synchronisation...\n\r");

    // Sémaphore pour les autres exercices
    vSemaphoreCreateBinary(xSemaphore);
    if (xSemaphore != NULL) {
        printf("xSemaphore créé avec succès\n\r");
        xSemaphoreTake( xSemaphore, portMAX_DELAY );
    } else {
        printf("ERREUR: Impossible de créer xSemaphore\n\r");
    }

    // Queue pour les autres exercices
    qh = xQueueCreate( 1, sizeof(struct AMessage ) );
    if (qh != NULL) {
        printf("Queue créée avec succès\n\r");
    } else {
        printf("ERREUR: Impossible de créer la queue\n\r");
    }

#if SYNCHRO_EX == EX1
    // Sémaphore pour la synchronisation des données capteurs
    vSemaphoreCreateBinary(xDataSemaphore);
    if (xDataSemaphore != NULL) {
        printf("xDataSemaphore créé avec succès\n\r");
        xSemaphoreGive(xDataSemaphore);
    } else {
        printf("ERREUR: Impossible de créer xDataSemaphore\n\r");
        while(1); // Arrêt en cas d'erreur critique
    }
#endif

    // ============ VÉRIFICATION MÉMOIRE ============
    size_t free_heap = xPortGetFreeHeapSize();
    printf("Mémoire heap libre avant création des tâches: %d bytes\n\r", (int)free_heap);

    if (free_heap < 2000) {
        printf("ATTENTION: Mémoire heap faible! Risque d'échec de création des tâches\n\r");
    }

    // ============ CRÉATION DES TÂCHES ============
    printf("Création des tâches...\n\r");

#if SYNCHRO_EX == EX1
    // Tâche micro-ROS
    BaseType_t result = xTaskCreate( microros_task, "microros_task", 2000, NULL, 24, NULL );
    if (result == pdPASS) {
        printf("✓ microros_task créée\n\r");
    } else {
        printf("✗ ERREUR: Impossible de créer microros_task\n\r");
    }

    // Tâche contrôle robot
    result = xTaskCreate( robot_control_task, "robot_control", 384, NULL, 28, NULL );
    if (result == pdPASS) {
        printf("✓ robot_control_task créée\n\r");
    } else {
        printf("✗ ERREUR: Impossible de créer robot_control_task\n\r");
    }

    // Tâche affichage
    result = xTaskCreate( displayTask, "display_task", 128, NULL, 24, NULL );
    if (result == pdPASS) {
        printf("✓ display_task créée\n\r");
    } else {
        printf("✗ ERREUR: Impossible de créer display_task\n\r");
    }

    // Tâche collecteur de données capteurs
    result = xTaskCreate( sensor_data_collector_task, "sensor_collector", 384, NULL, 27, NULL );
    if (result == pdPASS) {
        printf("✓ sensor_data_collector_task créée\n\r");
    } else {
        printf("✗ ERREUR: Impossible de créer sensor_data_collector_task\n\r");
    }

    // Tâche analyseur de données
    result = xTaskCreate( data_analyzer_task, "data_analyzer", 384, NULL, 26, NULL );
    if (result == pdPASS) {
        printf("✓ data_analyzer_task créée\n\r");
    } else {
        printf("✗ ERREUR: Impossible de créer data_analyzer_task\n\r");
    }

#elif SYNCHRO_EX == EX2
    xTaskCreate( task_C, "task C", 128, NULL, 28, NULL );
    xTaskCreate( task_D, "task D", 128, NULL, 27, NULL );

#elif SYNCHRO_EX == EX3
    xTaskCreate( task_E, "task E", 128, NULL, 30, NULL );
    xTaskCreate( task_F, "task F", 128, NULL, 29, NULL );
#endif

    // ============ VÉRIFICATION FINALE ============
    free_heap = xPortGetFreeHeapSize();
    printf("Mémoire heap libre après création des tâches: %d bytes\n\r", (int)free_heap);

    printf("=== DÉMARRAGE DU SYSTÈME ===\n\r");
    osKernelStart();

    // Cette ligne ne devrait jamais être atteinte
    printf("ERREUR: osKernelStart() a retourné!\n\r");
    while (1) {}
}

//=========================================================================
// Fonctions de transport micro-ROS (déclarées dans vos fichiers existants)
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

// Callbacks ROS
void subscription_handle_mode(const void * msgin)
{
    const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
    printf("*** MODE RECU: '%s' ***\n\r", msg->data.data);

    // Changement de mode direct
    if(msg->data.data[0] == '0') {
        currentMode = MODE_MANUEL;
        command_received = 's'; // Reset commande
        speed_received = 0;
    } else if(msg->data.data[0] == '1') {
        currentMode = MODE_ALEATOIRE;
    } else if(msg->data.data[0] == '2') {
        currentMode = MODE_SUIVI_CAMERA;
    }

    printf("*** Mode changé vers: %d ***\n\r", currentMode);
}

void subscription_handle_command(const void * msgin)
{
    const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
    printf("*** COMMANDE RECUE: '%s' ***\n\r", msg->data.data);

    // Parse la commande
    char cmd = process_command_data(msg->data.data);
    int spd = process_vitess_data(msg->data.data);

    printf("*** DEBUG: cmd='%c' (ASCII=%d), spd=%d ***\n\r", cmd, (int)cmd, spd);

    // Si c'est une commande de vitesse pure (v + nombre), mettre à jour la consigne globale
    if (cmd == '\0' && spd > 0) {
        consigne_globale = spd;
        printf("*** CONSIGNE GLOBALE MISE A JOUR: %d ***\n\r", consigne_globale);
    }
    // Sinon, traiter les commandes de mouvement seulement en mode manuel
    else if (currentMode == MODE_MANUEL && cmd != '\0') {
        command_received = cmd;
        speed_received = spd;
        printf("*** MODE MANUEL - Commande: '%c', Vitesse: %d ***\n\r", command_received, speed_received);
    }
    else {
        printf("*** COMMANDE IGNOREE - Mode: %d, cmd='%c', spd=%d ***\n\r", currentMode, cmd, spd);
    }
}

void subscription_camera_callback(const void * msgin)
{
    const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;

    // Parse les coordonnées caméra (utilise la fonction existante dans motorCommand.c)
    get_xy(msg->data.data, values_xy);
    printf("*** CAM XY: [%.1f, %.1f] ***\n\r", values_xy[0], values_xy[1]);
}

void microros_task(void *argument)
{
    printf("=== DEMARRAGE MICRO-ROS ===\n\r");

    rmw_uros_set_custom_transport( true, (void *) &huart1, cubemx_transport_open, cubemx_transport_close, cubemx_transport_write, cubemx_transport_read);
    printf("1. Transport configuré\n\r");

    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate = microros_allocate;
    freeRTOS_allocator.deallocate = microros_deallocate;
    freeRTOS_allocator.reallocate = microros_reallocate;
    freeRTOS_allocator.zero_allocate = microros_zero_allocate;

    if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
        printf("ERREUR: Allocation mémoire\n\r");
        return;
    }
    printf("2. Allocateur configuré\n\r");

    // micro-ROS app
    rclc_support_t support;
    rcl_allocator_t allocator;
    allocator = rcl_get_default_allocator();

    // create node
    rcl_node_t node;
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID);

    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
    rclc_node_init_default(&node, "STM32_Robot_Node","", &support);
    printf("3. Node STM32_Robot_Node créé\n\r");

    // create publishers
    rcl_publisher_t publisherSpeed;
    std_msgs__msg__String sensor_speed_msg;
    rclc_publisher_init_default(&publisherSpeed, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),"/sensor/motor_speed");

    rcl_publisher_t publisherObstacle;
    std_msgs__msg__String sensor_obs_msg;
    rclc_publisher_init_default(&publisherObstacle, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),"/sensor/receive_obstacle");

    // ============= NOUVEAUX PUBLISHERS POUR LE DIAGNOSTIC =============
    rcl_publisher_t publisherSensorData;
    std_msgs__msg__String sensor_data_msg;
    rclc_publisher_init_default(&publisherSensorData, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),"/diagnostic/sensor_data");

    rcl_publisher_t publisherAnalysis;
    std_msgs__msg__String analysis_msg;
    rclc_publisher_init_default(&publisherAnalysis, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),"/diagnostic/analysis");

    // Assigner les pointeurs globaux pour que les autres tâches puissent les utiliser
    g_publisherSensorData = &publisherSensorData;
    g_publisherAnalysis = &publisherAnalysis;
    g_sensor_data_msg = &sensor_data_msg;
    g_analysis_msg = &analysis_msg;
    // ===================================================================

    printf("4. Publishers créés (avec diagnostic)\n\r");

    // create subscribers
    rcl_subscription_t subscriberMode;
    std_msgs__msg__String str_msg1;
    rclc_subscription_init_default(&subscriberMode,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),"/command/mode");

    rcl_subscription_t subscriberMove;
    std_msgs__msg__String str_msg2;
    rclc_subscription_init_default(&subscriberMove,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),"/command/move");

    rcl_subscription_t subscriberCamera;
    std_msgs__msg__String str_msg3;
    rclc_subscription_init_default(&subscriberCamera,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),"/camera/src_xy");
    printf("5. Subscribers créés\n\r");

    // Add subscription to the executor
    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 3, &allocator);
    rclc_executor_add_subscription(&executor, &subscriberMode, &str_msg1, &subscription_handle_mode, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &subscriberMove, &str_msg2, &subscription_handle_command, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &subscriberCamera, &str_msg3, &subscription_camera_callback, ON_NEW_DATA);
    printf("6. Executor configuré\n\r");

    // Allocation mémoire messages
    sensor_speed_msg.data.data = (char * ) malloc(ARRAY_LEN * sizeof(char));
    sensor_speed_msg.data.size = 0;
    sensor_speed_msg.data.capacity = ARRAY_LEN;

    sensor_obs_msg.data.data = (char * ) malloc(ARRAY_LEN * sizeof(char));
    sensor_obs_msg.data.size = 0;
    sensor_obs_msg.data.capacity = ARRAY_LEN;

    str_msg1.data.data = (char * ) malloc(ARRAY_LEN * sizeof(char));
    str_msg1.data.size = 0;
    str_msg1.data.capacity = ARRAY_LEN;

    str_msg2.data.data = (char * ) malloc(ARRAY_LEN * sizeof(char));
    str_msg2.data.size = 0;
    str_msg2.data.capacity = ARRAY_LEN;

    str_msg3.data.data = (char * ) malloc(ARRAY_LEN * sizeof(char));
    str_msg3.data.size = 0;
    str_msg3.data.capacity = ARRAY_LEN;

    // ============= ALLOCATION MÉMOIRE POUR LES NOUVEAUX MESSAGES =============
    sensor_data_msg.data.data = (char * ) malloc(ARRAY_LEN * sizeof(char));
    sensor_data_msg.data.size = 0;
    sensor_data_msg.data.capacity = ARRAY_LEN;

    analysis_msg.data.data = (char * ) malloc(ARRAY_LEN * sizeof(char));
    analysis_msg.data.size = 0;
    analysis_msg.data.capacity = ARRAY_LEN;
    // ==========================================================================

    printf("=== MICRO-ROS PRET - Boucle principale (avec diagnostic) ===\n\r");

    for(;;)
    {
        // Publication des données du robot - NOUVELLE VERSION AVEC UNE SEULE VITESSE
        sprintf(sensor_speed_msg.data.data, "speed=#%d,consigne=#%d", (int32_t)vitesse_mesuree, (int32_t)consigne_globale);
        sensor_speed_msg.data.size = strlen(sensor_speed_msg.data.data);

        sprintf(sensor_obs_msg.data.data, "front_obstacle=#%d,rear_obstacle=#%d",
                (int32_t)obstacle_detected_avant, (int32_t)obstacle_detected_arriere);
        sensor_obs_msg.data.size = strlen(sensor_obs_msg.data.data);

        // Publish sensor data
        rcl_publish(&publisherSpeed, &sensor_speed_msg, NULL);
        rcl_publish(&publisherObstacle, &sensor_obs_msg, NULL);

        // Traitement des messages entrants
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        osDelay(10);
    }
}

//=========================================================================
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4) {
        HAL_IncTick();
    }
}

//=========================================================================
void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

//=========================================================================
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
