// ARMADA : commande du robot par micro-ROS (alternative à la page web de armada_esp.ino).
//
// L'ESP32 se connecte à votre Wi-Fi puis au micro-ROS agent qui tourne sur le PC (armada_ros, real.launch.py).
// Il expose la MÊME interface que la simulation (armada_simulation), dans le namespace ROBOT_NAME :
//
//   /<ROBOT_NAME>/cmd_vel     geometry_msgs/Twist     entrée : linear.x (m/s), angular.z (rad/s)
//   /<ROBOT_NAME>/odom        nav_msgs/Odometry       sortie : odométrie des servos (10 Hz)
//   /<ROBOT_NAME>/scan        sensor_msgs/LaserScan   sortie : anneau de 8 ToF, 1 rayon / 45° à partir de l'avant (5 Hz)
//   /<ROBOT_NAME>/floor/range sensor_msgs/Range       sortie : HC-SR04 de sol (10 Hz)
//
// Frames : <ROBOT_NAME>/odom -> <ROBOT_NAME>/base_footprint (TF fabriquée par le PC), <ROBOT_NAME>/lidar_link, ...
// Sécurité : sans cmd_vel pendant COMMAND_TIMEOUT_MS, ou si l'agent disparaît, le robot s'arrête.
//
// Prérequis : bibliothèque micro_ros_arduino de la MÊME distribution ROS 2 que le PC (Jazzy), SCServo, Adafruit VL53L0X.
// Le code du robot est partagé avec armada_esp.ino (dossier voisin ../armada_esp), il n'est pas dupliqué.

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <sensor_msgs/msg/range.h>

// Le code du robot vit dans ../armada_esp : l'IDE Arduino ne compile que le dossier du croquis, on inclut donc aussi les .cpp.
#include "../armada_esp/Motor.cpp"
#include "../armada_esp/tof.cpp"
#include "../armada_esp/ultrasonic.cpp"
#include "../armada_esp/Robot.cpp"

// ─── À RÉGLER ───────────────────────────────────────────────────────────────
#define ROBOT_NAME "armada1"            // namespace ROS, UNIQUE par robot ; à donner à real.launch.py (robots:=...)
#define WIFI_SSID "VOTRE_WIFI"          // ne pas commiter vos vrais identifiants
#define WIFI_PASSWORD "VOTRE_MOT_DE_PASSE"
#define AGENT_IP "192.168.1.10"         // IP du PC qui exécute le micro-ROS agent
#define AGENT_PORT 8888
#define ROS_DOMAIN_ID 0                 // = ROS_DOMAIN_ID du PC

// ─── Robot : mêmes valeurs que armada_esp.ino (à garder synchronisées) ──────
#define WHEEL_DIAMETER 0.0772f // m
#define ROBOT_WIDTH 0.0925f    // m
#define MAX_RPM 60.0f
#define LEFT_MOTOR_ID 2
#define RIGHT_MOTOR_ID 1
#define RX_MOTORS 16
#define TX_MOTORS 17
#define MOTORS_BAUDRATE 1000000
#define LEFT_INVERTED true
#define TOF_SDA 21
#define TOF_SCL 22
#define TOF_MUX_ADDRESS 0x70
#define FLOOR_TRIG_PIN 27
#define FLOOR_ECHO_PIN 34
#define COMMAND_TIMEOUT_MS 400

// ─── Capteurs publiés ───────────────────────────────────────────────────────
#define ODOM_PERIOD_MS 100
#define SCAN_PERIOD_MS 200
#define FLOOR_PERIOD_MS 100
#define TOF_RING_RADIUS 0.0356f // m : distance centre de l'anneau (lidar_link) -> capteurs ToF (URDF)
#define SCAN_RANGE_MIN 0.04f    // m
#define SCAN_RANGE_MAX 1.2f     // m
#define FLOOR_RANGE_MIN 0.02f   // m
#define FLOOR_RANGE_MAX 0.4f    // m
#define FLOOR_FOV 0.26f         // rad

Robot robot(WHEEL_DIAMETER, ROBOT_WIDTH, MAX_RPM, LEFT_MOTOR_ID, RIGHT_MOTOR_ID, RX_MOTORS, TX_MOTORS,
            MOTORS_BAUDRATE, LEFT_INVERTED, TOF_SDA, TOF_SCL, TOF_MUX_ADDRESS, FLOOR_TRIG_PIN, FLOOR_ECHO_PIN);

// ─── micro-ROS ──────────────────────────────────────────────────────────────
#define EXECUTE_EVERY_N_MS(MS, X)          \
    do                                     \
    {                                      \
        static volatile int64_t init = -1; \
        if (init == -1)                    \
        {                                  \
            init = uxr_millis();           \
        }                                  \
        if (uxr_millis() - init > MS)      \
        {                                  \
            X;                             \
            init = uxr_millis();           \
        }                                  \
    } while (0)

enum states
{
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

rclc_support_t support;
rcl_init_options_t init_options;
rcl_node_t node;
rclc_executor_t executor;
rcl_allocator_t allocator;

rcl_subscription_t cmd_sub;
rcl_publisher_t odom_pub;
rcl_publisher_t scan_pub;
rcl_publisher_t floor_pub;

geometry_msgs__msg__Twist cmd_msg;
nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__LaserScan scan_msg;
sensor_msgs__msg__Range floor_msg;

// Mémoire des messages de taille variable (chaînes, tableaux) : statique, pas d'allocation dynamique
char frame_odom[40], frame_base[40], frame_lidar[40], frame_floor[40];
float scan_ranges[MAX_TOF_SENSORS];

// Odométrie précédente, pour en déduire les vitesses
float prev_x = 0, prev_y = 0, prev_theta = 0;
bool odom_has_prev = false;
uint32_t prev_odom_ms = 0;
uint32_t last_odom_ms = 0, last_scan_ms = 0, last_floor_ms = 0; // dernières publications

static void set_string(rosidl_runtime_c__String &s, char *buffer, size_t capacity, const char *text)
{
    snprintf(buffer, capacity, "%s", text);
    s.data = buffer;
    s.size = strlen(buffer);
    s.capacity = capacity;
}

static void stamp_now(builtin_interfaces__msg__Time &t)
{
    int64_t ns = rmw_uros_epoch_nanos(); // heure du PC si la session est synchronisée
    t.sec = (int32_t)(ns / 1000000000LL);
    t.nanosec = (uint32_t)(ns % 1000000000LL);
}

static void init_messages()
{
    char name[40];

    snprintf(name, sizeof(name), "%s/odom", ROBOT_NAME);
    set_string(odom_msg.header.frame_id, frame_odom, sizeof(frame_odom), name);
    snprintf(name, sizeof(name), "%s/base_footprint", ROBOT_NAME);
    set_string(odom_msg.child_frame_id, frame_base, sizeof(frame_base), name);
    for (int i = 0; i < 36; ++i)
    {
        odom_msg.pose.covariance[i] = 0.0;
        odom_msg.twist.covariance[i] = 0.0;
    }
    odom_msg.pose.covariance[0] = odom_msg.pose.covariance[7] = 0.01;
    odom_msg.pose.covariance[14] = odom_msg.pose.covariance[21] = odom_msg.pose.covariance[28] = 1e3; // z, roll, pitch : non mesurés
    odom_msg.pose.covariance[35] = 0.05;
    odom_msg.twist.covariance[0] = 0.01;
    odom_msg.twist.covariance[14] = odom_msg.twist.covariance[21] = odom_msg.twist.covariance[28] = 1e3;
    odom_msg.twist.covariance[7] = 1e3;
    odom_msg.twist.covariance[35] = 0.05;

    // Anneau de 8 ToF : 8 rayons de 0 à 315° (F FL L BL B BR R FR), repère lidar_link
    snprintf(name, sizeof(name), "%s/lidar_link", ROBOT_NAME);
    set_string(scan_msg.header.frame_id, frame_lidar, sizeof(frame_lidar), name);
    scan_msg.angle_min = 0.0f;
    scan_msg.angle_increment = M_PI / 4.0f;
    scan_msg.angle_max = 7.0f * M_PI / 4.0f;
    scan_msg.range_min = SCAN_RANGE_MIN;
    scan_msg.range_max = SCAN_RANGE_MAX;
    scan_msg.time_increment = 0.0f;
    scan_msg.scan_time = SCAN_PERIOD_MS / 1000.0f;
    scan_msg.ranges.data = scan_ranges;
    scan_msg.ranges.size = MAX_TOF_SENSORS;
    scan_msg.ranges.capacity = MAX_TOF_SENSORS;
    scan_msg.intensities.data = NULL;
    scan_msg.intensities.size = 0;
    scan_msg.intensities.capacity = 0;

    snprintf(name, sizeof(name), "%s/ultrasonic_link", ROBOT_NAME);
    set_string(floor_msg.header.frame_id, frame_floor, sizeof(frame_floor), name);
    floor_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
    floor_msg.field_of_view = FLOOR_FOV;
    floor_msg.min_range = FLOOR_RANGE_MIN;
    floor_msg.max_range = FLOOR_RANGE_MAX;
}

// ─── Commande ───────────────────────────────────────────────────────────────

void cmd_callback(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    // Robot::set_velocity bride chaque roue à MAX_RPM et relance le délai de sécurité
    robot.set_velocity((float)msg->linear.x, (float)msg->angular.z);
}

// ─── Publications ───────────────────────────────────────────────────────────

void publish_odom()
{
    if (!robot.update_odometry())
    {
        return; // servo muet : on garde la dernière pose
    }
    float x, y, theta;
    robot.get_odometry(x, y, theta);

    float vx = 0.0f, vw = 0.0f;
    uint32_t now = millis();
    if (odom_has_prev && now > prev_odom_ms)
    {
        float dt = (now - prev_odom_ms) / 1000.0f;
        float mid = prev_theta + 0.5f * (theta - prev_theta);
        vx = ((x - prev_x) * cosf(mid) + (y - prev_y) * sinf(mid)) / dt;
        vw = (theta - prev_theta) / dt;
    }
    prev_x = x;
    prev_y = y;
    prev_theta = theta;
    odom_has_prev = true;
    prev_odom_ms = now;

    stamp_now(odom_msg.header.stamp);
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = sinf(theta / 2.0f);
    odom_msg.pose.pose.orientation.w = cosf(theta / 2.0f);
    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = vw;
    rcl_publish(&odom_pub, &odom_msg, NULL);
}

void publish_scan()
{
    float d[MAX_TOF_SENSORS];
    uint8_t n = robot.get_lidar_values(d, MAX_TOF_SENSORS);
    for (uint8_t i = 0; i < MAX_TOF_SENSORS; ++i)
    {
        // Distance mesurée depuis le centre de l'anneau, comme le lidar simulé ; INFINITY = rien dans la portée
        float r = (i < n && !isnan(d[i])) ? d[i] + TOF_RING_RADIUS : INFINITY;
        scan_ranges[i] = (r >= SCAN_RANGE_MIN && r <= SCAN_RANGE_MAX) ? r : INFINITY;
    }
    stamp_now(scan_msg.header.stamp);
    rcl_publish(&scan_pub, &scan_msg, NULL);
}

void publish_floor()
{
    float d = robot.get_floor_distance();
    floor_msg.range = (!isnan(d) && d >= FLOOR_RANGE_MIN && d <= FLOOR_RANGE_MAX) ? d : INFINITY;
    stamp_now(floor_msg.header.stamp);
    rcl_publish(&floor_pub, &floor_msg, NULL);
}

// ─── Connexion à l'agent ────────────────────────────────────────────────────

bool create_entities()
{
    allocator = rcl_get_default_allocator();
    init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID);
    if (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator) != RCL_RET_OK)
    {
        return false;
    }
    if (rclc_node_init_default(&node, "armada_esp", ROBOT_NAME, &support) != RCL_RET_OK) // namespace = nom du robot
    {
        return false;
    }

    // Topics relatifs : /<ROBOT_NAME>/cmd_vel, ... Best effort : compatible avec tous les abonnés/publieurs, adapté au Wi-Fi.
    // SAUF odom : un message best effort ne peut pas dépasser le MTU (512 octets), et un Odometry en fait ~700
    // (deux covariances de 36 doubles). En reliable, micro-ROS le fragmente.
    bool ok = true;
    ok &= rclc_subscription_init_best_effort(&cmd_sub, &node,
                                             ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel") == RCL_RET_OK;
    ok &= rclc_publisher_init_default(&odom_pub, &node,
                                      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom") == RCL_RET_OK;
    ok &= rclc_publisher_init_best_effort(&scan_pub, &node,
                                          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "scan") == RCL_RET_OK;
    ok &= rclc_publisher_init_best_effort(&floor_pub, &node,
                                          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), "floor/range") == RCL_RET_OK;
    if (!ok)
    {
        return false;
    }

    executor = rclc_executor_get_zero_initialized_executor();
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &cmd_callback, ON_NEW_DATA);

    rmw_uros_sync_session(1000); // horodatage aligné sur le PC
    odom_has_prev = false;
    return true;
}

void destroy_entities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rclc_executor_fini(&executor);
    rcl_subscription_fini(&cmd_sub, &node);
    rcl_publisher_fini(&odom_pub, &node);
    rcl_publisher_fini(&scan_pub, &node);
    rcl_publisher_fini(&floor_pub, &node);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
    rcl_init_options_fini(&init_options);
}

// ─── Arduino ────────────────────────────────────────────────────────────────

void setup()
{
    Serial.begin(115200);

    robot.init();
    robot.set_command_timeout(COMMAND_TIMEOUT_MS);
    init_messages();

    set_microros_wifi_transports((char *)WIFI_SSID, (char *)WIFI_PASSWORD, (char *)AGENT_IP, AGENT_PORT);
    Serial.printf("[micro-ROS] robot \"%s\", agent %s:%d\n", ROBOT_NAME, AGENT_IP, AGENT_PORT);
    state = WAITING_AGENT;
}

void loop()
{
    // Toujours actif, connecté ou non : arrêt de sécurité + mesures des capteurs
    robot.update();
    robot.floor_sensor().update();
    robot.lidar().update_next();

    switch (state)
    {
    case WAITING_AGENT:
        EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        break;
    case AGENT_AVAILABLE:
        state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
        if (state == WAITING_AGENT)
        {
            destroy_entities();
        }
        else
        {
            Serial.println("[micro-ROS] connecte a l'agent");
        }
        break;
    case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        if (state == AGENT_CONNECTED)
        {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));

            uint32_t now = millis();
            if (now - last_odom_ms >= ODOM_PERIOD_MS)
            {
                publish_odom();
                last_odom_ms = millis(); // aussi si le servo n'a pas répondu : pas de rafale de lectures
            }
            if (now - last_scan_ms >= SCAN_PERIOD_MS)
            {
                last_scan_ms = now;
                publish_scan();
            }
            if (now - last_floor_ms >= FLOOR_PERIOD_MS)
            {
                last_floor_ms = now;
                publish_floor();
            }
        }
        break;
    case AGENT_DISCONNECTED:
        robot.stop(); // plus de pilote : on ne roule pas
        destroy_entities();
        Serial.println("[micro-ROS] agent perdu");
        state = WAITING_AGENT;
        break;
    default:
        break;
    }
}
