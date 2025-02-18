/////////////////////
/// For Micro ROS ///
/////////////////////
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
/*
 * TODO : include your desired msg header file
 */
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <sensor_msgs/msg/imu.h>

#define LED_PIN 23
#define LED_PIN_TEST 18 // subscription LED
bool led_test_state = false;

/*
 * Helper functions to help reconnect
 */
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

/*
 * Declare rcl object
 */
rclc_support_t support;
rcl_init_options_t init_options;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;

/*
 * Declare pub&sub
 */
rcl_publisher_t int16_pub;
rcl_publisher_t int16array_pub;
rcl_subscription_t led_sub;
rcl_subscription_t int16array_sub;

/*
 * Define msgs
 */

std_msgs__msg__Int16 int16_msg;
std_msgs__msg__Int16MultiArray int16array_send_msg;
std_msgs__msg__Int16MultiArray int16array_recv_msg;
std_msgs__msg__Bool led_msg;

/*
 * Define your subscription callbacks (the last one has to be timer_callback)
 */

void led_callback(const void *msgin)
{

    std_msgs__msg__Bool *led_msg = (std_msgs__msg__Bool *)msgin; // the message

    led_test_state = led_msg->data;             // get the led status
    digitalWrite(LED_PIN_TEST, led_test_state); // print the led status
}

void int16array_callback(const void *msgin)
{

    std_msgs__msg__Int16MultiArray *int16array_recv_msg = (std_msgs__msg__Int16MultiArray *)msgin; // the message

    int16array_send_msg.data.data[0] = int16array_recv_msg->data.data[0];
    int16array_send_msg.data.data[1] = int16array_recv_msg->data.data[1];
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer != NULL)
    {

        /*
           Publish anything inside here / will pub after each timer completion
        */

        rcl_publish(&int16array_pub, &int16array_send_msg, NULL);

        int16_msg.data++;
        rcl_publish(&int16_pub, &int16_msg, NULL);
    }
}

/*
   Create object (Initialization)
*/

/*
   Create object (Initialization)
*/
bool create_entities()
{
    /*
      Define your
       - ROS node name
       - namespace
       - ROS_DOMAIN_ID
    */
    const char *node_name = "esp32_test";
    const char *ns = "";
    const int domain_id = 0;

    /*
     * Initialize node
     */
    allocator = rcl_get_default_allocator();
    init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, domain_id);
    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
    rclc_node_init_default(&node, node_name, ns, &support);

    /*
     Init of publisher and subscriber
     */
    rclc_publisher_init(
        &int16array_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
        "/esp/int16array_pub", &rmw_qos_profile_default);

    rclc_publisher_init(
        &int16_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
        "/esp/int_pub", &rmw_qos_profile_default);

    rclc_subscription_init(
        &int16array_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
        "/esp/int16array_sub", &rmw_qos_profile_default);

    rclc_subscription_init(
        &led_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/esp/led", &rmw_qos_profile_default);

    /*
     * Init timer_callback
     * 50ms : 20Hz
     * 20ms : 50Hz
     * 10ms : 100Hz
     */
    const unsigned int timer_timeout = 50;
    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback);

    /*
     * Init Executor
     * num_handles = total_of_subscriber + timer
     * publisher is not counted
     *
     * name of sub msg and callback has to be correct
     */
    unsigned int num_handles = 3;
    executor = rclc_executor_get_zero_initialized_executor();
    rclc_executor_init(&executor, &support.context, num_handles, &allocator);
    rclc_executor_add_subscription(&executor, &int16array_sub, &int16array_recv_msg, &int16array_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &led_sub, &led_msg, &led_callback, ON_NEW_DATA);
    rclc_executor_add_timer(&executor, &timer);

    return true;
}

/*
 * Clean up all the created objects
 */
void destroy_entities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_timer_fini(&timer);
    rclc_executor_fini(&executor);
    rcl_init_options_fini(&init_options);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
    /*
     name of publisher and subscriber has to be correct
     */
    rcl_publisher_fini(&int16array_pub, &node);
    rcl_publisher_fini(&int16_pub, &node);
    rcl_subscription_fini(&int16array_sub, &node);
    rcl_subscription_fini(&led_sub, &node);
}

void setup()
{
    /*
     select either of USB or WiFi
     * comment the one that not use
     */
    // set_microros_transports();
    Serial.begin(9600);
    Serial.println("init setup");
    set_microros_wifi_transports("Bbox-DCA15097", "uhzW#PnunvUD%zAc", "192.168.1.18", 8888);

    /*
     * Optional, setup output pin for LEDs
     */
    pinMode(LED_PIN, OUTPUT);
    pinMode(LED_PIN_TEST, OUTPUT);

    /*
     * TODO : Initialze the message data variable
     */
    int16_t array_data[2] = {0, 0};
    int16array_send_msg.data.capacity = 2; // number of array length
    int16array_send_msg.data.size = 2;     // number of array length
    int16array_send_msg.data.data = array_data;

    int16array_recv_msg.data.capacity = 2; // number of array length
    int16array_recv_msg.data.size = 2;     // number of array length
    int16array_recv_msg.data.data = array_data;

    int16_msg.data = 0;

    led_msg.data = false;

    /*
     * Setup first state
     */
    state = WAITING_AGENT;
    Serial.println("setup successful, launching the loop!");
}

void loop()
{
    /*
     * Try ping the micro-ros-agent (HOST PC)
     * from the example
     * https://github.com/micro-ROS/micro_ros_arduino/blob/galactic/examples/micro-ros_reconnection_example/micro-ros_reconnection_example.ino
     *
     */
    Serial.println(state);
    switch (state)
    {
    case WAITING_AGENT:
        EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        break;
    case AGENT_AVAILABLE:
        state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
        if (state == WAITING_AGENT)
        {
            destroy_entities();
        };
        break;
    case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        if (state == AGENT_CONNECTED)
        {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        }
        break;
    case AGENT_DISCONNECTED:
        destroy_entities();
        state = WAITING_AGENT;
        break;
    default:
        break;
    }
    /*
     * Output LED when in AGENT_CONNECTED state
     */
    if (state == AGENT_CONNECTED)
    {
        digitalWrite(LED_PIN, 1);
    }
    else
    {
        digitalWrite(LED_PIN, 0);
    }

    /*
     * TODO :
     * Do anything else you want to do here,
     * like read sensor data,
     * calculate something, etc.
     */
}