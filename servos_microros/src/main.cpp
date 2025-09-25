#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

void setup() {
  // Inicializa Serial para micro-ROS
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  delay(2000); // tiempo para estabilizar

  allocator = rcl_get_default_allocator();

  // Inicializa soporte
  rclc_support_init(&support, 0, NULL, &allocator);

  // Crea nodo
  rclc_node_init_default(&node, "esp32_node", "", &support);

  // Crea publisher
  rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "counter");
}

void loop() {
  static int count = 0;
  msg.data = count++;

  rcl_publish(&publisher, &msg, NULL);

  delay(1000); // Publica cada segundo
}
