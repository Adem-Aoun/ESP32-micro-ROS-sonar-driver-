#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/range.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

// HC-SR04 Ultrasonic Sensor Pinout for ESP32
#define TRIG_PIN 4    // Trigger pin - GPIO4
#define ECHO_PIN 16   // Echo pin - GPIO16

// Sensor specifications for HC-SR04
#define MIN_RANGE 0.02  // 2cm minimum range
#define MAX_RANGE 4.00  // 400cm maximum range
#define FOV 0.26        // ~15 degrees field of view in radians

// Kalman Filter Structure
struct KalmanFilter {
  float x;        // State estimate (distance)
  float P;        // Error covariance
  float Q;        // Process noise covariance
  float R;        // Measurement noise covariance
  float K;        // Kalman gain
  bool initialized;
};

rcl_publisher_t publisher;
rcl_publisher_t raw_publisher;  // For publishing raw measurements
sensor_msgs__msg__Range range_msg;
sensor_msgs__msg__Range raw_range_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Kalman filter instance
KalmanFilter kf;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    digitalWrite(2, HIGH);  // Built-in LED
    delay(100);
    digitalWrite(2, LOW);
    delay(100);
  }
}

// Initialize Kalman Filter
void initKalmanFilter(KalmanFilter* kf) {
  kf->x = 0.0;           // Initial state estimate
  kf->P = 1.0;           // Initial error covariance
  kf->Q = 0.01;          // Process noise (how much we expect the true distance to change)
  kf->R = 0.1;           // Measurement noise (sensor uncertainty)
  kf->K = 0.0;           // Kalman gain
  kf->initialized = false;
}

// Kalman Filter Prediction Step
void kalmanPredict(KalmanFilter* kf) {
  // Predict the state (assuming constant distance model)
  // x_pred = x_prev (no change expected)
  // P_pred = P_prev + Q
  kf->P = kf->P + kf->Q;
}

// Kalman Filter Update Step
void kalmanUpdate(KalmanFilter* kf, float measurement) {
  if (!kf->initialized) {
    // Initialize with first measurement
    kf->x = measurement;
    kf->initialized = true;
    return;
  }
  
  // Calculate Kalman gain
  kf->K = kf->P / (kf->P + kf->R);
  
  // Update state estimate
  kf->x = kf->x + kf->K * (measurement - kf->x);
  
  // Update error covariance
  kf->P = (1.0 - kf->K) * kf->P;
}

// Complete Kalman Filter Step
float kalmanFilter(KalmanFilter* kf, float measurement) {
  kalmanPredict(kf);
  kalmanUpdate(kf, measurement);
  return kf->x;
}

// Function to get current time in seconds
double get_time_seconds() {
  return millis() / 1000.0;
}

// Function to read distance from HC-SR04
float readDistance() {
  // Clear the trigger pin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);
  
  // Send 10us pulse to trigger pin
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Read the echo pin, returns the sound wave travel time in microseconds
  long duration = pulseIn(ECHO_PIN, HIGH, 50000); // 50ms timeout
  
  // Check for timeout (no echo received)
  if (duration == 0) {
    return -1.0; // Return -1 to indicate error/out of range
  }
  
  // Calculate distance in meters (ROS2 standard unit)
  // Speed of sound = 343 m/s = 0.000343 m/us
  // Distance = (duration * 0.000343) / 2 (divide by 2 for round trip)
  float distance = (duration * 0.000343) / 2.0;
  
  // HC-SR04 effective range check
  if (distance < MIN_RANGE || distance > MAX_RANGE) {
    return -1.0; // Out of range
  }
  
  return distance;
}

// Function to fill Range message
void fillRangeMessage(sensor_msgs__msg__Range* msg, float range_value, const char* frame_suffix) {
  // Get current time
  double current_time = get_time_seconds();
  
  // Fill the Range message header
  msg->header.stamp.sec = (int32_t)current_time;
  msg->header.stamp.nanosec = (uint32_t)((current_time - (int32_t)current_time) * 1e9);
  
  // Set frame_id
  char frame_id[30];
  snprintf(frame_id, sizeof(frame_id), "ultrasonic_%s", frame_suffix);
  strcpy(msg->header.frame_id.data, frame_id);
  msg->header.frame_id.size = strlen(frame_id);
  
  // Set sensor type and specifications
  msg->radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
  msg->field_of_view = FOV;
  msg->min_range = MIN_RANGE;
  msg->max_range = MAX_RANGE;
  
  // Set the range measurement
  if (range_value > 0) {
    msg->range = range_value;
  } else {
    // For out of range or error, set to max_range + 1 to indicate invalid
    msg->range = MAX_RANGE + 1.0;
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Read raw distance from ultrasonic sensor
    float raw_distance = readDistance();
    
    // Apply Kalman filter only if we have a valid measurement
    float filtered_distance = raw_distance;
    if (raw_distance > 0) {
      filtered_distance = kalmanFilter(&kf, raw_distance);
    }
    
    // Fill and publish raw measurement
    fillRangeMessage(&raw_range_msg, raw_distance, "raw");
    RCSOFTCHECK(rcl_publish(&raw_publisher, &raw_range_msg, NULL));
    
    // Fill and publish filtered measurement
    fillRangeMessage(&range_msg, filtered_distance, "filtered");
    RCSOFTCHECK(rcl_publish(&publisher, &range_msg, NULL));
    
    // Debug output (optional - can be removed for production)
    if (raw_distance > 0) {
      Serial.print("Raw: ");
      Serial.print(raw_distance, 3);
      Serial.print("m, Filtered: ");
      Serial.print(filtered_distance, 3);
      Serial.print("m, Gain: ");
      Serial.println(kf.K, 3);
    }
  }
}

void setup() {
  // Initialize pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(2, OUTPUT);  // Built-in LED
  
  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  // Initialize Kalman Filter
  initKalmanFilter(&kf);

  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "ultrasonic_sensor_node", "", &support));

  // Create publisher for filtered data
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    "ultrasonic_sensor/range"));

  // Create publisher for raw data
  RCCHECK(rclc_publisher_init_default(
    &raw_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    "ultrasonic_sensor/range_raw"));

  // Create timer - 50ms interval for distance readings (20Hz)
  const unsigned int timer_timeout = 50;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // Initialize the Range messages
  // Allocate memory for frame_id strings
  range_msg.header.frame_id.data = (char*)malloc(30 * sizeof(char));
  range_msg.header.frame_id.capacity = 30;
  
  raw_range_msg.header.frame_id.data = (char*)malloc(30 * sizeof(char));
  raw_range_msg.header.frame_id.capacity = 30;
  
  // Brief LED flash to indicate successful initialization
  digitalWrite(2, HIGH);
  delay(500);
  digitalWrite(2, LOW);
  
  Serial.println("Ultrasonic sensor with Kalman filter initialized!");
  Serial.println("Publishing filtered data to: /ultrasonic_sensor/range");
  Serial.println("Publishing raw data to: /ultrasonic_sensor/range_raw");
  Serial.println("Kalman Filter Parameters:");
  Serial.print("  Process Noise (Q): "); Serial.println(kf.Q);
  Serial.print("  Measurement Noise (R): "); Serial.println(kf.R);
}

void loop() {
  delay(25);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(25)));
}

/*
HC-SR04 Ultrasonic Sensor with Kalman Filter - ROS2 Humble
==========================================================

Kalman Filter Benefits:
- Smooths noisy sensor readings
- Reduces outliers and spikes
- Provides more stable distance measurements
- Adapts to measurement uncertainty

Wiring for ESP32:
HC-SR04 Pin  |  ESP32 Pin    |  Description
-------------|---------------|----------------------------------
VCC          |  5V or 3.3V   |  Power supply
GND          |  GND          |  Ground
Trig         |  GPIO4        |  Trigger pin
Echo         |  GPIO16       |  Echo pin

ROS2 Topics:
===========
- /ultrasonic_sensor/range      (Kalman filtered data)
- /ultrasonic_sensor/range_raw  (Raw sensor data)

Message Type: sensor_msgs/Range
Frame IDs: ultrasonic_filtered, ultrasonic_raw
Update Rate: 20 Hz

Kalman Filter Tuning:
====================
Process Noise (Q): 0.01
- Lower = assumes distance changes slowly
- Higher = allows faster adaptation to changes

Measurement Noise (R): 0.1  
- Lower = trusts sensor measurements more
- Higher = relies more on prediction

Micro-ROS Agent Setup:
=====================
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0

ROS2 Commands:
=============
# Compare raw vs filtered data
ros2 topic echo /ultrasonic_sensor/range_raw
ros2 topic echo /ultrasonic_sensor/range

# Plot both topics for comparison
ros2 run plotjuggler plotjuggler

# Check message rates
ros2 topic hz /ultrasonic_sensor/range
ros2 topic hz /ultrasonic_sensor/range_raw

Visualization in RViz2:
======================
1. ros2 run rviz2 rviz2
2. Add -> By topic -> /ultrasonic_sensor/range -> Range
3. Add -> By topic -> /ultrasonic_sensor/range_raw -> Range
4. Set different colors to compare filtered vs raw

Advanced Tuning:
===============
Modify these values in code for different scenarios:
- Q = 0.01: Normal indoor use
- Q = 0.05: Moving robot or changing environment  
- Q = 0.005: Very stable environment

- R = 0.1: Normal HC-SR04 uncertainty
- R = 0.05: High-quality sensor or controlled environment
- R = 0.2: Noisy environment or poor sensor mounting
*/