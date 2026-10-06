/**
 * =============================================================================
 * @file main.cpp
 * @brief Real-time motor control and differential drive firmware using Micro-ROS on ESP32.
 *
 * Responsibilities:
 * - Controls dual DC motors via Pololu VNH5019 H-Bridge driver with PWM.
 * - Handles quadrature encoder pulses using hardware interrupts (3200 CPR).
 * - Implements a 50 Hz discrete PID velocity controller with feed-forward gain.
 * - Resolves differential inverse kinematics from incoming /cmd_vel messages.
 * - Publishes wheel rotational velocities (/left_motor_vel, /right_motor_vel) for odometry.
 * =============================================================================
 */

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <rmw_microros/rmw_microros.h>

// -----------------------------------------------------------------------------
// Error Checking Macros
// -----------------------------------------------------------------------------
#define RCCHECK(fn)     { rcl_ret_t rc = fn; if((rc != RCL_RET_OK)){ while(1){ delay(100); } } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

// -----------------------------------------------------------------------------
// Pin Definitions (Pololu VNH5019 Dual H-Bridge Driver)
// -----------------------------------------------------------------------------
const int IN1A = 17, IN1B = 16, PWM1 = 15;   // Left motor driver pins
const int IN2A = 6,  IN2B = 5,  PWM2 = 4;    // Right motor driver pins

// Quadrature Encoder Pins (Hardware Interrupts)
#define ENCODER_RIGHT_A 9
#define ENCODER_RIGHT_B 18
#define ENCODER_LEFT_A  10
#define ENCODER_LEFT_B  11

// Direction & Sign Inversions
const bool MOTOR_RIGHT_INVERT = true;
const bool MOTOR_LEFT_INVERT  = true;
const int  ENC_RIGHT_SIGN     = +1;
const int  ENC_LEFT_SIGN      = -1;

// -----------------------------------------------------------------------------
// Control Loop & Kinematic Constants
// -----------------------------------------------------------------------------
const float CONTROL_DT         = 0.02f;                       // Discrete control period [s] (50 Hz)
const int   MAX_PWM            = 255;                         // Maximum PWM resolution (8-bit)
const float MAX_VEL_CPS        = 6600.0f;                     // Maximum no-load wheel velocity [counts/s]
const float MAX_TARGET_CPS     = 0.80f * MAX_VEL_CPS;         // Safe target velocity limit [counts/s]
const long  VEL_DEADBAND_COUNTS = 1;                          // Static deadband threshold [counts]

const float TRACK_WIDTH        = 0.30f;                       // Track width between drive wheels [m]
const float COUNTS_PER_REV     = 3200.0f;                     // Encoder counts per full wheel rotation
const float WHEEL_DIAM_M       = 0.07f;                       // Wheel diameter [m]
const float M_PER_COUNT        = (3.14159265f * WHEEL_DIAM_M) / COUNTS_PER_REV;

// -----------------------------------------------------------------------------
// Velocity PID Controller Structure
// -----------------------------------------------------------------------------
struct PID {
  float kp, ki, kd, kff;
  float integral;
  float prev_err;
};

struct PID pid_right = { 0.02f, 0.05f, 0.0f, MAX_PWM / MAX_VEL_CPS };
struct PID pid_left  = { 0.02f, 0.05f, 0.0f, MAX_PWM / MAX_VEL_CPS };

// Target velocities [counts/s] (positive = forward)
volatile float target_right = 0.0f;
volatile float target_left  = 0.0f;

// Volatile encoder accumulators and ISR tracking states
volatile long encoderCountRight = 0;
volatile long encoderCountLeft  = 0;
volatile int  lastA_Right       = LOW;
volatile int  lastA_Left        = LOW;
long lastCountRight             = 0;
long lastCountLeft              = 0;

// -----------------------------------------------------------------------------
// Micro-ROS Handles & Message Structures
// -----------------------------------------------------------------------------
rcl_subscription_t left_sub, right_sub, cmdvel_sub;
rcl_publisher_t    left_vel_pub, right_vel_pub;
std_msgs__msg__Int32 left_msg, right_msg, left_vel_msg, right_vel_msg;
geometry_msgs__msg__Twist cmdvel_msg;

rcl_timer_t     control_timer;
rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;

// -----------------------------------------------------------------------------
// Hardware Interrupt Service Routines (ISRs) for Encoders
// -----------------------------------------------------------------------------
void IRAM_ATTR encoderISR_Right() {
  int a = digitalRead(ENCODER_RIGHT_A);
  int b = digitalRead(ENCODER_RIGHT_B);
  if (a != lastA_Right) {
    if (a == HIGH) encoderCountRight += (b == LOW)  ? 1 : -1;
    else           encoderCountRight += (b == HIGH) ? 1 : -1;
    lastA_Right = a;
  }
}

void IRAM_ATTR encoderISR_Left() {
  int a = digitalRead(ENCODER_LEFT_A);
  int b = digitalRead(ENCODER_LEFT_B);
  if (a != lastA_Left) {
    if (a == HIGH) encoderCountLeft += (b == LOW)  ? 1 : -1;
    else           encoderCountLeft += (b == HIGH) ? 1 : -1;
    lastA_Left = a;
  }
}

// -----------------------------------------------------------------------------
// Low-Level Motor Actuation Functions
// -----------------------------------------------------------------------------
void driveRight(int pwm) {
  if (MOTOR_RIGHT_INVERT) pwm = -pwm;
  pwm = constrain(pwm, -MAX_PWM, MAX_PWM);
  if (pwm >= 0) {
    digitalWrite(IN2A, HIGH);
    digitalWrite(IN2B, LOW);
    analogWrite(PWM2, pwm);
  } else {
    digitalWrite(IN2A, LOW);
    digitalWrite(IN2B, HIGH);
    analogWrite(PWM2, -pwm);
  }
}

void driveLeft(int pwm) {
  if (MOTOR_LEFT_INVERT) pwm = -pwm;
  pwm = constrain(pwm, -MAX_PWM, MAX_PWM);
  if (pwm >= 0) {
    digitalWrite(IN1A, HIGH);
    digitalWrite(IN1B, LOW);
    analogWrite(PWM1, pwm);
  } else {
    digitalWrite(IN1A, LOW);
    digitalWrite(IN1B, HIGH);
    analogWrite(PWM1, -pwm);
  }
}

void motorStopRight() {
  digitalWrite(IN2A, LOW);
  digitalWrite(IN2B, LOW);
  analogWrite(PWM2, 0);
}

void motorStopLeft() {
  digitalWrite(IN1A, LOW);
  digitalWrite(IN1B, LOW);
  analogWrite(PWM1, 0);
}

// -----------------------------------------------------------------------------
// PID Velocity Control Algorithm
// -----------------------------------------------------------------------------
float pidCompute(PID &p, float target, float measured, float dt) {
  float err = target - measured;
  float deriv = (err - p.prev_err) / dt;
  p.prev_err = err;
  p.integral += err * dt;

  // Anti-windup clamping on integral accumulator
  if (p.ki > 1e-6f) {
    float i_max = MAX_PWM / p.ki;
    p.integral = constrain(p.integral, -i_max, i_max);
  }

  // Combined feed-forward and PID output
  float out = p.kff * target + p.kp * err + p.ki * p.integral + p.kd * deriv;
  return constrain(out, -(float)MAX_PWM, (float)MAX_PWM);
}

void resetPID(PID &p) {
  p.integral = 0.0f;
  p.prev_err = 0.0f;
}

// -----------------------------------------------------------------------------
// Periodic Control Timer Callback (50 Hz)
// -----------------------------------------------------------------------------
void controlCallback(rcl_timer_t *timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer == NULL) return;

  // Read atomic snapshot of encoder pulse counters
  long cr = encoderCountRight;
  long cl = encoderCountLeft;
  long dR = cr - lastCountRight;  lastCountRight = cr;
  long dL = cl - lastCountLeft;   lastCountLeft  = cl;

  // Filter noise within encoder deadband
  if (labs(dR) <= VEL_DEADBAND_COUNTS) dR = 0;
  if (labs(dL) <= VEL_DEADBAND_COUNTS) dL = 0;

  // Calculate actual wheel velocities in counts per second
  float velR = (ENC_RIGHT_SIGN * dR) / CONTROL_DT;
  float velL = (ENC_LEFT_SIGN  * dL) / CONTROL_DT;

  // --- Right Wheel Control ---
  int outR;
  if (target_right == 0.0f) {
    resetPID(pid_right);
    outR = 0;
    motorStopRight();
  } else {
    outR = (int) pidCompute(pid_right, target_right, velR, CONTROL_DT);
    driveRight(outR);
  }

  // --- Left Wheel Control ---
  int outL;
  if (target_left == 0.0f) {
    resetPID(pid_left);
    outL = 0;
    motorStopLeft();
  } else {
    outL = (int) pidCompute(pid_left, target_left, velL, CONTROL_DT);
    driveLeft(outL);
  }

  // Publish wheel velocity feedback for ROS 2 odometry calculation
  right_vel_msg.data = (int32_t) velR;
  left_vel_msg.data  = (int32_t) velL;
  RCSOFTCHECK(rcl_publish(&right_vel_pub, &right_vel_msg, NULL));
  RCSOFTCHECK(rcl_publish(&left_vel_pub,  &left_vel_msg,  NULL));
}

// -----------------------------------------------------------------------------
// Velocity Target Scaling & Clamping
// -----------------------------------------------------------------------------
void setTargetsClamped(float r_cps, float l_cps) {
  float m = max(fabs(r_cps), fabs(l_cps));
  if (m > MAX_TARGET_CPS) {
    float scale = MAX_TARGET_CPS / m;
    r_cps *= scale;
    l_cps *= scale;
  }
  target_right = r_cps;
  target_left  = l_cps;
}

// -----------------------------------------------------------------------------
// Topic Callbacks
// -----------------------------------------------------------------------------
void cmdvel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *m = (const geometry_msgs__msg__Twist *)msgin;
  float v  = m->linear.x;   // Target linear velocity [m/s]
  float w  = m->angular.z;  // Target angular velocity [rad/s]
  float vr = v + w * (TRACK_WIDTH * 0.5f);
  float vl = v - w * (TRACK_WIDTH * 0.5f);
  setTargetsClamped(vr / M_PER_COUNT, vl / M_PER_COUNT);
}

// Direct wheel velocity setpoint subscriber (alternative control channel)
void left_callback(const void *msgin) {
  const std_msgs__msg__Int32 *m = (const std_msgs__msg__Int32 *)msgin;
  target_left = constrain((float) m->data, -MAX_TARGET_CPS, MAX_TARGET_CPS);
}

void right_callback(const void *msgin) {
  const std_msgs__msg__Int32 *m = (const std_msgs__msg__Int32 *)msgin;
  target_right = constrain((float) m->data, -MAX_TARGET_CPS, MAX_TARGET_CPS);
}

// -----------------------------------------------------------------------------
// System Setup
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  // Initialize H-Bridge motor direction and PWM pins
  pinMode(IN1A, OUTPUT); pinMode(IN1B, OUTPUT); pinMode(PWM1, OUTPUT);
  pinMode(IN2A, OUTPUT); pinMode(IN2B, OUTPUT); pinMode(PWM2, OUTPUT);
  motorStopRight(); motorStopLeft();

  // Attach hardware interrupts to optical encoder channels
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_A,  INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B,  INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), encoderISR_Right, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A),  encoderISR_Left,  CHANGE);

  // Block until micro-ROS agent responds on serial port
  while (rmw_uros_ping_agent(500, 10) != RMW_RET_OK) {
    delay(200);
  }

  // Initialize micro-ROS support structures
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_hardware_node", "", &support));

  // Initialize subscriptions
  RCCHECK(rclc_subscription_init_default(&left_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/left_motor_target"));
  RCCHECK(rclc_subscription_init_default(&right_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/right_motor_target"));
  RCCHECK(rclc_subscription_init_default(&cmdvel_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel"));

  // Initialize publishers
  RCCHECK(rclc_publisher_init_default(&left_vel_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/left_motor_vel"));
  RCCHECK(rclc_publisher_init_default(&right_vel_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/right_motor_vel"));

  // Initialize 50 Hz control timer
  RCCHECK(rclc_timer_init_default(&control_timer, &support,
    RCL_MS_TO_NS((int)(CONTROL_DT * 1000)), controlCallback));

  // Initialize executor with 4 handles (3 subscriptions + 1 timer)
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &left_sub,   &left_msg,   &left_callback,   ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &right_sub,  &right_msg,  &right_callback,  ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmdvel_sub, &cmdvel_msg, &cmdvel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

  lastCountRight = encoderCountRight;
  lastCountLeft  = encoderCountLeft;
}

// -----------------------------------------------------------------------------
// Main Execution Loop
// -----------------------------------------------------------------------------
void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}