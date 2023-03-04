#include "Dynamixel.h"
#include "DynamixelInterface.h"
#include "DynamixelMotor.h"

//#include <XBee.h>

#define USE_TEENSY_HW_SERIAL
#define ROS_BAUD 1000000

#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt32.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>

/* ============= */
/* Board Config  */
/* ============= */

#define BOARD_V1
#ifdef BOARD_V1

// Dynamixel Pins
#define DYNAMIXEL_SERIAL Serial5
#define DYNAMIXEL_RXEN 14
#define DYNAMIXEL_TXEN 15

// Control Pins
#define BRAKE_RELAY_PIN 25
#define INTERRUPT_PIN 41

// Radio PWM Pins
#define STEER_PIN 26
#define BRAKE_PIN 27

#else // Breadboard

// Dynamixel Pins
#define DYNAMIXEL_SERIAL Serial7
#define DYNAMIXEL_RXEN 27
#define DYNAMIXEL_TXEN 33

// Control Pins
#define BRAKE_RELAY_PIN 25
#define INTERRUPT_PIN 41

// Radio PWM Pins
#define STEER_PIN 40
#define BRAKE_PIN 7

#endif

/* ============= */
/* ROS Serial    */
/* ============= */

volatile double ros_servo_angle = 0.0;
volatile double ros_brake = 1.0;

void steer_cb(const std_msgs::Float64& cmd_msg){
  ros_servo_angle = cmd_msg.data;
}

void brake_cb(const std_msgs::Float64& cmd_msg){
  ros_brake = cmd_msg.data;
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Float64> steer("SteerOut_T", steer_cb);
ros::Subscriber<std_msgs::Float64> brake("BrakeOut_T", brake_cb);

diagnostic_msgs::DiagnosticStatus teensy_status;
diagnostic_msgs::KeyValue status_values[7];
ros::Publisher debug("TeensyStateIn_T", &teensy_status);

/* ============= */
/* RC Controller */
/* ============= */

#define NUM_RC_CHANNELS 2

#define RC_STEER 0
#define RC_BRAKE 1

typedef void (*interrupt_handler)(void);

const int rc_pins[NUM_RC_CHANNELS] = { STEER_PIN, BRAKE_PIN };

// Ranges from 1000 to 2000 under normal conditions.
volatile int rc_width[NUM_RC_CHANNELS] = { 0 };
// Set to millis() when an edge occurs.
volatile unsigned long rc_last_edge[NUM_RC_CHANNELS] = { 0 };

#define CONCAT( A, B ) A ## B
#define CREATE_PDM_INTERRUPT_HANDLER( RC , PIN )  \
  do {                                            \
    rc_last_edge[ RC ] = millis();                \
    static unsigned long start_time = 0;          \
    if (digitalRead( PIN )) {                     \
      start_time = micros();                      \
    } else {                                      \
      int new_width = micros() - start_time;      \
      if (10 <= new_width && new_width <= 2000) { \
        rc_width[ RC ] = new_width;               \
      }                                           \
      if (new_width > 2000) { digitalWrite(INTERRUPT_PIN, HIGH); digitalWrite(INTERRUPT_PIN, LOW); } \
    }                                             \
  } while (0);


void steer_pin_change() {
  CREATE_PDM_INTERRUPT_HANDLER( RC_STEER , STEER_PIN );
}

void brake_pin_change() {
  CREATE_PDM_INTERRUPT_HANDLER( RC_BRAKE , BRAKE_PIN );
}

const interrupt_handler rc_interrupt_handlers[NUM_RC_CHANNELS] = { steer_pin_change, brake_pin_change };

#undef CONCAT
#undef CREATE_PDM_INTERRUPT_HANLDER

#define AVERAGE_AMT 50

volatile int next_pin_3 = 0;
volatile int pin_3_widths[AVERAGE_AMT] = { 0 };

void pin_2_change(void);
void pin_3_change(void);

enum class BuggyState {
  Armed,

  Moving,

  Stopped0,
  Stopped1,
  Stopped2,
};

BuggyState buggy_state = BuggyState::Armed;
int buggy_arming_timer_ms = 0;

const char *get_buggy_state_str() {
  switch (buggy_state) {
    case BuggyState::Armed:    return "Armed   ";
    case BuggyState::Moving:   return "Moving  ";
    case BuggyState::Stopped0: return "Stopped0";
    case BuggyState::Stopped1: return "Stopped1";
    case BuggyState::Stopped2: return "Stopped2";
    default:                   return "UNKNOWN ";
  }
}

#define ARM_TIME_MS 1000

#define BRAKE_TIME_MS 100

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < NUM_RC_CHANNELS; ++i) {
    pinMode(rc_pins[i], INPUT);

    attachInterrupt(digitalPinToInterrupt(rc_pins[i]), rc_interrupt_handlers[i], CHANGE);
  }

  nh.getHardware()->setBaud(ROS_BAUD);
  nh.initNode();
  nh.subscribe(steer);
  nh.subscribe(brake);
  nh.advertise(debug);

  pinMode(BRAKE_RELAY_PIN, OUTPUT);
  digitalWrite(BRAKE_RELAY_PIN, LOW);
  pinMode(INTERRUPT_PIN, OUTPUT);
}

/*#define ANGLE_LIMIT_LO 880
#define ANGLE_LIMIT_HI 1440
#define ANGLE_RANGE (ANGLE_LIMIT_HI - ANGLE_LIMIT_LO)
#define ANGLE_CENTER ((ANGLE_LIMIT_HI + ANGLE_LIMIT_LO) / 2)*/

int i = 0;

double t = 0.0;

int left_angle_limit = 1460;
int right_angle_limit = 890;

void recenterWheel(DynamixelMotor &motor) {

  int a = 0;

  uint16_t start = -1;
  a = motor.currentPosition(start);

  Serial.printf("Start position is %d %d\n", start, a);

  if (start == -1) {
    return;
  }

  a = motor.goalPosition(start - 100);
  Serial.println(a);

  int offset = 0;
  while (true) {
    offset += 5;
    a = motor.goalPosition(start - offset);
    Serial.println(a);

    delay(100);

    uint16_t current_pos;
    a = motor.currentPosition(current_pos);

    Serial.printf("right offset %d, current position is %d %d\n", start - offset, current_pos, a);

    if (abs((int)current_pos - (int)(start - offset)) > 100) {
      right_angle_limit = current_pos + 100;
      Serial.printf("Stopping right limit at %d\n", right_angle_limit);
      break;
    }
  }

  Serial.println("Going to center");

  motor.goalPosition(start);
  delay(500);

  offset = 0;
  while (true) {
    offset += 5;
    motor.goalPosition(start + offset);

    delay(100);

    uint16_t current_pos;
    a = motor.currentPosition(current_pos);
    Serial.printf("offset %d, current position is %d %d\n", start + offset, current_pos, a);

    if (abs((int)current_pos - (int)(start + offset)) > 100) {
      left_angle_limit = current_pos - 100;
      Serial.printf("Stopping left limit at %d\n", left_angle_limit);
      break;
    }
  }

  motor.goalPosition((left_angle_limit + right_angle_limit) / 2);
  delay(500);
}

double map_pin_width(int pin_width) {
  float range = (pin_width - 1000.0) / 650.0; // 0 to 1

  float midpoint = 0.51;

  float mapped;

  if (range > midpoint) {
    mapped = (range - midpoint) / (1.0 - midpoint) * 0.5 + 0.5;
  } else {
    mapped = 0.5 - (midpoint - range) / (midpoint) * 0.5;
  }

  mapped = (mapped * 2.0) - 1.0;

  mapped = ((mapped >= 0) ? 1 : -1) * (mapped * mapped);

  return (mapped + 1.0) / 2.0;
}

void loop()
{
  delay(3000);

  Serial.println("hello, world");

  Serial2.begin(115200);

  /*XBee xbee = XBee();
  xbee.setSerial(Serial1);

  char payload[] = "HELLO, WORLD!\n";

  while (true) {
    Tx16Request tx = Tx16Request(0x0002, payload, sizeof(payload));

    TxStatusResponse txStatus = TxStatusResponse();

    Serial.println("Response!");
    Serial.println(txStatus.isSuccess());

    delay(1000);
  }*/
  
  DynamixelInterface dInterface(DYNAMIXEL_SERIAL, DYNAMIXEL_RXEN, DYNAMIXEL_TXEN, DirPinMode::ReadHiWriteLo); // Stream
  dInterface.begin(1000000, 50); // baudrate, timeout
  DynamixelMotor motor(dInterface, 5);  // Interface , ID


  motor.init(); // This will get the returnStatusLevel of the servo
  Serial.printf("Status return level = %u\n", motor.statusReturnLevel());
  

  motor.statusReturnLevel(1);

  Serial.printf("new return level = %u\n", motor.statusReturnLevel());

  // Status packet delay = 20us
  motor.write(5, (byte)250);
  
  motor.enableTorque(false);
  motor.jointMode(1, 0xFFF);
  motor.enableTorque();
  recenterWheel(motor);

  motor.enableTorque(false);
  motor.jointMode(right_angle_limit, left_angle_limit); // Set the angular limits of the servo. Set to [min, max] by default
  motor.enableTorque();

  int rc_pin_widths[NUM_RC_CHANNELS] = { 0 };
  bool rc_pin_timed_out[NUM_RC_CHANNELS] = { false };

  int log_timer = 0;

  while (true)
  {

    t += 5.0;
    unsigned long currentMillis = millis();

    for (int i = 0; i < NUM_RC_CHANNELS; ++i) {
      rc_pin_widths[i] = rc_width[i];
      // TODO: Perhaps we should warn if it's ever outside of this range
      //if (rc_pin_widths[i] < 1000) { rc_pin_widths[i] = 1000; }
      //if (rc_pin_widths[i] > 2000) { rc_pin_widths[i] = 2000; }
      
      // 50 ms is two and a half RC periods
      rc_pin_timed_out[i] = (currentMillis - rc_last_edge[i]) >= 50.0;
    }

    const int steer_average_count = 5;
    static int steer_average_samples[steer_average_count] = { 0 };
    static int steer_average_idx = 0;

    steer_average_samples[steer_average_idx++] = rc_pin_widths[RC_STEER];
    steer_average_idx %= steer_average_count;
    float steer_average = 0.0;
    for (int i = 0; i < steer_average_count; ++i) {
      steer_average += steer_average_samples[i];
    }
    steer_average /= steer_average_count;
    

    bool throttleAuto = (1750 <= rc_pin_widths[RC_BRAKE]);
    bool throttleTele = (rc_pin_widths[RC_BRAKE] <= 1250);
    bool throttleNeutral = !throttleAuto && !throttleTele;

    switch (buggy_state) {
      case BuggyState::Stopped0:
        // Must be moved to AUTONOMOUS
        if (throttleAuto) {
          if (millis() - buggy_arming_timer_ms > ARM_TIME_MS) {
            buggy_arming_timer_ms = millis();

          }
        } else {
          buggy_arming_timer_ms = millis();
        }
        break;
      
      case BuggyState::Stopped1:
        // Must be moved to TELEOP
        if (throttleTele) {
          if (millis() - buggy_arming_timer_ms > ARM_TIME_MS) {
            buggy_arming_timer_ms = millis();
            buggy_state = BuggyState::Stopped2;
          }
        } else {
          buggy_arming_timer_ms = millis();
        }
        break;
      
      case BuggyState::Stopped2:
        if (throttleNeutral) {
          if (millis() - buggy_arming_timer_ms > ARM_TIME_MS) {
            buggy_arming_timer_ms = millis();
            buggy_state = BuggyState::Armed;
          }
        } else {
          buggy_arming_timer_ms = millis();
        }
        break;
      
      case BuggyState::Armed:
        if (throttleTele || throttleAuto) {
          if (millis() - buggy_arming_timer_ms > ARM_TIME_MS) {
            buggy_arming_timer_ms = millis();
            buggy_state = BuggyState::Moving;
          }
        } else {
          buggy_arming_timer_ms = millis();
        }
        break;
      
      default:
        if (throttleNeutral) {
          if (millis() - buggy_arming_timer_ms > BRAKE_TIME_MS) {
            buggy_arming_timer_ms = millis();

            buggy_state = BuggyState::Armed;
          }
        } else {
          buggy_arming_timer_ms = millis();
        }
        break;
    }

    
    ++log_timer;
    if (log_timer == 100) {
      log_timer = 0;

      Serial2.printf("state: %s   ", get_buggy_state_str());
      int steer_age = currentMillis - rc_last_edge[RC_STEER];
      int brake_age = currentMillis - rc_last_edge[RC_BRAKE];
      Serial2.printf("str wdt/tmt: %4d %d   ", rc_pin_widths[RC_STEER], steer_age);
      Serial2.printf("thr wdt/tmt: %4d %d   ", rc_pin_widths[RC_BRAKE], brake_age);

      uint16_t current_pos = 65535;
      motor.currentPosition(current_pos);
      Serial2.printf("dyna pos %4d\n", current_pos);
      
      char steer_width[32];
      char steer_time[32];
      char brake_width[32];
      char brake_time[32];
      char pos_str[32];
      char avg_str[32];

      String(rc_pin_widths[RC_STEER], DEC).toCharArray(&steer_width[0], 32);
      String(steer_age, DEC).toCharArray(&steer_time[0], 32);
      String(rc_pin_widths[RC_BRAKE], DEC).toCharArray(&brake_width[0], 32);
      String(brake_age, DEC).toCharArray(&brake_time[0], 32);
      String(current_pos, DEC).toCharArray(&pos_str[0], 32);
      String(steer_average, DEC).toCharArray(&avg_str[0], 32);

      teensy_status.name = "Drive Teensy";
      teensy_status.level = diagnostic_msgs::DiagnosticStatus::OK;
      teensy_status.message = "Logging Status";
      teensy_status.values = &status_values[0];
      status_values[0].key = "state";
      status_values[0].value = get_buggy_state_str();
      status_values[1].key = "str wdt";
      status_values[1].value = steer_width;
      status_values[2].key = "str tmt";
      status_values[2].value = steer_time;
      status_values[3].key = "thr wdt";
      status_values[3].value = brake_width;
      status_values[4].key = "thr tmt";
      status_values[4].value = brake_time;
      status_values[5].key = "dyna pos";
      status_values[5].value = pos_str;
      status_values[6].key = "avg_str";
      status_values[6].value = avg_str;
      teensy_status.values_length = sizeof(status_values)/sizeof(diagnostic_msgs::KeyValue);
      debug.publish(&teensy_status);
    }

    //uint16_t currentPos = 0;
    //motor.read(36, currentPos);

    //uint16_t currentSpeed;
    //motor.read(38, currentSpeed);

    if (++i == 100) {
      i = 0;
      Serial.printf("pin width is %d\n", rc_pin_widths[RC_STEER]);
      Serial.printf("buggy state: %d\n", (int)buggy_state);
    }

    bool autoSteer = (buggy_state == BuggyState::Moving && throttleAuto);

    if (autoSteer) {
      // TODO: Autonomomous
      int angle_center = (left_angle_limit + right_angle_limit) / 2.0;
      motor.goalPosition(angle_center + ros_servo_angle * 3.41);

      float ros_brake = 1.0;
      bool engage_brakes = abs(ros_brake - 1.0) < 1e-6;
      digitalWrite(BRAKE_RELAY_PIN, !engage_brakes);
    } else {
      float range = map_pin_width(steer_average);
      
      range = (range * 2.0) - 1.0; // -1 to 1

      /*if (range > 0.5) {
        range = (range - 0.5) * 2.0 + 0.5;
      }*/

      int angle_center = (left_angle_limit + right_angle_limit) / 2.0;
      int angle_range = (right_angle_limit - left_angle_limit);

      motor.goalPosition(angle_center + angle_range * range * 0.5);
    
      //motor.goalPositionDegree(90 + (range - 0.5) * 2.0 * 90.0);

      digitalWrite(BRAKE_RELAY_PIN, (buggy_state == BuggyState::Moving));
    }

    delay(1);
    nh.spinOnce();
  }
}
