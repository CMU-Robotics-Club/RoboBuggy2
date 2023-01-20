#include "Dynamixel.h"
#include "DynamixelInterface.h"
#include "DynamixelMotor.h"

/* ============= */
/* Board Config  */
/* ============= */

#define BOARD_V1
#ifdef BOARD_V1

#define DYNAMIXEL_SERIAL Serial5
#define DYNAMIXEL_RXEN 14
#define DYNAMIXEL_TXEN 15

#define BRAKE_RELAY_PIN 25
#define INTERRUPT_PIN 41

#define STEER_PIN 26
#define BRAKE_PIN 27

#else // Breadboard

#define DYNAMIXEL_SERIAL Serial7
#define DYNAMIXEL_RXEN 27
#define DYNAMIXEL_TXEN 33

#define BRAKE_RELAY_PIN 25
#define INTERRUPT_PIN 41

#define STEER_PIN 40
#define BRAKE_PIN 7

#endif

#define NUM_RC_CHANNELS 2

#define RC_STEER 0
#define RC_BRAKE 1

/* ============= */
/* RC Controller */
/* ============= */

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

BuggyState buggy_state = BuggyState::Stopped0;
int buggy_arming_timer_ms = 0;

#define ARM_TIME_MS 1000

#define BRAKE_TIME_MS 100

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < NUM_RC_CHANNELS; ++i) {
    pinMode(rc_pins[i], INPUT);

    attachInterrupt(digitalPinToInterrupt(rc_pins[i]), rc_interrupt_handlers[i], CHANGE);
  }

  pinMode(BRAKE_RELAY_PIN, OUTPUT);
  digitalWrite(BRAKE_RELAY_PIN, LOW);
  pinMode(INTERRUPT_PIN, OUTPUT);
}


int i = 0;

double t = 0.0;

void loop()
{
  Serial.println("foobar");
  
  DynamixelInterface dInterface(DYNAMIXEL_SERIAL, DYNAMIXEL_RXEN, DYNAMIXEL_TXEN, DirPinMode::ReadHiWriteLo); // Stream
  dInterface.begin(1000000, 50); // baudrate, timeout
  DynamixelMotor motor(dInterface, 5);  // Interface , ID

  motor.init(); // This will get the returnStatusLevel of the servo
  Serial.printf("Status return level = %u\n", motor.statusReturnLevel());

  motor.statusReturnLevel(1);

  Serial.printf("new return level = %u\n", motor.statusReturnLevel());

  // Status packet delay = 20us
  motor.write(5, (byte)250);

  motor.jointMode(); // Set the angular limits of the servo. Set to [min, max] by default
  motor.enableTorque();

  int rc_pin_widths[NUM_RC_CHANNELS] = { 0 };
  bool rc_pin_timed_out[NUM_RC_CHANNELS] = { false };

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

    bool throttleAuto = (1750 <= rc_pin_widths[RC_BRAKE]);
    bool throttleTele = (rc_pin_widths[RC_BRAKE] <= 1250);
    bool throttleNeutral = !throttleAuto && !throttleTele;

    switch (buggy_state) {
      case BuggyState::Stopped0:
        // Must be moved to AUTONOMOUS
        if (throttleAuto) {
          if (millis() - buggy_arming_timer_ms > ARM_TIME_MS) {
            buggy_arming_timer_ms = millis();
            buggy_state = BuggyState::Stopped1;
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
            buggy_state = BuggyState::Stopped0;
          }
        } else {
          buggy_arming_timer_ms = millis();
        }
        break;
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

    if (buggy_state == BuggyState::Moving) {
      if (throttleTele) {
        float range = (rc_pin_widths[RC_STEER] - 1000.0) / 1000.0;

        double angle = (0 ? sin(t / 100.0) : 0);
      
        motor.goalPositionDegree(90 + (range - 0.5) * 2.0 * 90.0);
      } else if (throttleAuto) {
        // TODO: Autonomomous
      }

      digitalWrite(BRAKE_RELAY_PIN, HIGH);
    } else {
      digitalWrite(BRAKE_RELAY_PIN, LOW);
    }

    delay(1);
  }
}
