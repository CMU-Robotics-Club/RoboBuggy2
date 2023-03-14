#include "Dynamixel.h"
#include "DynamixelInterface.h"
#include "DynamixelMotor.h"

// #include <XBee.h>

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
#define BOARD_V1

// Dynamixel Pins
#define DYNAMIXEL_SERIAL Serial5
#define DYNAMIXEL_RXEN 14
#define DYNAMIXEL_TXEN 15

DynamixelMotor *motor;

// Teensy Pins
#define BRAKE_RELAY_PIN 25
#define INTERRUPT_PIN 41

// RC Controller PWM Pins
#define STEERING_PIN 26
#define THROTTLE_PIN 27

// Width of RC steering and brake pulses.
volatile int v_rcSteeringWidth = 0;
volatile int v_rcThrottleWidth = 0;

/**
 * @brief Timestamp of the most recent rising OR falling edge on the steering pin.
 */
volatile unsigned long rcSteeringLastEdge = 0;
/**
 * @brief Timestamp of the most recent rising OR falling edge on the brake pin.
 */
volatile unsigned long rcThrottleLastEdge = 0;

/**
 * @brief Timestamp of the most recent rising edge on the steering pin.
 */
unsigned long rcSteeringUptime = 0;
/**
 * @brief Timestamp of the most recent rising edge on the steering pin.
 */
unsigned long rcThrottleUptime = 0;

// For averaging the past 5 consecutive steering pulse widths.
int rcSteeringSamples[5] = {0};
int rcSteeringSampleIndex = 0;

// TODO make sure i implemented these interrupt handlers correctly lol

/**
 * @brief This method gets called every time STEERING_PIN switches from low to high or from high to low.
 * It updates rcSteeringLastEdge to current timestamp.
 * If the change is a rising edge, rcSteeringUptime is also updated to the current time.
 * If the change is a falling edge, we are measuring the width of the last high pulse that just ended,
 * and storing that value in v_rcSteeringWidth.
 *
 */
void steeringInterruptHandler()
{
  rcSteeringLastEdge = millis();

  if (digitalRead(STEERING_PIN))
  {
    rcSteeringUptime = micros();
  }
  else
  {
    int width = micros() - rcSteeringUptime;
    if (10 <= width && width <= 2000)
    { // Filtering out blips and long pauses in the signal.
      v_rcSteeringWidth = width;
    }

    if (width > 2000)
    {
      digitalWrite(INTERRUPT_PIN, HIGH);
      digitalWrite(INTERRUPT_PIN, LOW);
    }
  }
}

/**
 * @brief See description for the steeringInterrupHandler method.
 */
void throttleInterruptHandler()
{

  rcThrottleLastEdge = millis();

  if (digitalRead(THROTTLE_PIN))
  { // The pin has changed from low to high, so we're resetting the uptime timer.
    rcThrottleUptime = micros();
  }
  else
  {
    int width = micros() - rcThrottleUptime;
    if (10 <= width && width <= 2000)
    { // Filtering out blips and long pauses in the signal.
      v_rcThrottleWidth = width;
    }

    if (width > 2000)
    {
      digitalWrite(INTERRUPT_PIN, HIGH);
      digitalWrite(INTERRUPT_PIN, LOW);
    }
  }
}

volatile double rosSteeringAngle = 0.0;
volatile double rosBrake = 1.0;

/**
 * @brief Simple wrapper to pull ROS number and store it in rosSteeringAngle.
 *
 * @param cmd_msg idk lol.  i simply copied this from the old version.
 */
void rosSteeringCallback(const std_msgs::Float64 &cmd_msg)
{
  rosSteeringAngle = cmd_msg.data;
}

/**
 * @brief Simple wrapper to pull ROS number and store it in rosBrake.
 *
 * @param cmd_msg idk lol.  i simply copied this from the old version.
 */
void rosBrakeCallback(const std_msgs::Float64 &cmd_msg)
{
  rosBrake = cmd_msg.data;
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Float64> steer("SteerOut_T", rosSteeringCallback);
ros::Subscriber<std_msgs::Float64> brake("BrakeOut_T", rosBrakeCallback);

diagnostic_msgs::DiagnosticStatus teensy_status;
diagnostic_msgs::KeyValue status_values[7];
ros::Publisher debug("TeensyStateIn_T", &teensy_status);

int LEFT_DYNAMIXEL_LIMIT = 1516;
int RIGHT_DYNAMIXEL_LIMIT = 829;
int DYNAMIXEL_CENTER = (LEFT_DYNAMIXEL_LIMIT + RIGHT_DYNAMIXEL_LIMIT) / 2.0;
int DYNAMIXEL_RANGE = LEFT_DYNAMIXEL_LIMIT - RIGHT_DYNAMIXEL_LIMIT;

int LEFT_RC_LIMIT = 1000;
int RIGHT_RC_LIMIT = 1770;
int RC_CENTER = 1450;

/**
 * @brief scales and skews the pulse width to input to the dynamixel
 *
 * @param pulseWidth width of pulse from steering RC pin in milliseconds
 * @return signal to send directly to the dynamixel
 */
int rcToDynamixelWidth(int pulseWidth)
{
  double displacement = abs(pulseWidth - RC_CENTER); // Displacement of the wheel from center.

  // Skewing and scaling based on if the RC pulse is right from center or left from center
  if (pulseWidth < RC_CENTER) { // Left: (0, 1]
    displacement /= RC_CENTER - LEFT_RC_LIMIT;
  } else if (pulseWidth > RC_CENTER) { // Right: [-1, 0)
    displacement /= RC_CENTER - RIGHT_RC_LIMIT;
  }
  
  // Quadratic steering scale
  // (known to the programmers of Saints Robotics as "odd square")
  displacement *= abs(displacement);

  Serial.println(displacement);

  // Translating [-1, 1] to Dynamixel units

  displacement = DYNAMIXEL_CENTER + displacement * DYNAMIXEL_RANGE * 0.5;

  return displacement;
}

enum class BuggyState
{
  ARMED,
  MOVING
};
BuggyState buggyState = BuggyState::ARMED;
/**
 * @brief Keeps count of how long the buggy has been armed.
 */
int buggyStateTimer = 0;


/**
 * @brief TODO this should be cleaned up eventually
 */
void calibrateSteering()
{

  int a = 0;

  uint16_t startPos = -1;
  a = motor->currentPosition(startPos);
  if (startPos == -1)
  {
    return;
  }

  int targetPos = startPos;
  uint16_t currentPos = startPos;

  // Calibrate right limit
  while (currentPos - targetPos < 100)
  {
    Serial.print("right ");
    a = motor->goalPosition(targetPos);

    a = motor->currentPosition(currentPos);

    targetPos -= 5;
    delay(50);
  }
  RIGHT_DYNAMIXEL_LIMIT = currentPos;

  targetPos = startPos;
  currentPos = startPos;
  Serial.println();
    
  a = motor->goalPosition(targetPos);
  delay(2000);
  
  // Calibrate left limit
  while (targetPos - currentPos < 100)
  {
    Serial.print("left ");
    a = motor->goalPosition(targetPos);


    a = motor->currentPosition(currentPos);

    targetPos += 5;
    delay(50);
  }
  LEFT_DYNAMIXEL_LIMIT = currentPos;


}

void setup()
{
  Serial.begin(115200);

  pinMode(STEERING_PIN, INPUT);
  pinMode(THROTTLE_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(STEERING_PIN), steeringInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(THROTTLE_PIN), throttleInterruptHandler, CHANGE);

  nh.getHardware()->setBaud(ROS_BAUD);
  nh.initNode();
  nh.subscribe(steer);
  nh.subscribe(brake);
  nh.advertise(debug);

  pinMode(BRAKE_RELAY_PIN, OUTPUT);
  digitalWrite(BRAKE_RELAY_PIN, LOW);
  pinMode(INTERRUPT_PIN, OUTPUT);

  DynamixelInterface *dInterface = new DynamixelInterface(DYNAMIXEL_SERIAL, DYNAMIXEL_RXEN, DYNAMIXEL_TXEN, DirPinMode::ReadHiWriteLo); // Stream
  dInterface->begin(1000000, 50);                // baudrate, timeout
  motor = new DynamixelMotor(*dInterface, 5); // Interface , ID

  motor->init(); // This will get the returnStatusLevel of the servo
  Serial.printf("Status return level = %u\n", motor->statusReturnLevel());
  motor->statusReturnLevel(1);
  Serial.printf("new return level = %u\n", motor->statusReturnLevel());
  // Status packet delay = 20us
  motor->write(5, (byte)250);

  motor->enableTorque(false);
  motor->jointMode(1, 0xFFF);
  motor->enableTorque();
  
  // calibrateSteering();
  // Serial.print("Left limit is ");
  // Serial.println(LEFT_DYNAMIXEL_LIMIT);
  // Serial.print("Right limit is ");
  // Serial.println(RIGHT_DYNAMIXEL_LIMIT);

  motor->enableTorque(false);
  //motor->jointMode(RIGHT_DYNAMIXEL_LIMIT, LEFT_DYNAMIXEL_LIMIT); // Set the angular limits of the servo. Set to [min, max] by default
  motor->enableTorque();
}

void loop()
{
  int rcSteeringWidth = v_rcSteeringWidth;
  int rcThrottleWidth = v_rcThrottleWidth;

  bool rcSteeringTimeout = (millis() - rcSteeringLastEdge) >= 50.0;
  bool rcThrottleTimeout = (millis() - rcThrottleLastEdge) >= 50.0;

  // Capture most recent RC steering sample.
  rcSteeringSampleIndex++;
  rcSteeringSampleIndex %= 5;
  rcSteeringSamples[rcSteeringSampleIndex] = rcSteeringWidth;

  // Calculating the average of the past 5 RC steering samples.
  float rcSteeringAvg = 0.0;
  for (int i = 0; i < 5; i++)
  {
    rcSteeringAvg += rcSteeringSamples[i];
  }
  rcSteeringAvg /= 5;

  // Determining the state of the throttle trigger on the RC controller.
  bool autoMode = (1750 <= rcThrottleWidth);
  bool teleMode = (rcThrottleWidth <= 1250);
  bool neutralMode = !autoMode && !teleMode;

  switch (buggyState)
  {
  case BuggyState::ARMED:
  {
    if ((teleMode || autoMode) && (millis() - buggyStateTimer > 1000))
    {
      buggyStateTimer = millis();
      buggyState = BuggyState::MOVING;
    }
    else if (!(teleMode || autoMode))
    {
      buggyStateTimer = millis();
    }
    break;
  }

  default:
  {
    if (neutralMode && millis() - buggyStateTimer > 1000)
    {
      buggyStateTimer = millis();
      buggyState = BuggyState::ARMED;
    }
    else if (!neutralMode)
    {
      buggyStateTimer = millis();
    }
  }
  }

  // Controlling hardware thru RC.
  float steeringCommand = rcToDynamixelWidth(rcSteeringAvg);
  bool brakeCommand = buggyState == BuggyState::MOVING;

  // If auton is enabled, it will set inputs to ROS inputs.
  // if (buggyState == BuggyState::MOVING && autoMode)
  // {
  //   steeringCommand = DYNAMIXEL_CENTER + DYNAMIXEL_RANGE * rosSteeringAngle / 0.088 * 0.5;
  //   brakeCommand = abs(rosBrake - 1.0) < 1e-6;
  // }


  motor->goalPosition(steeringCommand);
  digitalWrite(BRAKE_RELAY_PIN, brakeCommand);

  delay(1);
  nh.spinOnce();
}
