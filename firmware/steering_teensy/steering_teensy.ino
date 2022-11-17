#include "Dynamixel.h"
#include "DynamixelInterface.h"
#include "DynamixelMotor.h"

volatile int pin_2_width = 1500;
volatile int pin_3_width = 1500;

#define TIMEOUT_CYCLES 10

volatile int pin_2_timeout = TIMEOUT_CYCLES;
volatile int pin_3_timeout = TIMEOUT_CYCLES;

void setup() {
  Serial.begin(115200);

  pinMode(2, INPUT);
  pinMode(3, INPUT);

  pinMode(25, OUTPUT);

  digitalWrite(25, LOW);
  
  attachInterrupt(digitalPinToInterrupt(2), pin_2_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), pin_3_change, CHANGE);
}

int i = 0;

double t = 0.0;

void loop()
{ 
  /*while (1) {
  digitalWrite(25, i);
  i = !i;
  delay(5000);
  }*/
  
  Serial.println("foobar");
  
  DynamixelInterface dInterface(Serial7, 27, DirPinMode::ReadHiWriteLo); // Stream
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

  while (true)
  {
    t += 5.0;

    if (pin_2_timeout > 0) { --pin_2_timeout; } else { pin_2_width = 0; }
    if (pin_3_timeout > 0) { --pin_3_timeout; } else { pin_3_width = 0; }
      
    int p2width = pin_2_width;
    if (p2width < 1000) { p2width = 1000; }
    if (p2width > 2000) { p2width = 2000; }
    
    float range = (pin_2_width - 1000.0) / 800.0;

    //uint16_t currentPos = 0;
    //motor.read(36, currentPos);

    //uint16_t currentSpeed;
    //motor.read(38, currentSpeed);

    if (++i == 10) {
      i = 0;
      //Serial.printf("speed is %d\n", currentSpeed);

      //Serial.printf("pin width is %d\n", pin_3_width);
    }

    double angle = sin(t / 100.0);
    
    //motor.goalPositionDegree(90 - (range - 0.5) * 2.0 * 60.0);
    //motor.goalPositionDegree(90 + angle * 45.0);

    if (pin_3_width > 1500.0) {
        digitalWrite(25, HIGH);
    } else {
        digitalWrite(25, LOW);
    }

    delay(10);
  }
}

void pin_2_change() {
  pin_2_timeout = TIMEOUT_CYCLES;
  
  static int start_time = 0;
  if (digitalRead(2)) {
    start_time = micros();
  } else {
    pin_2_width = micros() - start_time;
  }
}


void pin_3_change() {
  pin_3_timeout = TIMEOUT_CYCLES;
  
  static int start_time = 0;
  if (digitalRead(3)) {
    start_time = micros();
  } else {
    pin_3_width = micros() - start_time;
  }
}
