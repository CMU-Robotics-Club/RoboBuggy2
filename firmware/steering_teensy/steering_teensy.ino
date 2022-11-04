#if 1

#include "Dynamixel.h"
#include "DynamixelInterface.h"
#include "DynamixelMotor.h"

volatile int pin_2_width = 1500;

void setup() {
  Serial.begin(115200);

  pinMode(2, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(2), pin_2_change, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(3), pin_3_change, CHANGE);
}

int i = 0;

double t = 0.0;

void loop()
{

  
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
      
    int p2width = pin_2_width;
    if (p2width < 1000) { p2width = 1000; }
    if (p2width > 2000) { p2width = 2000; }

    
    float range = (pin_2_width - 1000.0) / 800.0;

    //uint16_t currentPos = 0;
    //motor.read(36, currentPos);

    uint16_t currentSpeed;
    motor.read(38, currentSpeed);

    if (++i == 10) {
      i = 0;
      Serial.printf("speed is %d\n", currentSpeed);

      Serial.printf("pin width is %d\n", p2width);
    }

    double angle = sin(t / 100.0);
    
    motor.goalPositionDegree(90 - (range - 0.5) * 2.0 * 60.0);
    //motor.goalPositionDegree(90 + angle * 45.0);

    delay(10);
  }
}

void pin_2_change() {
  static int start_time = 0;
  if (digitalRead(2)) {
    start_time = micros();
  } else {
    pin_2_width = micros() - start_time;
  }
}

#else

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  Serial7.begin(9600);

  digitalWrite(12, HIGH);

  delay(100);

  byte data2[] = {
    0xFF, 0xFF,
    0x05, // ID
    0x04, // LEN
    0x03, // WRITE
    0x18, // ADDR
    0x00, // ON
    (byte)(~(0x05 + 0x04 + 0x03 + 0x18 + 0x00))
  };

  Serial7.write(data2, sizeof(data2));

  delay(100);
}

uint16_t pos = 0;

int i = 0;

void loop() {
  /*byte data3[] = {
    0xFF, 0xFF,
    0x05, // ID
    0x04, // LEN
    0x03, // WRITE
    0x18, // ADDR
    0x00, // ON
    (byte)(~(0x05 + 0x04 + 0x03 + 0x18 + 0x00))
  };

  Serial1.write(data3, sizeof(data3));

  delay(100);*/
  
  /*byte data1[] = {
    0xFF, 0xFF,
    0x05, // ID
    0x02, // LEN
    0x01, // PING
    (byte)(~(0x05 + 0x02 + 0x01))
  };

  Serial1.write(data1, sizeof(data1));

  delay(100);*/

  /*byte data2[] = {
    0xFF, 0xFF,
    0x05, // ID
    0x04, // LEN
    0x02, // READ
    0x2B, // ADDR
    0x01, // LEN
    (byte)(~(0x05 + 0x04 + 0x02 + 0x2B + 0x01))
  };

  Serial1.write(data2, sizeof(data2));

  delay(100);*/ 

  pos += 100;

  pos %= 0xFFF;
  
  byte data2[] = {
    0xFF, 0xFF,
    0x05, // ID
    0x05, // LEN
    0x03, // WRITE
    0x1E, // ADDR
    (byte)(pos & 0xFF), // DATA LO
    (byte)(pos >> 8), // DATA HI
    (byte)(~(0x05 + 0x05 + 0x03 + 0x1E + (byte)(pos & 0xFF) + (byte)(pos >> 8)))
  };

  Serial7.write(data2, sizeof(data2));

  delay(100);
}
#endif
