#include <TaskScheduler.h>
#include <IRremote.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <NewPing.h>

// Callback methods prototypes
void tIRrecvCheck();
void tNewPingCheck();
void tBumpCheck();
void tServoUpDown();

// ---------------------------------------------------------------------------
//Tasks
// ---------------------------------------------------------------------------
#define MOVE_10 1000
Task tIRrecv(500, TASK_FOREVER, &tIRrecvCheck);
Task tNewPing(300, TASK_FOREVER, &tNewPingCheck);
Task tBump(300, TASK_FOREVER, &tBumpCheck);
Task tServoStraight(MOVE_10, TASK_FOREVER, &tServoUpDown);

Scheduler runner;

// ---------------------------------------------------------------------------
// Hardware & Constants
// ---------------------------------------------------------------------------

// Servo
#define SERVOSTOP 90

#define LFFAST 180
#define LBFAST 0
#define RFFAST 70
#define RBFAST 180

Servo leftServo;
#define leftServoPin 10
Servo rightServo;
#define rightServoPin 9

// IRrecv
int RECV_PIN = 8;
#define CODE_UP 0x810
#define CODE_DOWN 0xE10
#define CODE_LEFT 0xC10
#define CODE_RIGHT 0xA10
#define CODE_STOP 0x210
#define CODE_TURN 0x111

int servoState = CODE_STOP;
IRrecv irrecv(RECV_PIN);
decode_results results;

// ---------------------------------------------------------------------------
// Ultrasonic Sensor
// ---------------------------------------------------------------------------
#define TRIGGER_PIN 12
#define ECHO_PIN 11
#define MAX_DISTANCE 300
#define STOP_DISTANCE 5
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// ---------------------------------------------------------------------------
// Bumper Sensor
// ---------------------------------------------------------------------------
#define LeftBumpPin 5
#define RightBumpPin 4
int LED_right = 13;
int LED_left = 11;
// Variables
volatile byte bumperLeftState = LOW;
volatile byte bumperRightState = LOW;

boolean moving = false;

void tIRrecvCheck() {
  if (irrecv.decode(&results)) {
    servoState = results.value;
  } else {
    servoState = CODE_STOP;
  }
  irrecv.resume();
}

void tNewPingCheck() {
  int distance = sonar.ping_cm();
  if (distance <= STOP_DISTANCE) {
    servoState = CODE_STOP;
    servoStop();
  }
}

void tBumpCheck() {
  if (digitalRead(LeftBumpPin) == HIGH || digitalRead(RightBumpPin) == HIGH) {
    Serial.println("tBumpCheck");
    servoState = CODE_DOWN;
  }
}

void tServoUpDown() {
  Serial.print("tServoUpDown: [");
  Serial.print(moving);
  Serial.print("] [");
  Serial.print(servoState);
  Serial.println("]");
  switch (servoState) {
    case CODE_STOP:
      Serial.print("CODE_STOP: ");
      Serial.println(moving);
      if (moving) {
        servoStop();
      }
      break;
    case CODE_UP:
      Serial.println("CODE_UP");
      moving = true;
      leftServo.attach(leftServoPin);
      rightServo.attach(rightServoPin);
      leftServo.write(LFFAST);
      rightServo.write(RFFAST);
      break;
    case CODE_DOWN:
      Serial.println("CODE_DOWN");
      moving = true;
      leftServo.attach(leftServoPin);
      rightServo.attach(rightServoPin);
      leftServo.write(LBFAST);
      rightServo.write(RBFAST);
      break;
    case CODE_LEFT:
      Serial.println("CODE_LEFT");
      moving = true;
      leftServo.attach(leftServoPin);
      rightServo.attach(rightServoPin);
      leftServo.write(LBFAST);
      rightServo.write(RFFAST);
      break;
    case CODE_RIGHT:
      Serial.println("CODE_RIGHT");
      moving = true;
      leftServo.attach(leftServoPin);
      rightServo.attach(rightServoPin);
      leftServo.write(RBFAST);
      rightServo.write(LFFAST);
      break;
  }
}

void servoStop() {
  Serial.println("servoStop");
  moving = false;
  rightServo.write(SERVOSTOP);
  rightServo.detach();
  leftServo.write(SERVOSTOP);
  leftServo.detach();
}

void setup() {
  Serial.begin(9600);
  //  irrecv.enableIRIn(); // Start the receiver

  irrecv.enableIRIn(); // Start the receiver

  runner.init();

  runner.addTask(tIRrecv);
  tIRrecv.enable();
  runner.addTask(tNewPing);
  tNewPing.enable();
  runner.addTask(tBump);
  tBump.enable();
  runner.addTask(tServoStraight);
  tServoStraight.enable();
}

void loop() {
  // put your main code here, to run repeatedly:
  runner.execute();
}
