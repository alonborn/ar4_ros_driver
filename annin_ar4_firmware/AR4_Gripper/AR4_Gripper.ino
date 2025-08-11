#include <Servo.h>

Servo myServo;

const int servoPin = 5;

const int minAngle = 6;
const int maxAngle = 40;
const int angleStep = 1;

int stepDelay = 5; // Default delay (ms)

String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(9600);
  myServo.attach(servoPin);
  inputString.reserve(10);
  moveServoTo(minAngle);
}

void loop() {
  if (stringComplete) {
    handleCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n' || inChar == '\r') {
      if (inputString.length() > 0) {
        stringComplete = true;
      }
    } else {
      inputString += inChar;
    }
  }
}

void handleCommand(const String& cmd) {
  if (cmd == "close") {
    moveServoTo(maxAngle);
  } else if (cmd == "open") {
    moveServoTo(minAngle);
  } else if (cmd.startsWith("s")) {
    int newDelay = cmd.substring(1).toInt();
    if (newDelay > 0) {
      stepDelay = newDelay;
    }
  }
}

void moveServoTo(int targetAngle) {
  int currentAngle = myServo.read();
  int step = (targetAngle > currentAngle) ? angleStep : -angleStep;

  for (int angle = currentAngle; angle != targetAngle; angle += step) {
    myServo.write(angle);
    delay(stepDelay);
  }
  myServo.write(targetAngle); // Ensure exact position
}
