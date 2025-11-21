#include <AccelStepper.h>
#include <Bounce2.h>
#include <Encoder.h>
#include <avr/pgmspace.h>
#include <math.h>
#include <EEPROM.h>  
#include <Servo.h>

#include <map>

// Firmware version
const char* VERSION = "2.0.0";

// Model of the AR4, i.e. mk1, mk2, mk3
String MODEL = "";

///////////////////////////////////////////////////////////////////////////////
// Physical Params
///////////////////////////////////////////////////////////////////////////////

const int ESTOP_PIN = 39;
const int STEP_PINS[] = {0, 2, 4, 6, 8, 10};
const int DIR_PINS[] = {1, 3, 5, 7, 9, 11};
const int LIMIT_PINS[] = {26, 27, 28, 29, 30, 31};

const int servoPin = 40;
Servo myServo;

const int minServoAngle = 30;
const int maxServoAngle = 40;
const int ServoAngleStep = 1;
int ServoStepDelay = 5; // Default delay (ms)


std::map<String, const float*> MOTOR_STEPS_PER_DEG;
const float MOTOR_STEPS_PER_DEG_MK1[] = {44.44444444, 55.55555556, 55.55555556,
                                         42.72664356, 21.86024888, 22.22222222};
const float MOTOR_STEPS_PER_DEG_MK2[] = {44.44444444, 55.55555556, 55.55555556,
                                         49.77777777, 21.86024888, 22.22222222};
const float MOTOR_STEPS_PER_DEG_MK3[] = {44.44444444, 55.55555556, 55.55555556,
                                         49.77777777, 21.86024888, 22.22222222};

const int MOTOR_STEPS_PER_REV[] = {400, 400, 400, 400, 800, 400};

double ZERO_OFFSET_DEG[6] = {0};


// set encoder pins
Encoder encPos[6] = {Encoder(15, 14), Encoder(16, 17), Encoder(18, 19),
                     Encoder(20, 21), Encoder(22, 23), Encoder(24, 25)};
// +1 if encoder direction matches motor direction, -1 otherwise
int ENC_DIR[] = {-1, 1, 1, 1, 1, 1};
// +1 if encoder max value is at the minimum joint angle, 0 otherwise
int ENC_MAX_AT_ANGLE_MIN[] = {1, 0, 1, 0, 0, 1};
// motor steps * ENC_MULT = encoder steps
const float ENC_MULT[] = {10, 10, 10, 10, 5, 10};

// define axis limits in degrees, for calibration
std::map<String, const int*> JOINT_LIMIT_MIN;
int JOINT_LIMIT_MIN_MK1[] = {-170, -42, -89, -165, -105, -155};
int JOINT_LIMIT_MIN_MK2[] = {-170, -42, -89, -165, -105, -155};
int JOINT_LIMIT_MIN_MK3[] = {-170, -42, -89, -180, -105, -180};
std::map<String, const int*> JOINT_LIMIT_MAX;
int JOINT_LIMIT_MAX_MK1[] = {170, 90, 52, 165, 105, 155};
int JOINT_LIMIT_MAX_MK2[] = {170, 90, 52, 165, 105, 155};
int JOINT_LIMIT_MAX_MK3[] = {170, 90, 52, 180, 105, 180};

///////////////////////////////////////////////////////////////////////////////
// ROS Driver Params
///////////////////////////////////////////////////////////////////////////////

// roughly equals 0, 0, 0, 0, 0, 0 degrees without any user-defined offsets.
std::map<String, int*> REST_MOTOR_STEPS;

int CalOffset[] = {0,0,0,0,0,0};


int REST_MOTOR_STEPS_MK1[] = {7555, 2333, 4944, 7049, 2295, 3431};
int REST_MOTOR_STEPS_MK2[] = {7555, 2333, 4944, 7049, 2295, 3431};
//int REST_MOTOR_STEPS_MK3[] = {7555, 2333, 4944, 8960, 2295, 4000};
int REST_MOTOR_STEPS_MK3[] = {7555, 2310, 5290, 8985, 2213, 4230};

enum SM { STATE_TRAJ, STATE_ERR };
SM STATE = STATE_TRAJ;

const int NUM_JOINTS = 6;
AccelStepper stepperJoints[NUM_JOINTS];
Bounce2::Button limitSwitches[NUM_JOINTS];
const int DEBOUCE_INTERVAL = 10;  // ms

// calibration settings
const int LIMIT_SWITCH_HIGH[] = {
    1, 1, 1, 1, 1, 1};  // to account for both NC and NO limit switches
const int CAL_DIR[] = {-1, -1, 1,
                       -1, -1, 1};  // joint rotation direction to limit switch
const int CAL_SPEED = 500;  //500          // motor steps per second
const int CAL_SPEED_MULT[] = {
    1, 1, 1, 2, 1, 1};  // multiplier to account for motor steps/rev
// num of encoder steps in range of motion of joint
int ENC_RANGE_STEPS[NUM_JOINTS];

// speed and acceleration settings
float JOINT_MAX_SPEED[] = {60.0, 60.0, 60.0, 60.0, 60.0, 60.0};  // deg/s
float JOINT_MAX_ACCEL[] = {30.0, 30.0, 30.0, 30.0, 30.0, 30.0};  // deg/s^2
char JOINT_NAMES[] = {'A', 'B', 'C', 'D', 'E', 'F'};

float JOINT_MAX_SPEED_ORIGINAL[NUM_JOINTS];
float globalSpeedScale = 1.0;


bool isJointAtPosition[6];

void LoadRestStepsFromEEPROM() {
  for (int i = 0; i < NUM_JOINTS; ++i) {
    int val = 0;
    EEPROM.get(i * sizeof(int), val);
    REST_MOTOR_STEPS["mk3"][i] = val;
    Serial8.print("Joint ");
    Serial8.print(i);
    Serial8.print(": ");
    Serial8.println(val); 
    // Serial.println("Joint " + String(i) + ": " + String(val));  // for debugging
  }
  Serial8.println("EEPROM home positions loaded");
}


void ResetJointAtPosition() {
  for (int i = 0; i < NUM_JOINTS; ++i) {
    isJointAtPosition[i] = false;
  }
}

bool estop_pressed = false;

void estopPressed() { estop_pressed = true; }

void resetEstop() {
  // if ESTOP button is pressed still, do not reset the flag!
  if (digitalRead(ESTOP_PIN) == LOW) {
    return;
  }

  // reset any previously set MT commands
  for (int i = 0; i < NUM_JOINTS; ++i) {
    // NOTE: This may seem redundant but is the only permitted way to set
    // _stepInterval and _n to 0, which is required to avoid a jerk resume
    // when Estop is reset after interruption of an accelerated motion
    stepperJoints[i].setCurrentPosition(stepperJoints[i].currentPosition());
    stepperJoints[i].setSpeed(0);
  }

  estop_pressed = false;
}

void PrintRestMotorStepOffsets()
{
  for (int i = 0 ; i < NUM_JOINTS ; i++)  {
    Serial8.print (REST_MOTOR_STEPS[MODEL][i]);
    Serial8.print (" ");
  }
  Serial8.println("");
}


bool safeRun(AccelStepper& stepperJoint) {
  if (estop_pressed) {
     return false;
     Serial8.println("estop pressed");
  }

  return stepperJoint.run();
}


bool safeRunForNudge(AccelStepper& s) {
  if (estop_pressed) {
    Serial8.println("ER: E-Stop pressed");
    // Treat as "done" so outer loop can exit; handle the error outside.
    return true;
  }

  if (s.distanceToGo() != 0) {
    Serial8.print("Running joint, distance to go: ");
    Serial8.print(s.distanceToGo());
    Serial8.print("  target: ");
    Serial8.println(s.targetPosition());  // target in steps

    s.run();              // advance toward target
    return false;         // not done yet
  }
  return true;            // target reached
}



bool safeRunSpeed(AccelStepper& stepperJoint) {
  if (estop_pressed) return false;
  return stepperJoint.runSpeed();
}

void ManualHomeOffset(String inData) {
  int steps[NUM_JOINTS] = {0};

  // --- 1) Remember current speeds (current speed & maxSpeed) ---
  float prevSpeed[NUM_JOINTS];
  float prevMaxSpeed[NUM_JOINTS];
  for (int i = 0; i < NUM_JOINTS; ++i) {
    prevSpeed[i]     = stepperJoints[i].speed();     // current effective speed (steps/s)
    prevMaxSpeed[i]  = stepperJoints[i].maxSpeed();  // max allowed speed (steps/s)
  }

  // --- 2) Set speed to max (raise maxSpeed to your configured maximum) ---
  for (int i = 0; i < NUM_JOINTS; ++i) {
    // Convert deg/s limit to steps/s
    float maxStepsPerSec = JOINT_MAX_SPEED[i] * MOTOR_STEPS_PER_DEG[MODEL][i];
    if (maxStepsPerSec < 100.0f) maxStepsPerSec = 100.1835790f; // floor to avoid too-low limits
    stepperJoints[i].setMaxSpeed(maxStepsPerSec);
    // (Optional) nudge current speed toward max in the intended direction; run() ignores setSpeed,
    // but this won’t hurt and can help if you switch to runSpeedToPosition().
    stepperJoints[i].setSpeed( (steps[i] >= 0 ? +1.0f : -1.0f) * maxStepsPerSec );
  }

  // Parse the 6 values after "HM"
  int idx = 0;
  char* token = strtok(inData.c_str() + 2, " ");
  while (token != NULL && idx < NUM_JOINTS) {
    steps[idx++] = atoi(token);
    token = strtok(NULL, " ");
  }

  // Move specified joints and zero their stepper position
  for (int i = 0; i < NUM_JOINTS; ++i) {
    if (steps[i] != 0) {
      stepperJoints[i].setCurrentPosition(0);  // avoid skipping
      stepperJoints[i].move(steps[i]);
      Serial8.print("Moving joint ");
      Serial8.print(i);
      Serial8.print(" by ");
      Serial8.print(steps[i]);
      Serial8.println(" steps.");
    }
  }

  // Run all motions until completion
  bool allDone = false;
  while (!allDone) {
    allDone = true;
    // Serial8.println("Running stepper joints...");
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (steps[i] != 0 && !safeRunForNudge(stepperJoints[i])) {
        allDone = false;
      }
    }
  }

  Serial8.println("Movement Done");

  // --- 3) Revert speeds to the memorized values ---
  for (int i = 0; i < NUM_JOINTS; ++i) {
    stepperJoints[i].setMaxSpeed(prevMaxSpeed[i]);
    stepperJoints[i].setSpeed(prevSpeed[i]);
  }

  delay(100);  // let motors settle

  Serial8.println("encoders are set to match home positions");
  // Update and save new rest steps
  for (int i = 0; i < NUM_JOINTS; ++i) {
    REST_MOTOR_STEPS[MODEL][i] += steps[i];  // Apply delta to existing rest value
    EEPROM.put(i * sizeof(int), REST_MOTOR_STEPS[MODEL][i]);

    Serial8.print("Joint ");
    Serial8.print(i);
    Serial8.print(": Added ");
    Serial8.print(steps[i]);
    Serial8.print(" → New rest = ");
    Serial8.println(REST_MOTOR_STEPS[MODEL][i]);
  }
  
  // Serial8.println("HM: Manual move complete. New position set as home.");
  Serial8.println("EEPROM updated.");
  Serial8.println("Updating Calibration Offsets");
  UpdateCalibrationOffsets(NULL);
  Serial8.println("Encoders and Limit Switches:");
  PrintOutEncodersAndLimitSwitchS8();
  // Serial.println("OK");
}




void PrintEEPROMRestSteps() {
  Serial8.println("📤 EEPROM-stored REST_MOTOR_STEPS:");

  for (int i = 0; i < NUM_JOINTS; ++i) {
    int value;
    EEPROM.get(i * sizeof(int), value);
    Serial8.print("Joint ");
    Serial8.print(i);
    Serial8.print(": ");
    Serial8.println(value);
  }

  Serial8.println("✅ Done reading EEPROM.");
}


void SaveRestStepsToEEPROM() {
  Serial.println("Saving REST_MOTOR_STEPS to EEPROM...");
  
  for (int i = 0; i < NUM_JOINTS; ++i) {
    int value = REST_MOTOR_STEPS[MODEL][i];
    EEPROM.put(i * sizeof(int), value);
  }

  Serial8.println("Saved values:");
  for (int i = 0; i < NUM_JOINTS; ++i) {
    int saved;
    EEPROM.get(i * sizeof(int), saved);
    Serial8.print("Joint ");
    Serial8.print(i);
    Serial8.print(": ");
    Serial8.println(saved);
  }

  Serial8.println("✅ EEPROM save complete.");
}


void ClearRestStepsEEPROM() {
  for (int i = 0; i < NUM_JOINTS; ++i) {
    int zero = 0;
    EEPROM.put(i * sizeof(int), zero);
  }
  Serial8.println("EEPROM cleared.");
}

void ApplyRestMotorStepOffset(String data) {
    if (!data.startsWith("SR ")) {
        Serial.println("Error: Invalid command format:" + data);
        return;
    }
    Serial8.println(data);
    int stepValues[NUM_JOINTS] = {0};
    int index = 0;

    char* token = strtok(data.c_str() + 3, " ");  // Skip "SR "
    while (token != NULL && index < NUM_JOINTS) {
        stepValues[index] = atoi(token);
        token = strtok(NULL, " ");
        index++;
    }

    if (index == NUM_JOINTS) {  // Ensure all values are received
        for (int i = 0; i < NUM_JOINTS; i++) {
            REST_MOTOR_STEPS[MODEL][i] = stepValues[i];
            Serial8.println ("Setting " + String (i) + ":" + String(REST_MOTOR_STEPS[MODEL][i]));
        }
        Serial.println("Steps updated successfully");
    } else {
        Serial.println("Error: Incorrect number of values received");
    }


}

void setup() {
  Serial.begin(9600);
  Serial8.begin(9600);

  while (!Serial8) {
    ; // Wait for Serial port to connect
  }
  myServo.attach(servoPin);
  moveServoTo(70);
  Serial8.println("------------Setup Started---------------");
  MOTOR_STEPS_PER_DEG["mk1"] = MOTOR_STEPS_PER_DEG_MK1;
  MOTOR_STEPS_PER_DEG["mk2"] = MOTOR_STEPS_PER_DEG_MK2;
  MOTOR_STEPS_PER_DEG["mk3"] = MOTOR_STEPS_PER_DEG_MK3;

  JOINT_LIMIT_MIN["mk1"] = JOINT_LIMIT_MIN_MK1;
  JOINT_LIMIT_MIN["mk2"] = JOINT_LIMIT_MIN_MK2;
  JOINT_LIMIT_MIN["mk3"] = JOINT_LIMIT_MIN_MK3;

  JOINT_LIMIT_MAX["mk1"] = JOINT_LIMIT_MAX_MK1;
  JOINT_LIMIT_MAX["mk2"] = JOINT_LIMIT_MAX_MK2;
  JOINT_LIMIT_MAX["mk3"] = JOINT_LIMIT_MAX_MK3;

  REST_MOTOR_STEPS["mk1"] = REST_MOTOR_STEPS_MK1;
  REST_MOTOR_STEPS["mk2"] = REST_MOTOR_STEPS_MK2;
  REST_MOTOR_STEPS["mk3"] = REST_MOTOR_STEPS_MK3;

  for (int i = 0; i < NUM_JOINTS; ++i) {
    pinMode(STEP_PINS[i], OUTPUT);
    pinMode(DIR_PINS[i], OUTPUT);
    pinMode(LIMIT_PINS[i], INPUT);
  }

  LoadRestStepsFromEEPROM();

  for (int i = 0; i < NUM_JOINTS; ++i) {
    limitSwitches[i] = Bounce2::Button();
    limitSwitches[i].attach(LIMIT_PINS[i], INPUT);
    limitSwitches[i].interval(DEBOUCE_INTERVAL);
    limitSwitches[i].setPressedState(LIMIT_SWITCH_HIGH[i]);
  }

  pinMode(ESTOP_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), estopPressed, FALLING);

  // Store original max speeds (deg/s)
  for (int i = 0; i < NUM_JOINTS; i++) {
      JOINT_MAX_SPEED_ORIGINAL[i] = JOINT_MAX_SPEED[i];
  }

  delay (200);

  Serial8.println("------------Setup Ended---------------");
  Serial8.flush();
}

void setupSteppersMK1() {
  // initialise AccelStepper instance
  for (int i = 0; i < NUM_JOINTS; ++i) {
    stepperJoints[i] = AccelStepper(1, STEP_PINS[i], DIR_PINS[i]);
    stepperJoints[i].setPinsInverted(true, false, false);  // DM542T CW
    stepperJoints[i].setAcceleration(JOINT_MAX_ACCEL[i] *
                                     MOTOR_STEPS_PER_DEG[MODEL][i]);
    stepperJoints[i].setMaxSpeed(JOINT_MAX_SPEED[i] *
                                 MOTOR_STEPS_PER_DEG[MODEL][i]);
    stepperJoints[i].setMinPulseWidth(10);
  }
  stepperJoints[3].setPinsInverted(false, false, false);  // J4 DM320T CCW
}

void setupSteppersMK2() {
  // initialise AccelStepper instance
  for (int i = 0; i < NUM_JOINTS; ++i) {
    stepperJoints[i] = AccelStepper(1, STEP_PINS[i], DIR_PINS[i]);
    stepperJoints[i].setPinsInverted(false, false,
                                     false);  // DM320T / DM332T --> CW
    stepperJoints[i].setAcceleration(JOINT_MAX_ACCEL[i] *
                                     MOTOR_STEPS_PER_DEG[MODEL][i]);
    stepperJoints[i].setMaxSpeed(JOINT_MAX_SPEED[i] *
                                 MOTOR_STEPS_PER_DEG[MODEL][i]);
    stepperJoints[i].setMinPulseWidth(10);
  }
}


unsigned long lastLogTime = 0;  // Tracks the last log timestamp
unsigned long intervalMs = 100;

void logThrottled(const String& message) {
  return;
  unsigned long currentTime = millis();
  if (currentTime - lastLogTime >= intervalMs || message[0] == "*") {
      Serial8.println(message);
      lastLogTime = currentTime;
  }
}


void SendToROS(String data) {
  Serial.println(data);
  logThrottled(data);
}

void setupSteppersMK3() {
  // initialise AccelStepper instance
  for (int i = 0; i < NUM_JOINTS; ++i) {
    stepperJoints[i] = AccelStepper(1, STEP_PINS[i], DIR_PINS[i]);
    stepperJoints[i].setPinsInverted(false, false,
                                     false);  // DM320T / DM332T --> CW
    stepperJoints[i].setAcceleration(JOINT_MAX_ACCEL[i] *
                                     MOTOR_STEPS_PER_DEG[MODEL][i]);
    stepperJoints[i].setMaxSpeed(JOINT_MAX_SPEED[i] *
                                 MOTOR_STEPS_PER_DEG[MODEL][i]);
    stepperJoints[i].setMinPulseWidth(10);
  }
}

// initialize stepper motors and constants based on the model. Also verifies
// that the software version matches the firmware version
// bool initStateTraj2(String inData) {
//   // parse initialisation message
//   int idxVersion = inData.indexOf('A');
//   int idxModel = inData.indexOf('B');
//   String softwareVersion = "2.0.0";//inData.substring(idxVersion + 1, idxModel);
//   int versionMatches = (softwareVersion == VERSION);

//   String model = "mk3";//nData.substring(idxModel + 1, inData.length() - 1);
//   int modelMatches = false;
//   if (model == "mk1" || model == "mk2" || model == "mk3") {
//     modelMatches = true;
//     MODEL = model;

//     for (int i = 0; i < NUM_JOINTS; ++i) {
//       int joint_range = JOINT_LIMIT_MAX[MODEL][i] - JOINT_LIMIT_MIN[MODEL][i];
//       ENC_RANGE_STEPS[i] = static_cast<int>(MOTOR_STEPS_PER_DEG[MODEL][i] *
//                                             joint_range * ENC_MULT[i]);
//     }

//     if (model == "mk1") {
//       setupSteppersMK1();
//     } else if (model == "mk2") {
//       setupSteppersMK2();
//     } else if (model == "mk3") {
//       setupSteppersMK3();
//     }
//   }

//   // return acknowledgement with result
//   String msg = String("ST") + "A" + versionMatches + "B" + VERSION + "C" +
//                modelMatches + "D" + MODEL;
//   Serial.println(msg);

//   if (versionMatches && modelMatches) {
//     return true;
//   }
//   return false;
// }

//STA2.0.0Bmk3
// initialize stepper motors and constants based on the model. Also verifies
// that the software version matches the firmware version
bool initStateTraj(String inData) {
  // parse initialisation message
  int idxVersion = inData.indexOf('A');
  int idxModel = inData.indexOf('B');
  String softwareVersion = inData.substring(idxVersion + 1, idxModel);
  int versionMatches = (softwareVersion == VERSION);

  String model = inData.substring(idxModel + 1, inData.length() - 1);
  int modelMatches = false;
  if (model == "mk1" || model == "mk2" || model == "mk3") {
    modelMatches = true;
    MODEL = model;

    for (int i = 0; i < NUM_JOINTS; ++i) {
      int joint_range = JOINT_LIMIT_MAX[MODEL][i] - JOINT_LIMIT_MIN[MODEL][i];
      ENC_RANGE_STEPS[i] = static_cast<int>(MOTOR_STEPS_PER_DEG[MODEL][i] *
                                            joint_range * ENC_MULT[i]);
    }

    if (model == "mk1") {
      setupSteppersMK1();
    } else if (model == "mk2") {
      setupSteppersMK2();
    } else if (model == "mk3") {
      setupSteppersMK3();
    }
  }

  // return acknowledgement with result
  String msg = String("ST") + "A" + versionMatches + "B" + VERSION + "C" +
               modelMatches + "D" + MODEL;
  SendToROS(msg);

  if (versionMatches && modelMatches) {
    return true;
  }
  return false;
}

template <typename T>
int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

void readMotorSteps(int* motorSteps,int * joints = NULL) {
  for (int i = 0; i < NUM_JOINTS; ++i) {
    if (joints != NULL && joints[i] == 0)
      continue;
    motorSteps[i] = encPos[i].read() / ENC_MULT[i];
  }
  //delayMicroseconds(50);
}

static inline bool homesToMin(int i) {
  const double deg_per_step = ENC_DIR[i] / MOTOR_STEPS_PER_DEG[MODEL][i];
  return (deg_per_step * CAL_DIR[i]) < 0; // <0 ⇒ moving toward MIN
}

static inline long encAtMin(int i) {
  return (ENC_MAX_AT_ANGLE_MIN[i] == 1) ? (long)ENC_RANGE_STEPS[i] : 0L;
}
static inline long encAtMax(int i) {
  return (ENC_MAX_AT_ANGLE_MIN[i] == 1) ? 0L : (long)ENC_RANGE_STEPS[i];
}



void SetGlobalSpeedScale(String inData) {
  // Expect command like "SF 0.6"
  float scale = inData.substring(2).toFloat();

  if (scale <= 0 || scale > 2.0) {
    Serial8.println("ER: invalid scale (allowed 0 < scale ≤ 2.0)");
    return;
  }

  globalSpeedScale = scale;

  // Apply to all joints
  for (int i = 0; i < NUM_JOINTS; i++) {
    JOINT_MAX_SPEED[i] = JOINT_MAX_SPEED_ORIGINAL[i] * globalSpeedScale;
    stepperJoints[i].setMaxSpeed(JOINT_MAX_SPEED[i] * MOTOR_STEPS_PER_DEG[MODEL][i]);
  }

  Serial8.print("SF: Global speed scale set to ");
  Serial8.println(globalSpeedScale);
  Serial8.print("SF: New joint max speeds (deg/s): ");
  for (int i = 0; i < NUM_JOINTS; i++) {
    Serial8.print(JOINT_MAX_SPEED[i]);
    Serial8.print(" ");
  }
  Serial8.println();
}


void WriteEncodersAtHomingLimit(const int* calJoints) {
  for (int i = 0; i < NUM_JOINTS; ++i) if (!calJoints || calJoints[i]) {
    encPos[i].write(homesToMin(i) ? encAtMin(i) : encAtMax(i));
  }
}


void jointPosToEncSteps(double* jointPos, int* encSteps,int * joints = NULL) {
  for (int i = 0; i < NUM_JOINTS; ++i) {
      if (joints != NULL && joints[i] == 0)
        continue;
    encSteps[i] = jointPos[i] * MOTOR_STEPS_PER_DEG[MODEL][i] * ENC_DIR[i];
  }
}

String JointPosToString(double* jointPos) {
  String out;
  for (int i = 0; i < NUM_JOINTS; ++i) {
    out += JOINT_NAMES[i];
    out += String(jointPos[i], 6);
  }
  return out;
}

void SaveRestStepsFromCommand(String inData) {
  Serial8.println("ED: Received command to store rest steps.");
  Serial8.print("ED: Raw input: ");
  Serial8.println(inData);

  int steps[NUM_JOINTS] = {0};
  int idx = 0;
  char* token = strtok(inData.c_str() + 2, ",");

  while (token != NULL && idx < NUM_JOINTS) {
    steps[idx] = atoi(token);
    Serial8.print("ED: Parsed joint ");
    Serial8.print(idx);
    Serial8.print(" → ");
    Serial8.println(steps[idx]);
    idx++;
    token = strtok(NULL, ",");
  }

  if (idx != NUM_JOINTS) {
    Serial8.print("ED: Error - expected ");
    Serial8.print(NUM_JOINTS);
    Serial8.print(" values but got ");
    Serial8.println(idx);
    Serial8.println("ED: Aborting write to EEPROM.");
    return;
  }

  Serial8.println("ED: Writing to EEPROM...");
  for (int i = 0; i < NUM_JOINTS; ++i) {
    REST_MOTOR_STEPS[MODEL][i] = steps[i];
    EEPROM.put(i * sizeof(int), steps[i]);
    Serial8.print("ED: EEPROM[");
    Serial8.print(i);
    Serial8.print("] ← ");
    Serial8.println(steps[i]);
  }

  Serial8.println("ED: Successfully saved REST_MOTOR_STEPS to EEPROM.");
}



String JointVelToString(double* lastVelocity) {
  String out;

  for (int i = 0; i < NUM_JOINTS; ++i) {
    out += JOINT_NAMES[i];
    out += String(lastVelocity[i], 6);
  }

  return out;
}

void ParseMessage(String& inData, double* cmdJointPos) {
  for (int i = 0; i < NUM_JOINTS; i++) {
    bool lastJoint = i == NUM_JOINTS - 1;
    int msgIdxJ_S, msgIdxJ_E = 0;
    msgIdxJ_S = inData.indexOf(JOINT_NAMES[i]);
    msgIdxJ_E = (lastJoint) ? -1 : inData.indexOf(JOINT_NAMES[i + 1]);
    if (msgIdxJ_S == -1) {
      Serial.printf("ER: panic, missing joint %c\n", JOINT_NAMES[i]);
      return;
    }
    if (msgIdxJ_E != -1) {
      cmdJointPos[i] = inData.substring(msgIdxJ_S + 1, msgIdxJ_E).toFloat();
    } else {
      cmdJointPos[i] = inData.substring(msgIdxJ_S + 1).toFloat();
    }
  }
}

void MoveVelocity(String inData) {
  double cmdJointVel[NUM_JOINTS];
  ParseMessage(inData, cmdJointVel);

  for (int i = 0; i < NUM_JOINTS; i++) {
    if (abs(cmdJointVel[i]) > JOINT_MAX_SPEED[i]) {
      Serial.printf("DB: joint %c speed %f > %f, clipping.\n", JOINT_NAMES[i],
                    cmdJointVel[i], JOINT_MAX_SPEED[i]);
      cmdJointVel[i] = sgn(cmdJointVel[i]) * JOINT_MAX_SPEED[i];
    }
    cmdJointVel[i] *= MOTOR_STEPS_PER_DEG[MODEL][i];
    stepperJoints[i].setMaxSpeed(abs(cmdJointVel[i]));
    stepperJoints[i].setSpeed(cmdJointVel[i]);
    stepperJoints[i].move(sgn(cmdJointVel[i]) * __LONG_MAX__);
  }
}

void MoveTo(const int* cmdSteps, int* motorSteps,int * joints = NULL,bool verbose = false) {
  setAllMaxSpeeds();
  for (int i = 0; i < NUM_JOINTS; ++i) {
    if (joints != NULL && joints[i] == 0)
      continue;
    int diffEncSteps = cmdSteps[i] - motorSteps[i];
    if (abs(diffEncSteps) > 2) {
      int diffMotSteps = diffEncSteps * ENC_DIR[i];
      stepperJoints[i].move(diffMotSteps);
      if (verbose){
        // Serial8.print("Moving joint ");
        // Serial8.print(i);
        // Serial8.print(" to ");
        // Serial8.print(cmdSteps[i]);
        // Serial8.print(" from ");
        // Serial8.println(motorSteps[i]);
        // Serial8.print("Diff: ");
        // Serial8.print("Diff: ");
        // Serial8.println(diffMotSteps);
        // //print the current position
        // Serial8.print("Current position: ");
        // Serial8.println(stepperJoints[i].currentPosition());
        // Serial8.print("ENC_DIR["); 
        // Serial8.print(i); 
        // Serial8.print("]: ");
        // Serial8.println(ENC_DIR[i]);
      }

    }
  }
}

void encStepsToJointPos(int* encStepsMS, double* jointPos, int* joints = NULL) {
  // encStepsMS are motor steps (encPos.read() / ENC_MULT)
  for (int i = 0; i < NUM_JOINTS; ++i) {
    if (joints != NULL && joints[i] == 0) continue;

    // steps -> degrees with wiring direction
    double angle_deg = (double)encStepsMS[i] / MOTOR_STEPS_PER_DEG[MODEL][i] * ENC_DIR[i];

    // subtract calibrated zero in degrees (same sign convention)
    jointPos[i] = angle_deg - ZERO_OFFSET_DEG[i];
  }
}



void MoveTo(String inData, int* motorSteps,int * joints = NULL) {
  double cmdJointPos[NUM_JOINTS] = {0};
  ParseMessage(inData, cmdJointPos);

  for (int i = 0; i < NUM_JOINTS; i++) {
    if (joints != NULL && joints[i] == 0)
      continue;
    if (abs(cmdJointPos[i] > 380.0)) {
      Serial.printf("ER: panic, joint %c value %f out of range\n",
                    JOINT_NAMES[i], cmdJointPos[i]);
      return;
    }
  }

  // update target joint position
  int cmdEncSteps[NUM_JOINTS] = {0};
  logThrottled("*Moving to new pos"); 
  jointPosToEncSteps(cmdJointPos, cmdEncSteps,joints);

  MoveTo(cmdEncSteps, motorSteps,joints);
}

bool AtPosition(const int* targetMotorSteps, const int* currMotorSteps,const int maxDiff, int* joints) {
  bool allDone = true;
  for (int i = 0; i < NUM_JOINTS; ++i) {
    if (joints != NULL && joints [i] == 0)
      continue;
    int diffEncSteps = targetMotorSteps[i] - currMotorSteps[i];
    if (abs(diffEncSteps) > maxDiff) {
      allDone = false;
    }
    else {
      if (!isJointAtPosition[i]) {
        Serial8.println("Joint " + String(i) + " is at position");
      } 
      isJointAtPosition[i] = true;
    }
  }
  return allDone;
}

void setAllMaxSpeeds() {
  for (int i = 0; i < NUM_JOINTS; ++i) {
    stepperJoints[i].setMaxSpeed(JOINT_MAX_SPEED[i] *
                                 MOTOR_STEPS_PER_DEG[MODEL][i]);
  }
}

void updateAllLimitSwitches() {
  for (int i = 0; i < NUM_JOINTS; ++i) {
    limitSwitches[i].update();
  }
  delay(3);
}

bool moveToLimitSwitches(int* calJoints) {
  // check which joints to calibrate
  bool calAllDone = false;
  bool calJointsDone[NUM_JOINTS];
  for (int i = 0; i < NUM_JOINTS; ++i) {
    calJointsDone[i] = !calJoints[i];
  }

  Serial8.println("Setting speed");
  for (int i = 0; i < NUM_JOINTS; i++) {
    stepperJoints[i].setMaxSpeed(CAL_SPEED * CAL_SPEED_MULT[i] * CAL_DIR[i]);
    stepperJoints[i].setSpeed(CAL_SPEED * CAL_SPEED_MULT[i] * CAL_DIR[i]);
  }
  Serial8.println("Speed was set");
  unsigned long startTime = millis();
  while (!calAllDone) {
    updateAllLimitSwitches();
    // if (millis() %1000 == 0)
    //   Serial8.println ("moving to limit switches");
    calAllDone = true;
    for (int i = 0; i < NUM_JOINTS; ++i) {
      // if joint is not calibrated yet
      if (!calJointsDone[i]) {
        // check limit switches
        if (!limitSwitches[i].isPressed()) {
          // limit switch not reached, continue moving
          safeRunSpeed(stepperJoints[i]);
          calAllDone = false;
        } else {
          // limit switch reached
          Serial8.println("limit switch reached: " + String(i));
          stepperJoints[i].setSpeed(0);  // redundancy
          calJointsDone[i] = true;
        }
      }
    }

    if (millis() - startTime > 50000) {
      return false;
    }
  }
  Serial8.println("reached all limit swiches");
  delay(1000);
  return true;
}

void PrintOutEncodersAndLimitSwitchS8() {
  Serial8.print ("Enc:");
  for (int i = 0; i < NUM_JOINTS; ++i) {
    int encValue = encPos[i].read();
    Serial8.print (encValue);
    Serial8.print (" ");
  } 
  Serial8.print ("LimitSwitch:");
  updateAllLimitSwitches();
  for (int i = 0; i < NUM_JOINTS; ++i) {
      String Value = limitSwitches[i].isPressed() ? "1 " : "0 ";
      Serial8.print(Value);
  }
 
  Serial8.println (" ");
  Serial8.flush();
}



void PrintOutEncodersAndLimitSwitchS1() {
  Serial.print ("Enc:");
  for (int i = 0; i < NUM_JOINTS; ++i) {
    int encValue = encPos[i].read();
    Serial.print (encValue);
    Serial.print (" ");
  } 
  Serial.print ("LimitSwitch:");
  updateAllLimitSwitches();
  for (int i = 0; i < NUM_JOINTS; ++i) {
      String Value = limitSwitches[i].isPressed() ? "1 " : "0 ";
      Serial.print(Value);
  }
 
  Serial.println (" ");
  Serial.flush();
}


bool moveAwayFromLimitSwitch(int* calJoints) {
  Serial8.println("Start moveAwayFromLimitSwitch");
  for (int i = 0; i < NUM_JOINTS; i++) {
    if (calJoints[i]) {
      stepperJoints[i].setMaxSpeed(CAL_SPEED * CAL_SPEED_MULT[i] * CAL_DIR[i] *-1);
      stepperJoints[i].setSpeed(CAL_SPEED * CAL_SPEED_MULT[i] * CAL_DIR[i] *-1);
    }
  }

  bool limitSwitchesActive = true;
  unsigned long startTime = millis();
  while (limitSwitchesActive || millis() - startTime < 4000) {
    limitSwitchesActive = false;
    updateAllLimitSwitches();
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (calJoints[i]) {
        if (limitSwitches[i].isPressed()) {
          limitSwitchesActive = true;
        }
        safeRunSpeed(stepperJoints[i]);
      }
    }

    if (millis() - startTime > 20000) {
      return false;
    }
  }

  for (int i = 0; i < NUM_JOINTS; i++) {
    stepperJoints[i].setSpeed(0);  // redundancy
  }
  delay(1000);
  Serial8.println("End moveAwayFromLimitSwitch");
  return true;
}

bool moveLimitedAwayFromLimitSwitch(int* calJoints) {
  // move the ones that already hit a limit away from it before start of
  // calibration
  int limitedJoints[NUM_JOINTS] = {0};
  updateAllLimitSwitches();
  for (int i = 0; i < NUM_JOINTS; i++) {
    limitedJoints[i] = (calJoints[i] && limitSwitches[i].isPressed());
  }
  return moveAwayFromLimitSwitch(limitedJoints);
}

int counter = 0;
void PrintDiff(int* curMotorSteps)
{
    Serial8.println("====Diff======");
    //counter ++;
    //if (counter == 50)
    {
      Serial8.println("==============");
      for (int i = 0 ; i < NUM_JOINTS ; i++) {
        Serial8.println (String(i) + ":" + (REST_MOTOR_STEPS[MODEL][i] - curMotorSteps[i]));
      }
      counter = 0;
    }
}

int check_encoder_connected (int* curMotorSteps,int* initialMotorSteps, int* joints) {
  // check if encoder is connected
  for (int i = 0; i < NUM_JOINTS; ++i) {
    if (joints != NULL && joints [i] == 0)
      continue;
    if (curMotorSteps[i] == initialMotorSteps[i]) {  //if value is still 0, encoder is not connected
      Serial8.println("Encoder not connected: " + String(i));
      Serial8.println("Initial: " + String(initialMotorSteps[i]));
      Serial8.println("Current: " + String(curMotorSteps[i]));
      return i;
    }
  }
  return -1;
}

bool ReturnToOriginalPosition(String &outputMsg, int* calJoints) {
  Serial8.println("=== Start ReturnToOriginalPosition ===");

  unsigned long startTime = millis();
  int curMotorSteps[NUM_JOINTS];
  readMotorSteps(curMotorSteps, calJoints);

  // Log initial step readings
  Serial8.println("Initial motor steps:");
  for (int i = 0; i < NUM_JOINTS; ++i) {
    Serial8.print("  J"); Serial8.print(i+1);
    Serial8.print(" current="); Serial8.print(curMotorSteps[i]);
    Serial8.print(" target=");  Serial8.print(REST_MOTOR_STEPS[MODEL][i]);
    Serial8.print(" ENC_DIR="); Serial8.println(ENC_DIR[i]);
  }

  ResetJointAtPosition();

  // Workaround for AccelStepper bug
  for (int i = 0 ; i < NUM_JOINTS; ++i) {
    if (!calJoints || calJoints[i] == 1) {
      stepperJoints[i].setCurrentPosition(0);
      Serial8.print("Joint "); Serial8.print(i+1); Serial8.println(": position reset to 0");
    }
  }

  Serial8.println("Starting motion toward rest positions...");

  while (!AtPosition(REST_MOTOR_STEPS[MODEL], curMotorSteps, 3, calJoints)) {
    if (millis() - startTime > 40000) {
      outputMsg = "ER: Failed to return to original position.";
      Serial8.println(outputMsg);
      PrintDiff(curMotorSteps);
      break;
    }

    readMotorSteps(curMotorSteps, calJoints);

    // Log current diff from target
    // Serial8.print("Progress: ");
    // for (int i = 0; i < NUM_JOINTS; ++i) {
    //   if (!calJoints || calJoints[i] == 1) {
    //     long diff = REST_MOTOR_STEPS[MODEL][i] - curMotorSteps[i];
    //     Serial8.print("J"); Serial8.print(i+1);
    //     Serial8.print(" diff="); Serial8.print(diff);
    //     Serial8.print(" cur="); Serial8.print(curMotorSteps[i]);
    //     Serial8.print(" tgt="); Serial8.print(REST_MOTOR_STEPS[MODEL][i]);
    //     Serial8.print(" | ");
    //   }
    // }
    // Serial8.println();

    MoveTo(REST_MOTOR_STEPS[MODEL], curMotorSteps, calJoints);
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (!calJoints || calJoints[i] == 1) {
        safeRun(stepperJoints[i]);
      }
    }
  }

  Serial8.println("Reached original position (or timeout).");

  Serial8.println("Final motor steps:");
  for (int i = 0; i < NUM_JOINTS; ++i) {
    Serial8.print("  J"); Serial8.print(i+1);
    Serial8.print(" current="); Serial8.print(curMotorSteps[i]);
    Serial8.print(" target=");  Serial8.print(REST_MOTOR_STEPS[MODEL][i]);
    Serial8.println();
  }

  Serial8.println("=== End ReturnToOriginalPosition ===");
  return true;
}


bool doCalibrationRoutine(String& outputMsg, int* calJoints) {
  Serial8.println("Start Calibration");

  // 1) If any selected joint is already on a limit, back off first
  if (!moveLimitedAwayFromLimitSwitch(calJoints)) {
    outputMsg = "ER: Failed to move away from limit switches at the start.";
    return false;
  }

  // 2) Home the selected joints toward their limit switches
  Serial8.println("moving to limit switches");
  if (!moveToLimitSwitches(calJoints)) {
    outputMsg = "ER: Failed to move to limit switches.";
    return false;
  }
  Serial8.println("moveToLimitSwitches complete");

  // 3) Record raw encoder counts at the limit (for reporting)
  int calSteps[NUM_JOINTS];
  for (int i = 0; i < NUM_JOINTS; ++i) {
    calSteps[i] = encPos[i].read();
  }

  // 4) Set encoders to the correct limit value (0 or max) per wiring
  for (int i = 0; i < NUM_JOINTS; ++i) {
    encPos[i].write(ENC_RANGE_STEPS[i] * ENC_MAX_AT_ANGLE_MIN[i]);
  }

  // 5) Move off the switches a bit for safety
  if (!moveAwayFromLimitSwitch(calJoints)) {
    outputMsg = "ER: Failed to move away from limit switches.";
    return false;
  }
  Serial8.println("moveAwayFromLimitSwitch complete");

  // 6) Restore nominal max speeds
  for (int i = 0; i < NUM_JOINTS; ++i) {
    if (calJoints[i] == 0) continue;
    stepperJoints[i].setMaxSpeed(JOINT_MAX_SPEED[i] * MOTOR_STEPS_PER_DEG[MODEL][i]);
  }

  // 7) Return to configured REST_MOTOR_STEPS pose
  if (!ReturnToOriginalPosition(outputMsg, calJoints)) {
    return false;
  }

  // 8) Final report: encoders, limit switches
  Serial8.println("calibration complete");
  outputMsg = String("JC") +
              "A" + calSteps[0] +
              "B" + calSteps[1] +
              "C" + calSteps[2] +
              "D" + calSteps[3] +
              "E" + calSteps[4] +
              "F" + calSteps[5];
  Serial8.println(outputMsg);

  PrintOutEncodersAndLimitSwitchS8();

  // 9) Convert *current* motor steps → joint angles and print them
  //    (Uses your encStepsToJointPos)
  
  UpdateCalibrationOffsets(calJoints);
  
  // int curMotorSteps[NUM_JOINTS];
  // double curJointDeg[NUM_JOINTS];
  // double diffDeg[NUM_JOINTS] = {0.0};
  // // readMotorSteps returns motor steps = encPos.read() / ENC_MULT
  // readMotorSteps(curMotorSteps /*, calJoints*/);   // read all; omit mask
  // encStepsToJointPos(curMotorSteps, curJointDeg /*, nullptr*/);

  // Serial8.println("Final joint angles (deg) at end of calibration:");
  // for (int i = 0; i < NUM_JOINTS; ++i) {
  //   Serial8.print("  J"); Serial8.print(i + 1); Serial8.print(": ");
  //   Serial8.println(curJointDeg[i], 3);
  //   diffDeg[i] = fabs(curJointDeg[i]) - fabs(JOINT_LIMIT_MIN[MODEL][i]);
  //   Serial8.print("  Diff from min limit: ");
  //   Serial8.println(diffDeg[i], 3);
  //   ZERO_OFFSET_DEG[i] = diffDeg[i];
  //   if (i == 0) 
  //   {
  //     ZERO_OFFSET_DEG[i] = -ZERO_OFFSET_DEG[i]; // for J1 we need to invert the offset
  //   }
  // }
  Serial8.println("End of Calibration Process");
  Serial8.flush();
  return true;
}

void UpdateCalibrationOffsets(int* calJoints) {
  int curMotorSteps[NUM_JOINTS];
  double curJointDeg[NUM_JOINTS];
  double diffDeg[NUM_JOINTS] = {0.0};

  for (int i = 0; i < NUM_JOINTS; ++i) 
    if (calJoints == NULL || calJoints[i] == 1) {
      ZERO_OFFSET_DEG[i] = 0.0; // no calibration for this joint
      continue;
    }

  // readMotorSteps returns motor steps = encPos.read() / ENC_MULT
  readMotorSteps(curMotorSteps /*, calJoints*/);   // read all; omit mask


  encStepsToJointPos(curMotorSteps, curJointDeg /*, nullptr*/);

  Serial8.println("Final joint angles (deg) at end of calibration:");
  for (int i = 0; i < NUM_JOINTS; ++i) {
    Serial8.print("  J"); Serial8.print(i + 1); Serial8.print(": ");
    Serial8.println(curJointDeg[i], 3);
    diffDeg[i] = fabs(curJointDeg[i]) - fabs(JOINT_LIMIT_MIN[MODEL][i]);
    Serial8.print("  Diff from min limit: ");
    Serial8.println(diffDeg[i], 3);
    ZERO_OFFSET_DEG[i] = diffDeg[i];
    if (i == 0) 
    {
      ZERO_OFFSET_DEG[i] = -ZERO_OFFSET_DEG[i]; // for J1 we need to invert the offset
    }
  }
}

void updateMotorVelocities(int* motorSteps, int* lastMotorSteps,
                           int* checksteps, unsigned long* lastVelocityCalc,
                           double* lastVelocity) {
  for (int i = 0; i < NUM_JOINTS; i++) {
    // for really small velocities we still get
    // artifacts, but quite manageable now!

    if (micros() - lastVelocityCalc[i] < 5000) {
      // we want to trigger calculation only after x ms but
      // immediately when steps change after that
      checksteps[i] = motorSteps[i];
      continue;
    }

    // have to add some sort of outlier-filter here , maybe moving average ..
    if (abs(stepperJoints[i].speed() / MOTOR_STEPS_PER_DEG[MODEL][i]) < 5) {
      // NB! trying to fix artifacts at low velocity
      if (abs(motorSteps[i] - checksteps[i]) > 0) {
        lastVelocity[i] =
            stepperJoints[i].speed() / MOTOR_STEPS_PER_DEG[MODEL][i];
        lastMotorSteps[i] = motorSteps[i];
        lastVelocityCalc[i] = micros();
      } else if (stepperJoints[i].speed() == 0) {
        lastVelocity[i] = 0;
      }
    } else {
      unsigned long currentMicros = micros();
      double delta = (currentMicros - lastVelocityCalc[i]);
      if (abs(motorSteps[i] - checksteps[i]) > 0) {
        // calculate TRUE motor velocity
        lastVelocity[i] = ENC_DIR[i] * (motorSteps[i] - lastMotorSteps[i]) /
                          MOTOR_STEPS_PER_DEG[MODEL][i] / (delta / 1000000.0);
        lastMotorSteps[i] = motorSteps[i];
        lastVelocityCalc[i] = currentMicros;
      }
    }
  }
}

void parseRestPosString(const String input) {
  int count = 0;
  const int maxCount = NUM_JOINTS;

  int* outputArray = REST_MOTOR_STEPS["mk3"];  // Assuming REST_MOTOR_STEPS is a map-like structure with int* values
  if (outputArray == nullptr) {
    Serial8.println("Error: REST_MOTOR_STEPS[\"mk3\"] is null");
    return;
  }

  int startIdx = input.indexOf(' ') + 1;  // Skip the "RP "

  while (startIdx > 0 && count < maxCount) {
    int endIdx = input.indexOf(' ', startIdx);
    String numStr;

    if (endIdx == -1) {
      numStr = input.substring(startIdx);
    } else {
      numStr = input.substring(startIdx, endIdx);
    }

    outputArray[count++] = numStr.toInt();

    if (endIdx == -1) break;
    startIdx = endIdx + 1;
  }

  Serial8.print("Parsed values: ");
  for (int i = 0; i < count; ++i) {
    Serial8.print(outputArray[i]);
    Serial8.print(" ");
  }
  Serial8.println(); 
}

void ProcessCalibrationString(String input, int * calJoints) {
    // Ensure the input string is exactly 8 characters long
    if (input.length() != 9) {
        Serial8.println("no specific joints: " + String (input) + "<--" + String(input.length()) );
        return;
    }

    // Copy the last 6 characters into the array as integers
    for (int i = 0; i < NUM_JOINTS; i++) {
        calJoints[i] = input[i + 2] - '0'; // Convert char to int
    }

    // Print values for debugging
    Serial8.print("Extracted values: ");
    for (int i = 0; i < NUM_JOINTS; i++) {
        Serial8.print(calJoints[i]);
        Serial8.print(" ");
    }
    Serial8.println();
}

void moveServoTo(int targetAngle) {
  // int currentAngle = myServo.read();
  // int step = (targetAngle > currentAngle) ? ServoAngleStep : -ServoAngleStep;

  // for (int angle = currentAngle; angle != targetAngle; angle += step) {
  //   myServo.write(angle);
  //   delay(ServoStepDelay);
  // }
  
  myServo.write(targetAngle); // Ensure exact position
  delay(20);
}



void stateTRAJ() {
  // clear message
  String inData = "";

  // initialise joint steps
  double curJointPos[NUM_JOINTS];
  int curMotorSteps[NUM_JOINTS];
  int lastMotorSteps[NUM_JOINTS];
  int checksteps[NUM_JOINTS];
  double lastVelocity[NUM_JOINTS];
  unsigned long lastVelocityCalc[NUM_JOINTS];

  readMotorSteps(curMotorSteps);

  for (int i = 0; i < NUM_JOINTS; ++i) {
    lastVelocityCalc[i] = micros();
    lastMotorSteps[i] = curMotorSteps[i];
  }

  // start loop
  while (STATE == STATE_TRAJ) {
    char received = '\0';
    // check for message from host
    if (Serial.available()) {
      received = Serial.read();
      inData += received;
    }

    if (MODEL != "") {
      readMotorSteps(curMotorSteps);
      updateMotorVelocities(curMotorSteps, lastMotorSteps, checksteps,
                            lastVelocityCalc, lastVelocity);
    }

    // process message when new line character is received
    if (received == '\n') {
      String function = inData.substring(0, 2);

      //logThrottled("Received: " + inData);

      if (function == "ST") {
        if (!initStateTraj(inData)) {
          STATE = STATE_ERR;
          return;
        }
      } else if (MODEL == "") {
        // if model is not set, do not proceed with any other function
        STATE = STATE_ERR;
        return;
      }

      if (function == "MT") {
        // clear speed counter
        for (int i = 0; i < NUM_JOINTS; i++) {
          if (stepperJoints[i].speed() == 0) {
            lastVelocityCalc[i] = micros();
          }
        }

        MoveTo(inData, curMotorSteps);

        // update the host about estop state
        String msg = String("ES") + estop_pressed;
        SendToROS(msg);

      } else if (function == "MV") {
        // clear speed counter
        for (int i = 0; i < NUM_JOINTS; i++) {
          if (stepperJoints[i].speed() == 0) {
            lastVelocityCalc[i] = micros();
          }
        }

        MoveVelocity(inData);

        // update the host about estop state
        String msg = String("ES") + estop_pressed;
        SendToROS(msg);

      } else if (function == "RP") {
        // read rest motor step offsets
        parseRestPosString(inData);
      
      } else if (function == "JP") {
        readMotorSteps(curMotorSteps);
        encStepsToJointPos(curMotorSteps, curJointPos);
        String msg = String("JP") + JointPosToString(curJointPos);
        SendToROS(msg);
      } else if (function == "JV") {
        String msg = String("JV") + JointVelToString(lastVelocity);
        SendToROS(msg);
      } else if (function == "JC") {
        String msg;
        int calJoints[] = {1, 1, 1, 1, 1, 1};
        ProcessCalibrationString(inData,calJoints);
        if (!doCalibrationRoutine(msg,calJoints)) {
          for (int i = 0; i < NUM_JOINTS; ++i) {
            stepperJoints[i].setSpeed(0);
          }
        }
        SendToROS(msg);
        Serial8.println("Completed handling calibration");
      } else if (function == "RE") {
        resetEstop();
        // update host with Estop status after trying to reset it
        String msg = String("ES") + estop_pressed;
        SendToROS(msg);
      } else if (function == "LE") {//test Limit switches and encoders
        PrintOutEncodersAndLimitSwitchS1();
      } else if (function == "GR") {
        PrintRestMotorStepOffsets();
      }
      else if (function == "SR") {
        String msg;
        ApplyRestMotorStepOffset(inData);
        int calJoints[] = {1,1,1,1,1, 1};
        ReturnToOriginalPosition(msg,calJoints);
      }
      else if (function == "HM") {
        ManualHomeOffset(inData);
      }
      else if (function == "CE") {
        ClearRestStepsEEPROM();
      }
      else if (function == "SE") {
        SaveRestStepsToEEPROM();
      }
      else if (function == "ER") {
        PrintEEPROMRestSteps();
      }
      else if (inData.startsWith("ED")) {
        Serial8.println("Dispatch: Handling ED command.");
        SaveRestStepsFromCommand(inData);
      }
      else if (function == "OG") { // Open Gripper
        Serial8.println("Dispatch: Handling OG command.");
        moveServoTo(minServoAngle);
      }
      else if (function == "CG") { // Close Gripper
        Serial8.println("Dispatch: Handling CG command.");
        moveServoTo(maxServoAngle);
      }
      else if (function == "SF") {
          SetGlobalSpeedScale(inData);
      }
      else if (inData.startsWith("SA")) { 
        Serial8.println("Moving Servo to Angle");
        SetServoAngle(inData); // Set Servo Angle
      } 
      else {
        Serial8.print("couldnt process:");
        Serial8.println(inData);
      }
      inData = "";  // clear message
    }

    for (int i = 0; i < NUM_JOINTS; ++i) {
      safeRun(stepperJoints[i]);
    }
  }
}

void SetServoAngle(String inData) {
  // parse the angle
  int spaceIdx = 1;
  if (spaceIdx > 0) {
    double req = inData.substring(spaceIdx + 1).toFloat();

    // if (req < 0) req = 0;
    // if (req > 50) req = 50;

    // move (blocking stepped move, matches your OG/CG behavior)
    moveServoTo((int)req);

    // ACK back to host over Serial (USB)
    SendToROS(String("SAOK") + req);    // e.g. "SAOK23"
  } else {
    SendToROS("ER: SA missing angle");
  }
}


void stateERR() {
  // enter holding state
  for (int i = 0; i < NUM_JOINTS; ++i) {
    digitalWrite(STEP_PINS[i], LOW);
  }

  while (STATE == STATE_ERR) {
    SendToROS("ER: Unrecoverable error state entered. Please reset.");
    delay(1000);
  }
}

void loop() {
  STATE = STATE_TRAJ;

  switch (STATE) {
    case STATE_ERR:
      stateERR();
      break;
    default:
      stateTRAJ();
      break;
  }
}
