#include "Arduino.h"
#include <Servo.h>

// Pin definitions 
#define PRESSURE_SENSOR A1
#define LED_PIN_ON 2
#define LED_PIN_OFF 10
#define TOGGLE_PIN 13
#define BUTTON_PRESURE 1 // Use A5 as the button pin
#define E1 3
#define M1 4
#define E2 11
#define M2 12
#define E3 5
#define M3 8
#define E4 6
#define M4 7


// Motor pin
#define SERVO_PIN 9
#define POT_PIN A0

// Motor constants
const int IDLE_ANGLE = 90;   // Middle position
const int RIGHT_ANGLE = 180;  // Fully to the right
const int LEFT_ANGLE = 0;     // Fully to the left

// Constants and variables
const float SensorOffset = 4.44;
const float SensorGain = 0.109;
const float alpha = 0.2;
float pressure = 0.0;             // Raw pressure value
float filteredPressure = 0.0;     // Filtered pressure value
float prevFilteredPressure = 0.0; // Previous filtered pressure value
int stateprocess = 1;
bool lock = false;
int timecounter = 1;
// Global important variable
bool lastSystemOn = false; // Variable to remember the last state of the system
volatile bool StartProgram = false;

bool vacuumActive; // Variable to control the vacuum suction cup
bool suctionControlEnabled = false; // Flag to control Pump 2 directly



Servo myservo;

void setup() {

  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(M3, OUTPUT);
  pinMode(M4, OUTPUT);
  pinMode(PRESSURE_SENSOR, INPUT);
  pinMode(LED_PIN_ON, OUTPUT);
  pinMode(LED_PIN_OFF, OUTPUT);
  pinMode(TOGGLE_PIN, INPUT); 
  pinMode(BUTTON_PRESURE, INPUT);

  myservo.attach(SERVO_PIN); // Attach the servo to the SERVO_PIN


  Serial.begin(115200);
  Serial.println("Setup completed.");

  printHeaders();
}

void loop() {
  bool isSystemOn = digitalRead(TOGGLE_PIN);
  bool buttonState = digitalRead(BUTTON_PRESURE);

    // Toggle suction control based on the button press
  if (buttonState == HIGH) { // Assuming the button is active low
    suctionControlEnabled = true;
  } else {
    suctionControlEnabled = false;
  }

  if (isSystemOn) {
    if (!lastSystemOn) {
      handleSystemPowerOn();
    }
    digitalWrite(LED_PIN_ON, HIGH);
    digitalWrite(LED_PIN_OFF, LOW);
    static unsigned long lastUpdateTime = 0; // Last update time
    unsigned long currentMillis = millis(); // Current time


    // Update timecounter and manage system states at different rates
    if ((stateprocess == 3 && currentMillis - lastUpdateTime >= 100) || // Fast rate for deflation
        (stateprocess != 3 && currentMillis - lastUpdateTime >= 100)) { // Standard rate otherwise
      timecounter++;
      updatePressure();
      manageSystemStates();
      printData();
      updateServo();
      lastUpdateTime = currentMillis; // Save the last update time
    }
    lastSystemOn = true; 
  } else {
    if (lastSystemOn) {
      handleSystemPowerOff();
    }
    digitalWrite(LED_PIN_ON, LOW);
    digitalWrite(LED_PIN_OFF, HIGH);
    stopAll();
    updateServo();
  }
}

// void updatePressure() {
//   float pressure_sensorValue = (analogRead(PRESSURE_SENSOR) * SensorGain - SensorOffset);
//   pressure = pressure_sensorValue;
//   filteredPressure = filteredPressure + alpha * (pressure - prevFilteredPressure);
//   prevFilteredPressure = filteredPressure;
// }



void updatePressure() {
  float pressure_sensorValue = (analogRead(PRESSURE_SENSOR) * SensorGain - SensorOffset);
  // Validate sensor reading
  if (pressure_sensorValue < 0 || pressure_sensorValue > 100) { // Replace MAX_SENSOR_VALUE with the maximum expected value
    Serial.println("Invalid pressure sensor reading.");
    return;
  }
  pressure = pressure_sensorValue;
  filteredPressure = filteredPressure + alpha * (pressure - prevFilteredPressure);
  prevFilteredPressure = filteredPressure;
}
void manageSystemStates() {
  Serial.print("Current State: ");
  Serial.println(stateprocess);

  switch (stateprocess) {
    case 0:
      stopAll();
      break;
    case 1:
      Serial.println("Inflating system...");
      vacuumActive = false; // No vacuum during inflation
      controlVacuum();
      inflateSystem();
      break;
    case 2:
      vacuumActive = true; // Activate vacuum during lock
      controlVacuum();
      lockSystem();
      break;
    case 3:
      vacuumActive = false; // Deactivate vacuum during deflation
      controlVacuum();
      deflateSystem();
      break;
    case 4:
      transitionState();
      break;
    case 5:
      deflateRestingState();
      break;
  }
}

// Function to create a vacuum cycle
void createVacuumCycle() {
  //controlVacuum();
  // Make an extension with pump 1
  inflateSystem();


}


void controlVacuum() {
  if (vacuumActive && suctionControlEnabled) {
    Serial.println("Activating Vacuum");
    valve_2_on(); // Activate the suction cup
  } else {
    Serial.println("Deactivating Vacuum");
    valve_2_off(); // Deactivate the suction cup
  }
}


void handleSystemPowerOn() {
  stateprocess = 1; // Change to inflate state
  digitalWrite(LED_PIN_ON, HIGH);
  digitalWrite(LED_PIN_OFF, LOW);
}

void handleSystemPowerOff() {
  lastSystemOn = false;
  if(filteredPressure < 15) {
    stateprocess = 5;
  }
  digitalWrite(LED_PIN_ON, LOW);
  digitalWrite(LED_PIN_OFF, HIGH);
  stopAll();

  controlVacuum(); // Update the vacuum control to reflect the new state

  Serial.println("System powered off. Vacuum control deactivated.");
}

void deflateRestingState() {
  valve_1_on(); // Open the valve to deflate
  pump_1_off(); // Make sure the pump is off
  // Exit this state when pressure is low enough
  if (filteredPressure <= 15) {
    valve_1_off(); // Close the valve after deflation
    stateprocess = 1; // Go back to the initial state after deflation/resting
  }
}

void updateServo() {
  int totalReading = 0;
  const int NUM_READINGS = 7;

  for (int i = 0; i < NUM_READINGS; i++) {
    totalReading += analogRead(POT_PIN);
    delay(5);
  }

  int avgPotValue = totalReading / NUM_READINGS;

  // Serial.print("Pot value: ");
  // Serial.println(avgPotValue);

  int servoAngle;

  // Check which section the avgPotValue falls into and set the corresponding servo angle
  if (avgPotValue < 341) {
    servoAngle = LEFT_ANGLE;
  } else if (avgPotValue > 682) {
    servoAngle = RIGHT_ANGLE;
  } else {
    servoAngle = IDLE_ANGLE;
  }

  myservo.write(servoAngle);  // Set servo to the desired position

  // Serial.print("Servo angle: ");
  // Serial.println(servoAngle);
}


void stopAll() {
  pump_1_off();
  valve_1_off();
  pump_2_off(); // Ensure Pump 2 is also off
}
void inflateSystem() {
  if (!lock) {
    pump_1_on(250); // Pump 1 for inflating
    //pump_2_on(250); // Pump 2 for suction
    valve_1_off();
  }
  if (filteredPressure >=75) stateprocess = 2;
}

void lockSystem() {
  lock = true;
  valve_1_off();
  pump_1_off();
  if (suctionControlEnabled) {
    Serial.println("Active Vacuum, power on pump 2");
    pump_2_on(250); // Pump 2 for suction
    delay(100);
  }
  if (timecounter > 15) stateprocess = 3;
}

void deflateSystem() {
  pump_1_off(); // Turn off Pump 1
  valve_1_on(); // Open Valve 1 for deflation
  delay(200); // Additional delay to allow Pump 2 to fully release from the table
  pump_2_off(); // Turn off Pump 2 after the delay
  if (filteredPressure <= 15) {
    
    

    stateprocess = 4;
  }
}

void transitionState() {
  Serial.println("Transition");
  stopAll();
  lock = false;
  timecounter = 0;
  stateprocess = 1;
}

void pump_1_on(int motorspeed) {
  analogWrite(E1, motorspeed);
  digitalWrite(M1, HIGH);
}

void pump_1_off() {
  analogWrite(E1, 0);
  digitalWrite(M1, HIGH);
}

void pump_2_on(int motorspeed) {
  analogWrite(E3, motorspeed);
  digitalWrite(M3, HIGH);
}

void pump_2_off() {
  analogWrite(E3, 0);
  digitalWrite(M3, HIGH);
}

void valve_1_on() {
  analogWrite(E2, 255);
  digitalWrite(M2, HIGH);
}

void valve_1_off() {
  analogWrite(E2, 0);
  digitalWrite(M2, HIGH);
}
void valve_2_on() {
  analogWrite(E4, 255);
  digitalWrite(M4, HIGH);
}

void valve_2_off() {
  analogWrite(E4, 0);
  digitalWrite(M4, HIGH);
}


void printHeaders() {
  Serial.println("Process_Status,Pressure_sensor_Value,Filtered_Pressure");
}

void printData() {
  Serial.print(stateprocess);
  Serial.print(",");
  Serial.print(pressure);
  Serial.print(",");
  Serial.println(filteredPressure);
}
