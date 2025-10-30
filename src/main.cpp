#include <Arduino.h>
#include <stdint.h>
#include <LiquidCrystal.h>

/// Pin Definitions
// order based on physical pin location on esp32
// left side of the board
const uint8_t not_used0 = 36; 
const uint8_t not_used1 = 39;
const uint8_t speed_pot_input = 34;
const uint8_t encoder_a_input = 35;
const uint8_t motor_controller_In2 = 32;
const uint8_t motor_controller_In1 = 33;
const uint8_t motor_controller_EN = 25;
const uint8_t LCD_E = 26;
const uint8_t LCD_RW = 27;
const uint8_t LCD_RS= 14;
const uint8_t transducer_input = 12;
const uint8_t rotations_pot_input = 13;
// right side of the board
const uint8_t direction_switch_cw_input = 23; 
const uint8_t direction_switch_ccw_input = 22;
const uint8_t LCD_K = 1;
const uint8_t encoder_b_input = 3;
const uint8_t LCD_A = 21;
const uint8_t LCD_DB7 = 19;
const uint8_t LCD_DB6 = 18;
const uint8_t LCD_DB5 = 5;
const uint8_t LCD_DB4 = 17;
const uint8_t LCD_DB3 = 16;
const uint8_t LCD_DB2 = 4;
const uint8_t LCD_DB1 = 2;
const uint8_t LCD_DB0 = 15;

/// Global Variable Definitions
bool isMotorRunning = false;
bool direction = true;
bool onHold = true;
bool directionCWInput = 0;
bool directionCCWInput = 0;
const uint8_t directionTriggerThreshold = 30;
const uint16_t encoderResolution = 1440;
uint8_t speedOutput = 0;
uint8_t rotationCount = 0;
uint8_t speedSetting = 0;
uint8_t cwTriggerCount = 0;
uint8_t ccwTriggerCount = 0;
uint16_t encoderCount = 0;
uint8_t prevSpeedSetting = 0;
uint8_t filteredRotationCount = 0;
uint8_t filteredSpeed = 0;
uint8_t prevFilteredRotationCount = 0;
uint16_t speedInput = 0;
uint16_t encoderCountLimit = 0;
uint16_t transducerInput = 0;
uint16_t rotationsinput = 0;
uint32_t currentTime = 0;
uint32_t timeOfLastSwitchOff = 0;
double calculatedTorque = 0;
double maxCalculatedTorque = 0;
double prevMaxCalculatedTorque = 0;
String speedDisplay = "Slow Speed";

// Encoder interrupt service routine
void IRAM_ATTR encoderISR() {
  encoderCount++;
}

// Function Declarations:
void startMotor();
void stopMotor();
void spinSetRotations();
void displayTorque();
void displaySpeed();
void displayRotations();

// LCD object creation
LiquidCrystal lcd(LCD_RS, LCD_RW, LCD_E, LCD_DB0, LCD_DB1, LCD_DB2, LCD_DB3, LCD_DB4, LCD_DB5, LCD_DB6, LCD_DB7);

void setup() {
  // put your setup code here, to run once:
  // Pin Mode Definitions
  pinMode(speed_pot_input, INPUT);
  pinMode(encoder_a_input, INPUT);
  pinMode(motor_controller_In2, OUTPUT);
  pinMode(motor_controller_In1, OUTPUT);
  pinMode(motor_controller_EN, OUTPUT);
  pinMode(LCD_E, OUTPUT);
  pinMode(LCD_RW, OUTPUT);
  pinMode(LCD_RS, OUTPUT);
  pinMode(transducer_input, INPUT);
  pinMode(rotations_pot_input, INPUT);
  pinMode(direction_switch_cw_input, INPUT);
  pinMode(direction_switch_ccw_input, INPUT);
  //pinMode(LCD_K, OUTPUT);
  pinMode(encoder_b_input, INPUT);
  pinMode(LCD_A, OUTPUT);
  pinMode(LCD_DB7, OUTPUT);
  pinMode(LCD_DB6, OUTPUT);
  pinMode(LCD_DB5, OUTPUT);
  pinMode(LCD_DB4, OUTPUT);
  pinMode(LCD_DB3, OUTPUT);
  pinMode(LCD_DB2, OUTPUT);
  pinMode(LCD_DB1, OUTPUT);
  pinMode(LCD_DB0, OUTPUT);

  //digitalWrite(LCD_K, 0);
  digitalWrite(LCD_A, 1);

  // Attach interrupt to encoder_a_input pin
  attachInterrupt(digitalPinToInterrupt(encoder_a_input), encoderISR, RISING);

  // Start Serial Communication
  Serial.begin(9600);
  delay(100);
  Serial.println("Serial Started");

  // lcd init
  lcd.begin(16,2);

}

void loop() {

  // Read Pin Inputs
  speedInput = analogRead(speed_pot_input);
  transducerInput = analogReadMilliVolts(transducer_input);
  rotationsinput = analogRead(rotations_pot_input);
  directionCWInput = digitalRead(direction_switch_cw_input);
  directionCCWInput = digitalRead(direction_switch_ccw_input);

  // get current time
  currentTime = millis();


  // turn off hold
  if (!directionCWInput && !directionCCWInput){
    //Serial.println("Not On Hold");
    onHold = false;
  }

  /// ROTATION FILTERING
  // reduces the rotation input by a factor of 99 and removes floating point values
  rotationCount = rotationsinput/99;
  // Moving average filter for rotationCount
  // alpha = 0.3
  float alpha = 0.3f;
  filteredRotationCount = (uint8_t)((alpha * filteredRotationCount) + ((1.0f - alpha) * rotationCount));
  // Display filteredRotationCount on Serial if it changes
  if (filteredRotationCount != prevFilteredRotationCount) {
    displayRotations();
    prevFilteredRotationCount = filteredRotationCount;
  }

  /// SPEED FILTERING
  // reduces the speed input to one of three possibilities
  speedSetting = speedInput/1370;
  // Moving average filter for rotationCount
  // beta = 0.5
  float beta = 0.5f;
  filteredSpeed = (uint8_t)((beta * filteredSpeed) + ((1.0f - beta) * speedSetting));  
  // Only run switch-case if speedSetting has changed
  if (speedSetting != prevSpeedSetting) {
    switch (speedSetting) {
      case 0:
        // Handle speedSetting == 0
        speedOutput = 150;
        speedDisplay = "Slow Speed";
        displaySpeed();
        break;
      case 1:
        // Handle speedSetting == 1
        speedOutput = 200;
        speedDisplay = "Middle Speed";
        displaySpeed();
        break;
      case 2:
        // Handle speedSetting == 2
        speedOutput = 250;
        speedDisplay = "Fast Speed";
        displaySpeed();
        break;
      default:
        // Should not occur
        Serial.println("Speed setting: Invalid value");
        break;
    }
    prevSpeedSetting = speedSetting;
  }


  if (filteredRotationCount == 0){
    // manual mode
    Serial.println("Manual Mode");
  } else {
    //Serial.println(onHold);
    if (!onHold) {
      //Serial.println("Rotation Mode, Not on Hold");
      // check if in no direction position
      if (!directionCWInput && !directionCCWInput) {
        // reset direction trigger count
        cwTriggerCount = 0;
        ccwTriggerCount = 0;
      } else if (directionCWInput && !directionCCWInput) {
        // increment cw trigger count
        // reset ccw trigger count
        // set direction to true
        cwTriggerCount++;
        ccwTriggerCount = 0;
        direction = true;
      } else if (!directionCWInput && directionCCWInput) {
        // increment ccw trigger count
        // reset cw trigger count
        // set direction to false
        ccwTriggerCount++;
        cwTriggerCount = 0;
        direction = false;
      }
      // if a direction trigger counter has passed the threshold, start the motor
      if ((cwTriggerCount >= directionTriggerThreshold) || (ccwTriggerCount >= directionTriggerThreshold)) {
        // move the motor the set number of rotations
        // set motor to on hold so it cannot move again until this move is completed
        Serial.println("starting to spin set number of rotations");
        spinSetRotations();
        onHold = true;
      }
    }
  }



}

// Function Definitions

void startMotor() {
  if (!onHold) {
    if (direction){
      digitalWrite(motor_controller_In1, 1);
      digitalWrite(motor_controller_In2, 0);
      analogWrite(motor_controller_EN, speedOutput);
    } else if (!direction){
      digitalWrite(motor_controller_In1, 0);
      digitalWrite(motor_controller_In2, 1);
      analogWrite(motor_controller_EN, speedOutput);
    }
    isMotorRunning = true;
  }
}

void stopMotor() {
  digitalWrite(motor_controller_In1, 0);
  digitalWrite(motor_controller_In2, 0);
  analogWrite(motor_controller_EN, 0);
  isMotorRunning = false;
}

void spinSetRotations() {
  if (!onHold) {
    encoderCountLimit = filteredRotationCount*encoderResolution;
    startMotor();
    encoderCount = 0;
    //Serial.println(encoderCountLimit);
    while(encoderCount < encoderCountLimit) {
      displayTorque();
      //Serial.println(encoderCount);
    }
    stopMotor();
  }
}

void displayTorque() {
  transducerInput = analogReadMilliVolts(transducer_input);
  Serial.println(transducerInput);
  // Torque calculation
  if (calculatedTorque > maxCalculatedTorque){
    maxCalculatedTorque = calculatedTorque;
  }
  if (maxCalculatedTorque != prevMaxCalculatedTorque){ 
    lcd.clear();
    lcd.println("Max torque: ");
    lcd.print(maxCalculatedTorque);
    lcd.print(" oz-in");
    prevMaxCalculatedTorque = maxCalculatedTorque;
    delay(5);
  }
}

void displayRotations() {
  Serial.print("Rotations Set: ");
  Serial.println(filteredRotationCount);
  lcd.clear();
  lcd.print("Rotations: ");
  lcd.print(filteredRotationCount);
  delay(5);
}

void displaySpeed() {
  Serial.println(speedDisplay);
  lcd.clear();
  lcd.print(speedDisplay);
  delay(5);
}


