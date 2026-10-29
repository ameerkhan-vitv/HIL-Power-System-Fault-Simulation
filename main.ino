

#include <Wire.h>
#include <Adafruit_INA219.h>

const int RELAY_PIN = 4;     
const int MOTOR_IN1 = 5;     
const int MOTOR_IN2 = 6;     
const int RED_LED = 11;      
const int YEL_LED = 12;      
const int GRN_LED = 13;      

const float ACTUAL_BATTERY_VOLTAGE = 7.84; 
const int MAX_SAFE_PWM = (3.0 / ACTUAL_BATTERY_VOLTAGE) * 255; 

const float AMBIENT_WARNING_LOAD = 180.0; 
const float CRITICAL_TRIP_LOAD  = 350.0; 

const unsigned long RECLOSE_ISOLATION_TIME = 4000; 
const unsigned long RECLOSE_TEST_DURATION  = 1500; 

enum GridState { STATE_NORMAL, STATE_TRIPPED_LOCKOUT, STATE_RECLOSING_TEST };
GridState currentGridState = STATE_NORMAL;

Adafruit_INA219 ina219;
int baseTargetPWM = 55;    
int activeOutputPWM = 55;  
unsigned long stateTimer = 0;

bool faultLightningActive = false;
bool faultFallenTreeActive = false;
bool hazardInsulatorArcing = false;
bool modeBrownoutActive = false;
bool modeLoadShedding = false;
unsigned long transientTimer = 0;

void setup() {
  Serial.begin(115200);
  
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(YEL_LED, OUTPUT);
  pinMode(GRN_LED, OUTPUT);
  
  digitalWrite(RELAY_PIN, LOW); 
  digitalWrite(MOTOR_IN2, LOW); 
  
  if (!ina219.begin()) {
    while (1) { delay(10); }
  }
  
  analogWrite(MOTOR_IN1, 220); 
  delay(200); 
  analogWrite(MOTOR_IN1, baseTargetPWM);
  stateTimer = millis();
}

void loop() {
  unsigned long currentMillis = millis();

  if (faultLightningActive && (currentMillis - transientTimer > 400)) {
    faultLightningActive = false; 
  }
  
  if (Serial.available() > 0) {
    char inputChar = Serial.read();
    
    if (currentGridState == STATE_NORMAL) {
      if (inputChar >= '1' && inputChar <= '9') {
        int numericalCommand = inputChar - '0';
        baseTargetPWM = map(numericalCommand, 1, 9, 35, MAX_SAFE_PWM);
      }
      else if (inputChar == 'L' || inputChar == 'l') {
        faultLightningActive = true;
        transientTimer = currentMillis;
      }
      else if (inputChar == 'A' || inputChar == 'a') {
        hazardInsulatorArcing = !hazardInsulatorArcing; 
      }
      else if (inputChar == 'E' || inputChar == 'e') {
        faultFallenTreeActive = true;
      }
      else if (inputChar == 'B' || inputChar == 'b') {
        modeBrownoutActive = !modeBrownoutActive; 
      }
      else if (inputChar == 'S' || inputChar == 's') {
        modeLoadShedding = !modeLoadShedding; 
      }
    }
    
    if (inputChar == 'C' || inputChar == 'c') {
      faultFallenTreeActive = false;
      hazardInsulatorArcing = false;
      modeBrownoutActive = false;
      modeLoadShedding = false;
      faultLightningActive = false;
      if (currentGridState == STATE_TRIPPED_LOCKOUT) {
        currentGridState = STATE_NORMAL;
        digitalWrite(RELAY_PIN, LOW);
        analogWrite(MOTOR_IN1, 220); delay(200);
      }
    }
  }

  if (modeLoadShedding) {
    activeOutputPWM = 0; 
  } else if (modeBrownoutActive) {
    activeOutputPWM = baseTargetPWM / 2; 
  } else {
    activeOutputPWM = baseTargetPWM;     
  }

  float current_mA = 0.0;
  if (currentGridState != STATE_TRIPPED_LOCKOUT && !modeLoadShedding) {
    current_mA = ina219.getCurrent_mA();
    if (faultLightningActive || faultFallenTreeActive) {
      current_mA = 494.3; 
    } 
    else if (hazardInsulatorArcing) {
      current_mA = 140.0 + random(-40, 110); 
    }
  }

  digitalWrite(GRN_LED, LOW);
  digitalWrite(YEL_LED, LOW);
  digitalWrite(RED_LED, LOW);

  switch (currentGridState) {
    case STATE_NORMAL:
      if (modeLoadShedding) {
        digitalWrite(RELAY_PIN, LOW); 
        digitalWrite(YEL_LED, (currentMillis / 500) % 2); 
        analogWrite(MOTOR_IN1, 0);
      } else {
        digitalWrite(RELAY_PIN, LOW); 
        digitalWrite(MOTOR_IN2, LOW);
        analogWrite(MOTOR_IN1, activeOutputPWM);
        if (current_mA < AMBIENT_WARNING_LOAD) digitalWrite(GRN_LED, HIGH);
        else digitalWrite(YEL_LED, HIGH); 
      }
      if (current_mA >= CRITICAL_TRIP_LOAD && !modeLoadShedding) {
        digitalWrite(RELAY_PIN, HIGH); 
        analogWrite(MOTOR_IN1, 0);    
        currentGridState = STATE_TRIPPED_LOCKOUT;
        stateTimer = currentMillis;
      }
      break;

    case STATE_TRIPPED_LOCKOUT:
      digitalWrite(RED_LED, HIGH);   
      digitalWrite(RELAY_PIN, HIGH);  
      analogWrite(MOTOR_IN1, 0);
      if (currentMillis - stateTimer >= RECLOSE_ISOLATION_TIME) {
        digitalWrite(RELAY_PIN, LOW); 
        analogWrite(MOTOR_IN1, activeOutputPWM); 
        currentGridState = STATE_RECLOSING_TEST;
        stateTimer = currentMillis;
      }
      break;

    case STATE_RECLOSING_TEST:
      digitalWrite(RED_LED, (currentMillis / 150) % 2); 
      digitalWrite(RELAY_PIN, LOW); 
      analogWrite(MOTOR_IN1, activeOutputPWM);
      if (currentMillis - stateTimer >= RECLOSE_TEST_DURATION) {
        if (faultFallenTreeActive) {
          digitalWrite(RELAY_PIN, HIGH);
          analogWrite(MOTOR_IN1, 0);
          currentGridState = STATE_TRIPPED_LOCKOUT;
          stateTimer = currentMillis;
        } else {
          currentGridState = STATE_NORMAL;
        }
      }
      break;
  }

  Serial.print("#");
  Serial.print(current_mA, 1);         Serial.print(",");
  Serial.print((int)currentGridState); Serial.print(",");
  Serial.print(activeOutputPWM);
  Serial.println("%");

  delay(60); 
}
