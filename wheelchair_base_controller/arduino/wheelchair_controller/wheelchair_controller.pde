#define THROTTLE_PWM_PIN 9
#define STEERING_PWM_PIN 10

#define THROTTLE_READ_PIN 1
#define STEERING_READ_PIN 0

#define MOTORS_ENABLED_READ 3
#define MOTORS_ENABLED_WRITE 2

#define NEUTRAL_COMMAND 127
#define MIN_1V 57
#define MAX_4V 197

#define ESTOP_WAIT_SECONDS 2

int throttle_command = NEUTRAL_COMMAND;
int steering_command = NEUTRAL_COMMAND;

// PID gains
const float ktp = 0.3;
const float kti = 0.04;
const float ksp = 0.6;
const float ksi = 0.04;

// PID state variables
int ti = 0.0;
int si = 0.0;

// timeout variable
unsigned long time;
#define MAX_PERIOD 66 // gives a minimum rate of 15.1Hz

void executeCommand() {
  // Run the throttle PID iteration
  int err = throttle_command-(analogRead(THROTTLE_READ_PIN)-7)/4; // Error term, analog offset is to get better neutral position
  int pTerm = (int) (ktp * err);
  ti = ti + err;
  int iTerm = (int) (kti * ti);
  int cmd = throttle_command+pTerm+iTerm;
  ti = ti + 20*(outputSaturation(cmd) - cmd); // Prevent integral windup
  analogWrite(THROTTLE_PWM_PIN, outputSaturation(cmd));
  
  /*Serial.print(throttle_command);
  Serial.print(" ");
  Serial.print(outputSaturation(cmd));
  Serial.print(" " );
  Serial.print(analogRead(THROTTLE_READ_PIN)/4);
  Serial.print("\n");*/
    
  // Run the steering PID iteration
  err = steering_command-(analogRead(STEERING_READ_PIN)-6)/4; // Error term, analog offset is to get better neutral position
  pTerm = (int) (ksp * err);
  si = si + err;
  iTerm = (int) (ksi * si);
  cmd = steering_command+pTerm+iTerm;
  si = si + 20*(outputSaturation(cmd) - cmd); // Prevent integral windup
  analogWrite(STEERING_PWM_PIN, outputSaturation(cmd));
}

int avoidJoyStickFault(int cmd){
  if(cmd < MIN_1V){
     return  MIN_1V;
  } else {
    if(cmd > MAX_4V){
      return MAX_4V;
    } 
  }
  return cmd;
}

int outputSaturation(int cmd){
    if(cmd < 0){
     return  0;
  } else {
    if(cmd > 255){
      return 255;
    } 
  }
  return cmd;
}

byte checkSum(byte b1, byte b2, byte b3){
    return ~(b1 + b2 + b3)+1; 
}

void setup() {
  // Change the analog reference to avoid possible issues with USB supplies
  analogReference(EXTERNAL);
  
  Serial.begin(115200);
  
  // Initialization
  analogWrite(THROTTLE_PWM_PIN, NEUTRAL_COMMAND);
  analogWrite(STEERING_PWM_PIN, NEUTRAL_COMMAND);
  pinMode(MOTORS_ENABLED_READ, INPUT);
  pinMode(MOTORS_ENABLED_WRITE, OUTPUT);
  
  
  Serial.flush();
  time = millis();
}

// This loop runs very fast, like 3000 Hz, so limit it down on the prints
// countMulti 100 produces 27 to 30Hz outputs, much easier.
int lCounter = 0;
int countMult = 100;

void loop() {
  // Check estop and write status
  byte motorsEnabled = digitalRead(MOTORS_ENABLED_READ);
  digitalWrite(MOTORS_ENABLED_WRITE, motorsEnabled);
  if(lCounter % countMult == 0){
    if(motorsEnabled == 0){
      Serial.print("0\n");
    } else {
      Serial.print("1\n");
    }
  }
  
  if(motorsEnabled == 0){
    throttle_command = avoidJoyStickFault(NEUTRAL_COMMAND);
    steering_command = avoidJoyStickFault(NEUTRAL_COMMAND);
    // Wait and do nothing until the motors are reenabled
    while(!digitalRead(MOTORS_ENABLED_READ)){
      digitalWrite(MOTORS_ENABLED_WRITE, 0);
      if(lCounter % countMult == 0){
        Serial.print("0\n");
      }
      executeCommand();
      lCounter++;
    }
    unsigned long eStopTime = millis();
    // Wait for n seconds for the center to be achieved
    while(millis() - eStopTime < 1000*ESTOP_WAIT_SECONDS){
      if(!digitalRead(MOTORS_ENABLED_READ)){  // Lost the estop, time to restart the process
        digitalWrite(MOTORS_ENABLED_WRITE, 0);
        break;
      }
      digitalWrite(MOTORS_ENABLED_WRITE, 1);
      if(lCounter % countMult == 0){
        Serial.print("0\n");
      }
      executeCommand();
      lCounter++;
    }
    // Finally made it out of this estop cycle, time to get rid of any built up serial mess
    Serial.flush();
  } else if(Serial.available()) {
    byte b[4];
    for(int i = 0; i < 4; i++){
      int lastVal = Serial.read();
      while(lastVal == -1){
        lastVal = Serial.read();
      }
      b[i] = lastVal;
    }
    
    // byte format is uint8_t throttle, uint8_t steering, utint8_t speed, uint8_t check
    byte tTemp = b[0];
    byte sTemp = b[1];
    byte spTemp = b[2];  // No longer used
    byte check = b[3];
    if(check == checkSum(tTemp, sTemp, spTemp)){
      throttle_command = avoidJoyStickFault(tTemp);
      steering_command = avoidJoyStickFault(sTemp);
      
      // Store the current time
      time = millis();

    } else { // Something's gone wrong in the serial, flush and try again
      Serial.flush();
      // Keep timing
    }
  } else if((millis() - time) > MAX_PERIOD){
    // We've missed the desired rate, time to stop
    throttle_command = avoidJoyStickFault(NEUTRAL_COMMAND);
    steering_command = avoidJoyStickFault(NEUTRAL_COMMAND);
  }
  executeCommand();
  lCounter++;
}
