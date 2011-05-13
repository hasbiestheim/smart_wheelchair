#define THROTTLE_PWM_PIN 9
#define STEERING_PWM_PIN 10
#define SPEED_CONTROL_PWM_PIN 11

#define THROTTLE_READ_PIN 1
#define STEERING_READ_PIN 0


#define NEUTRAL_COMMAND 127
#define MIN_1V 57
#define MAX_4V 197
#define DEFAULT_SPEED 255

int throttle_command = NEUTRAL_COMMAND;
int steering_command = NEUTRAL_COMMAND;
int speed_command = DEFAULT_SPEED;

// PID gains
const float ktp = 0.1;
const float kti = 0.01;
const float ksp = 0.1;
const float ksi = 0.01;

// PID state variables
int ti = 0.0;
int si = 0.0;

// timeout variable
unsigned long time;
#define MAX_PERIOD 66 // gives a minimum rate of 15.1Hz

void executeCommand() {
  // Run the throttle PID iteration
  int err = throttle_command-(analogRead(THROTTLE_READ_PIN)+9)/4; // Error term, analog offset is to get better neutral position
  int pTerm = (int) (ktp * err);
  ti = ti + err;
  int iTerm = (int) (kti * ti);
  int cmd = throttle_command+pTerm+iTerm;
  ti = ti + 20*(outputSaturation(cmd) - cmd); // Prevent integral windup
  analogWrite(THROTTLE_PWM_PIN, outputSaturation(cmd));
  
  // Run the steering PID iteration
  err = steering_command-(analogRead(STEERING_READ_PIN)+9)/4; // Error term, analog offset is to get better neutral position
  pTerm = (int) (ksp * err);
  si = si + err;
  iTerm = (int) (ksi * si);
  cmd = steering_command+pTerm+iTerm;
  si = si + 20*(outputSaturation(cmd) - cmd); // Prevent integral windup
  analogWrite(STEERING_PWM_PIN, outputSaturation(cmd));
  
  // Always send max speed control.  I am not sure of their implementation.
  analogWrite(SPEED_CONTROL_PWM_PIN, DEFAULT_SPEED);
  
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
  Serial.begin(115200);
  
  // Initialization
  analogWrite(THROTTLE_PWM_PIN, NEUTRAL_COMMAND);
  analogWrite(STEERING_PWM_PIN, NEUTRAL_COMMAND);
  analogWrite(SPEED_CONTROL_PWM_PIN, DEFAULT_SPEED);
  Serial.flush();
  time = millis();
}
  
void loop() {
  if(Serial.available()) {
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
    byte spTemp = b[2];
    byte check = b[3];
    if(check == checkSum(tTemp, sTemp, spTemp)){
      throttle_command = avoidJoyStickFault(tTemp);
      steering_command = avoidJoyStickFault(sTemp);
      speed_command = outputSaturation(spTemp);
      
      // Store the current time
      time = millis();

    } else { // Something's gone wrong in the serial, flush and try again
      Serial.flush();
      // Keep timing
    }
  } else {
    if((millis() - time) > MAX_PERIOD){
      // We've missed the desired rate, time to stop
      throttle_command = avoidJoyStickFault(NEUTRAL_COMMAND);
      steering_command = avoidJoyStickFault(NEUTRAL_COMMAND);
      speed_command = outputSaturation(DEFAULT_SPEED);
    }
  }
  executeCommand();
}
