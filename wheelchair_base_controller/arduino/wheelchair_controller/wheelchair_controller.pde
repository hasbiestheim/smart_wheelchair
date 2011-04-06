const int throttle_pwm_pin = 8;
const int steering_pwm_pin = 9;
const int speed_control_pwm_pin = 10;

const int throttle_read_pin = 1;
const int steering_read_pin = 0;


const int neutralCmd = 127;
const int min_1v = 57;
const int max_4v = 197;

int throttle_command = neutralCmd;
int steering_command = neutralCmd;
int speed_command = 249;

// PID gains
const float ktp = 0.1;
const float kti = 0.01;
const float ksp = 0.1;
const float ksi = 0.01;

// PID state variables
int ti = 0.0;
int si = 0.0;

void executeCommand() {
  // Run the throttle PID iteration
  int err = throttle_command-(analogRead(throttle_read_pin)+9)/4; // Error term, analog offset is to get better neutral position
  int pTerm = (int) (ktp * err);
  ti = ti + err;
  int iTerm = (int) (kti * ti);
  int cmd = throttle_command+pTerm+iTerm;
  ti = ti + 20*(outputSaturation(cmd) - cmd); // Prevent integral windup
  analogWrite(throttle_pwm_pin, outputSaturation(cmd));
  Serial.print(throttle_command);
  Serial.print("|");
  Serial.print(outputSaturation(cmd));
  Serial.print("|");
  Serial.print((analogRead(throttle_read_pin)+9)/4);
  Serial.print(" ");
  // Run the steering PID iteration
  err = steering_command-(analogRead(steering_read_pin)+9)/4; // Error term, analog offset is to get better neutral position
  pTerm = (int) (ksp * err);
  si = si + err;
  iTerm = (int) (ksi * si);
  cmd = steering_command+pTerm+iTerm;
  si = si + 20*(outputSaturation(cmd) - cmd); // Prevent integral windup
  analogWrite(steering_pwm_pin, outputSaturation(cmd));
  Serial.print(steering_command);
  Serial.print("|");
  Serial.print(outputSaturation(cmd));
  Serial.print("|");
  Serial.print((analogRead(steering_read_pin)+9)/4);
  Serial.println(" 249 1");
  // Always send max speed control.  I am not sure of their implementation.
  analogWrite(speed_control_pwm_pin, 249);
  
}

int avoidJoyStickFault(int cmd){
  if(cmd < min_1v){
     return  min_1v;
  } else {
    if(cmd > max_4v){
      return max_4v;
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

void setup() {
  Serial.begin(115200);
  Serial.println("      Wheelchair Control Program v0.5");
  Serial.println("Sent me bytes, throttle, steering, then speed, followed by zero.");
  
  // Initialization
  analogWrite(throttle_pwm_pin, neutralCmd);
  analogWrite(steering_pwm_pin, neutralCmd);
  analogWrite(speed_control_pwm_pin, 249);
  Serial.flush();
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
    int tTemp = b[0];
    int sTemp = b[1];
    int spTemp = b[2];
    int check = b[3];
    if(check == 0){
      throttle_command = avoidJoyStickFault(tTemp);
      steering_command = avoidJoyStickFault(sTemp);
      speed_command = outputSaturation(spTemp);
      //Serial.println(tTemp);
      //Serial.println(sTemp);
      //Serial.println(spTemp);
      //Serial.println(check);
      //Serial.println("-----------------");
      
      // Reset timing
    } else { // Something's gone wrong in the serial, flush and try again
      Serial.flush();
      // Keep timing
    }
  }
  executeCommand();
}
