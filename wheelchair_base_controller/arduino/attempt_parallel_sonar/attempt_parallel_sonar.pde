#include <Messenger.h>

Messenger receiver;

const int throttle_pwm_pin = 4;
const int steering_pwm_pin = 3;
const int speed_control_pwm_pin = 2;
const int enable_assistant_dio_pin = 5;

const int neutral = 128;
const int min_1v = 51;
const int max_4v = 202;

int throttle_command = neutral;
int steering_command = neutral;
int speed_command = neutral;

void executeCommand() {
  while(receiver.available()) {
    throttle_command = avoidJoyStickFault(receiver.readInt());
    steering_command = avoidJoyStickFault(receiver.readInt());
    speed_command = limitSpeed(receiver.readInt());
  }
  analogWrite(throttle_pwm_pin, throttle_command);
  analogWrite(steering_pwm_pin, steering_command);
  analogWrite(speed_control_pwm_pin, speed_command);
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

int limitSpeed(int cmd){
    if(cmd < 0){
     return  0;
  } else {
    if(cmd > 255){
      return 255;
    } 
  }
  return cmd;
}

// Begin sonar counting section
#define SONARNUM 6
#define SONARSTARTPIN 22
#define TRIGGEROFFSET 14
#define SonarConversionuSperInch 147.0
#define MetersPerInch 0.0254

int lastSonarValues[SONARNUM];
unsigned long sonarStartTimes[SONARNUM];

void setup() {
  Serial.begin(115200);  
  receiver.attach(executeCommand);
  
  // Initialization
  analogWrite(throttle_pwm_pin, neutral);
  analogWrite(steering_pwm_pin, neutral);
  analogWrite(speed_control_pwm_pin, neutral);
  digitalWrite(enable_assistant_dio_pin, HIGH);
  
  for(int i=0; i<SONARNUM; i++){
    pinMode(i+SONARSTARTPIN, INPUT); // Set as input
    lastSonarValues[i] = 0; // Set initial value
    pinMode(i+SONARSTARTPIN+TRIGGEROFFSET, OUTPUT); // Set Trigger as output
    sonarStartTimes[i] = 0;
  }
}

unsigned long starty = 0;

void loop() {
  starty = micros();
  while(Serial.available()) {
    receiver.process(Serial.read());
    Serial.println(micros()-starty);
  }
  // Loop through sonars recording values
  for(int i=0; i<SONARNUM; i++){
    int thisRead = digitalRead(i+SONARSTARTPIN);
    int lastRead = lastSonarValues[i];
    if(thisRead > lastRead){ // This is a rising edge, so load the clock
      sonarStartTimes[i] = micros();
      lastSonarValues[i] = 1;
    } else if(lastRead > thisRead){ // This is a falling edge, so calculate and send the data up
      // Send packet  [sonarPort],[distanceInMeters]\n
      //Serial.print(i);
      //Serial.print(",");
      //Serial.print((float)(micros()-sonarStartTimes[i])/SonarConversionuSperInch*MetersPerInch);
      //Serial.println();
      // Set last value
      lastSonarValues[i] = 0;
    }
  }
}
