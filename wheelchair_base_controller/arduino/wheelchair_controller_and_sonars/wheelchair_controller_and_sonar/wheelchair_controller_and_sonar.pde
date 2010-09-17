#include <Messenger.h>

Messenger receiver;

#define THROTTLEPIN 4
#define STEERINGPIN 3
#define SPEEDPIN 2
#define ENABLEPIN 5

#define NEUTRAL 127 // 127 represents 2.5V output
#define MINCMD 51 // 51 represents 1V output
#define MAXCMD 204 // 204 represents 4V output

#define SONARNUM 6
#define SONARSTARTPIN 22
#define TRIGGEROFFSET 14
#define SONARCONVERSIONUSPERINCH 147.0
#define METERSPERINCH 0.0254


// Start the output control section
int throttle_command = NEUTRAL;
int steering_command = NEUTRAL;
int speed_command = NEUTRAL;

void executeCommand() {
  while(receiver.available()) {
    throttle_command = avoidJoyStickFault(receiver.readInt());
    steering_command = avoidJoyStickFault(receiver.readInt());
    speed_command = limitSpeed(receiver.readInt());
  }
  Serial.print(throttle_command);
  Serial.print(steering_command);
  Serial.print(speed_command);
  analogWrite(THROTTLEPIN, throttle_command);
  analogWrite(STEERINGPIN, steering_command);
  analogWrite(SPEEDPIN, speed_command);
}

int avoidJoyStickFault(int cmd){
  if(cmd < MINCMD){
     return  MINCMD;
  } else {
    if(cmd > MAXCMD){
      return MAXCMD;
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
float distance = 0;
unsigned long pulseDuration = 0;

void setup() {
  Serial.begin(115200);  
  receiver.attach(executeCommand);
  
  // Initialization
  analogWrite(THROTTLEPIN, NEUTRAL);
  analogWrite(STEERINGPIN, NEUTRAL);
  analogWrite(SPEEDPIN, 0);
  digitalWrite(ENABLEPIN, HIGH);
  
  for(int i=0; i<SONARNUM; i++){
    pinMode(i+SONARSTARTPIN, INPUT); // Set as input
    pinMode(i+SONARSTARTPIN+TRIGGEROFFSET, OUTPUT); // Set Trigger as output
  }
}


void loop() {
  // Loop through sonars recording values
  for(int i=0; i<SONARNUM; i++){
    // First check for new command, this can delay us up to 6000 us while reading the serial
    while(Serial.available()) {
      receiver.process(Serial.read());
    }
    
    digitalWrite(i+SONARSTARTPIN+TRIGGEROFFSET,HIGH); // Start trigger
    pulseDuration = pulseIn(i+SONARSTARTPIN, HIGH, 60000); // Record the next high duration for this sonar, timeout after 60ms
    digitalWrite(i+SONARSTARTPIN+TRIGGEROFFSET,LOW); // Stop trigger
    
    // Compute distance in meters
    distance = (float)pulseDuration/SONARCONVERSIONUSPERINCH*METERSPERINCH;  
    
    // Send packet  [sonarPort],[distanceInMeters]\n
    //Serial.print(i);
    //Serial.print(",");
    //Serial.print(distance);
    //Serial.println();
  }
}
