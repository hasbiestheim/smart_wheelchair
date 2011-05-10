// Constants
#define SONARNUM 6
#define SONARSTARTPIN 23 // Start pin on arduino
#define TRIGGEROFFSET -1 // Where the trigger is for the PWM pin (ie: 22 is trigger for PWM pin 23, so the offset is -1)
#define SonarConversionuSperInch 147.0 // Maxbotix uS of PWN per inch
#define MetersPerInch 0.0254 // Convert from inches to meter

float lastSonarValues[SONARNUM]; // holds the last PWM reading ie: 0 or 1
float sonarStartTimes[SONARNUM]; // holds the start time at which the PWM pin went high

void setup() {
  Serial.begin(115200);
  for(int i=0; i<SONARNUM; i++){
    pinMode(2*i+SONARSTARTPIN, INPUT); // Set as input
    lastSonarValues[i] = 0; // Set initial values
    sonarStartTimes[i] = 0;
    pinMode(2*i+SONARSTARTPIN+TRIGGEROFFSET, OUTPUT); // Set Trigger as output
    digitalWrite(2*i+SONARSTARTPIN+TRIGGEROFFSET, 0); // Start Sonars Disabled
  }
  // Initialize for first reading
  sonarStartTimes[0] = micros();
}

int sonI = 0; // Index variable for the loop

// Loop through sonars recording values one by one
void loop() {
  // Enable the current sonar
  digitalWrite(2*sonI+SONARSTARTPIN+TRIGGEROFFSET, 1);
  
  int thisRead = digitalRead(2*sonI+SONARSTARTPIN);
  int lastRead = lastSonarValues[sonI];
  if(thisRead > lastRead){ // This is a rising edge, so load the clock
    sonarStartTimes[sonI] = micros();
    lastSonarValues[sonI] = 1;
  } else if(lastRead > thisRead){ // This is a falling edge, so calculate and send the data up
    // Disable sonar
    digitalWrite(2*sonI+SONARSTARTPIN+TRIGGEROFFSET, 0);
  
    // Send packet  [sonarPort],[distanceInMeters]\n
    Serial.print(sonI+1);
    Serial.print(",");
    Serial.print((float)(micros()-sonarStartTimes[sonI])/SonarConversionuSperInch*MetersPerInch);
    Serial.println();
    // Set last value
    lastSonarValues[sonI] = 0;
    
    // Incremrent iterator and initialze for next sonar
    sonI = sonI + 1;
    if(sonI >= SONARNUM){
      sonI = 0;
    }
    sonarStartTimes[sonI] = micros();
    
    // Sleep for a whole round to avoid reflecting pings
    //delay(49);
    
    sonarStartTimes[sonI] = micros();
  } else if((micros()-sonarStartTimes[sonI]) > 50000){
    // Did not return in the alloted time, skip it.
    // Diable sonar
    digitalWrite(2*sonI+SONARSTARTPIN+TRIGGEROFFSET, 0);
    
    lastSonarValues[sonI] = 0;
    
    // Incremrent iterator
    sonI = sonI + 1;
    if(sonI >= SONARNUM){
      sonI = 0;
    }
    sonarStartTimes[sonI] = micros();
  }
}
