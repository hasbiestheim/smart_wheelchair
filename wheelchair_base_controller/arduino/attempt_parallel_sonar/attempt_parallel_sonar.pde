// Begin sonar counting section
#define SONARNUM 6
#define SONARSTARTPIN 23
#define TRIGGEROFFSET -1
#define SonarConversionuSperInch 147.0
#define MetersPerInch 0.0254

float lastSonarValues[SONARNUM];
float sonarStartTimes[SONARNUM];

void setup() {
  Serial.begin(115200);
  for(int i=0; i<SONARNUM; i++){
    pinMode(2*i+SONARSTARTPIN, INPUT); // Set as input
    lastSonarValues[i] = 2; // Set initial values
    sonarStartTimes[i] = 0;
    pinMode(2*i+SONARSTARTPIN+TRIGGEROFFSET, OUTPUT); // Set Trigger as output
    digitalWrite(2*i+SONARSTARTPIN+TRIGGEROFFSET, 1); // Enable Sonars
  }
}

void loop() {
  // Loop through sonars recording values
  int i = 0;
  for(int i=0; i<SONARNUM; i++){
    int thisRead = digitalRead(2*i+SONARSTARTPIN);
    int lastRead = lastSonarValues[i];
    if(thisRead > lastRead){ // This is a rising edge, so load the clock
      sonarStartTimes[i] = micros();
      lastSonarValues[i] = 1;
    } else if(lastRead > thisRead){ // This is a falling edge, so calculate and send the data up
      if(lastRead < 2){ // Ignore the bootup count
        // Send packet  [sonarPort],[distanceInMeters]\n
        Serial.print(i);
        Serial.print(",");
        Serial.print((float)(micros()-sonarStartTimes[i])/SonarConversionuSperInch*MetersPerInch);
        Serial.println();
      }
      // Set last value
      lastSonarValues[i] = 0;
    }
  }
}
