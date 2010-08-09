unsigned long pulseDuration = 0;
float distance = 0;
#define SonarConversionuSperInch 147.0
#define MetersPerInch 0.0254

void setup(){
  // Set up pins 2 through ? as INPUTs
  pinMode(0, INPUT);
  pinMode(1, INPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);
  // Set up pins 10 through ? as OUTPUTs (note that 14 and 15 are repurposed Analog Inputs (0 and 1)
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(14, OUTPUT);
  pinMode(15, OUTPUT);

  Serial.begin(115200);
}

void loop(){
  for(int i=2; i<=3; i++){
    digitalWrite(i+8,HIGH); // Start trigger
    pulseDuration = pulseIn(i, HIGH, 70000); // Record the next high duration for this sonar, timeout after 50ms
    digitalWrite(i+8,LOW); // Stop trigger
    
    // Compute distance in meters
    distance = (float)pulseDuration/SonarConversionuSperInch*MetersPerInch;
    
    // Send packet  [sonarPort],[distanceInMeters]\n
    Serial.print(i);
    Serial.print(",");
    Serial.print(distance);
    Serial.println();
  }
}
