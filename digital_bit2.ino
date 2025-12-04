// Ultrasonic sensors
const int trig1 = 2;
const int echo1 = 3;
const int trig2 = 4;
const int echo2 = 5;

// Fan PWM
const int fanPWM = 9;

// Status byte pins to PLC (detection + heartbeat)
const int statusPins[8] = {6,7,8,14,15,16,17,18}; // Bits 0-7

// Inputs FROM PLC (4-bit fan speed command)
const int speedBitPins[4] = {10,11,12,13}; // LSB → MSB

// Optional outputs back to PLC (fan speed feedback)
const int outBitPins[4] = {A0,A1,A2,A3}; // LSB → MSB

bool systemOK = true;
unsigned long lastGoodRead = 0;

// --- Setup ---
void setup() {
  // Ultrasonic
  pinMode(trig1, OUTPUT); pinMode(echo1, INPUT);
  pinMode(trig2, OUTPUT); pinMode(echo2, INPUT);

  // Status byte outputs
  for(int i=0;i<8;i++) pinMode(statusPins[i], OUTPUT);

  // Speed input pins from PLC
  for(int i=0;i<4;i++) pinMode(speedBitPins[i], INPUT);

  // Fan PWM
  pinMode(fanPWM, OUTPUT);

  digitalWrite(statusPins[2], HIGH); // Start heartbeat HIGH
  Serial.begin(9600);
}

// --- Functions ---
int readDistance(int trigPin, int echoPin){
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  if(duration==0) return -1;
  return duration*0.034/2;
}

int readPLCSpeed(){
  int value=0;
  for(int i=0;i<4;i++){
    value |= digitalRead(speedBitPins[i]) << i;
  }
  return value; // 0-15
}

void sendSpeedToPLC(int speedLevel){
  for(int i=0;i<4;i++){
    digitalWrite(outBitPins[i], (speedLevel>>i)&1);
  }
}

void sendStatusByte(bool ballDetected, bool setpointDetected, bool heartbeat){
  byte status=0;
  if(ballDetected) status |= 0b00000001; // Bit 0
  if(setpointDetected) status |= 0b00000010; // Bit 1
  if(heartbeat) status |= 0b00000100; // Bit 2
  // Bits 3-7 reserved
  for(int i=0;i<8;i++){
    digitalWrite(statusPins[i], (status>>i)&1);
  }
}

// --- Main Loop ---
void loop() {
  // Read sensors
  int ballDist = readDistance(trig1, echo1);
  int setpointDist = readDistance(trig2, echo2);

  // System health
  if(ballDist==-1 || setpointDist==-1) systemOK=false;
  else { systemOK=true; lastGoodRead=millis(); }
  if(millis()-lastGoodRead>2000) systemOK=false;

  bool ballDetected = (ballDist>0 && ballDist<50);
  bool setpointDetected = (setpointDist>0 && setpointDist<50);

  // Read speed from PLC
  int speedLevel = readPLCSpeed(); // 0-15
  int fanPWMValue = map(speedLevel,0,15,0,255);
  analogWrite(fanPWM, fanPWMValue);

  // Send status byte to PLC
  sendStatusByte(ballDetected,setpointDetected,systemOK);

  /*Debug
  Serial.print("Ball: "); Serial.print(ballDist);
  Serial.print(" cm | Setpoint: "); Serial.print(setpointDist);
  Serial.print(" cm | Heartbeat: "); Serial.print(systemOK?"HIGH":"LOW");
  Serial.print(" | Speed: "); Serial.print(speedLevel);
  Serial.print(" | PWM: "); Serial.println(fanPWMValue);*/

  delay(100);
}
