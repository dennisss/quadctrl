
// NOTE: This currently only supports using a Pro Mini
// Motors at 3, 9, 10, 11

int pins[] = {10, 11, 3, 9};
#define n 4

#define PWM_LOW 2 // Zero throttle level
#define PWM_HIGH 253 // Full throttle level

void setup() {

  // The LED is mainly for debugging failures
  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(LED_BUILTIN, HIGH);

  // Setup and set to 0
  for(int i = 0; i < n; i++){
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], LOW);
  }


  // Calibrate/arm
  for(int i = 0; i < n; i++){
    analogWrite(pins[i], PWM_HIGH);
  }
  delay(1000);

  for(int i = 0; i < n; i++){
    analogWrite(pins[i], PWM_LOW);
  }
  delay(1000);


  Serial.begin(115200);
  while(!Serial){ // Wait for serial to be configured
    ;
  }
  Serial.setTimeout(100);

  digitalWrite(LED_BUILTIN, LOW);
}

// Call when something bad seems to have happened.
// This will cut the motor signal
void fail(){
  for(int i = 0; i < n; i++)
    analogWrite(pins[i], PWM_LOW);
}

byte buffer[n];
void loop(){

  if(Serial.readBytes((char *)buffer, n) != n){
    fail();
    return;
  }

  for(int i = 0; i < n; i++){
    int c = buffer[i];
    c = PWM_LOW + ((float)(c / 255.0))*(PWM_HIGH - PWM_LOW);
    analogWrite(pins[i], c);
  }
}

