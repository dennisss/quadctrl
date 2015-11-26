

// Motors at 5, 6, 10, 11

int pins[] = {5, 6, 10, 11};
#define n 4

#define PWM_LOW 20 // Zero throttle level
#define PWM_HIGH 230 // Full throttle level

void setup() {
  
  // The LED is mainly for debugging failures
  pinMode(LED_BUILTIN, OUTPUT);
  
  
  // Setup and set to 0
  for(int i = 0; i < n; i++){
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], LOW);
  }

 
  // Calibrate/arm 
  for(int i = 0; i < n; i++){
    analogWrite(pins[i], PWM_HIGH);
    delay(1000);
    analogWrite(pins[i], PWM_LOW);
    delay(1000); 
  }
  
  
  //for(int i = 0; i < 255; i++){
  //   analogWrite(10, i);
  //   delay(100);
  //}
 
 
  Serial.begin(115200);
  while(!Serial){ // Wait for serial to be configured
    ;
  }
  Serial.setTimeout(1000);
  
}

// Call when something bad seems to have happened.
// This will cut the motor signal and prevent a reset
void fail(){
  for(int i = 0; i < n; i++)
    digitalWrite(pins[i], LOW);
  
  while(1){
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
  } 
}

byte buffer[n];
void loop(){
  
  if(Serial.readBytes((char *)buffer, n) != n){
    fail(); // TODO: Only do this if a connection to the app has already been established
    return;
  }
  
  for(int i = 0; i < n; i++){
    int c = buffer[i];
    c = PWM_LOW + ((float)(c / 255.0))*(PWM_HIGH - PWM_LOW); // TODO: Make sure this does floating point
    analogWrite(pins[i], c);
  }
}

