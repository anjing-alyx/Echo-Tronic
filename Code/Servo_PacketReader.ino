#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SERVOMIN 125  // Minimum pulse length (0 degrees)
#define SERVOMAX 625  // Maximum pulse length (180 degrees)
#define N_SERVOS 11   // Total servos

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(50); // Servos ~60 Hz

  // Initialize all servos to neutral 90°
  for(int i=0; i<N_SERVOS; i++){
    pwm.setPWM(i, 0, angleToPulse(90));
  }
}

// Convert angle (0-180) to PWM pulse
int angleToPulse(int angle) {
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

void loop() {
  if(Serial.available()){
    static String buffer = "";
    char c = Serial.read();

    if(c == '<'){
      buffer = ""; // start marker
    } 
    else if(c == '>'){
      // end marker -> parse packet
      int count = 0;
      char *token = strtok((char*)buffer.c_str(), ",");
      while(token != NULL && count < N_SERVOS){
        int angle = atoi(token);
        pwm.setPWM(count, 0, angleToPulse(angle));
        token = strtok(NULL, ",");
        count++;
      }
    } 
    else {
      buffer += c;
    }
  }
}

/* Servo Mapping (Python packet indices):
   S0: ReyeX       -> Channel 0
   S1: LeyeX       -> Channel 1
   S2: ReyeY       -> Channel 2
   S3: LeyeY       -> Channel 3
   S4: Rblink      -> Channel 4
   S5: Lblink      -> Channel 5
   S6: Rebrow      -> Channel 6
   S7: Lebrow      -> Channel 7
   S8: Right Mouth -> Channel 8
   S9: Left Mouth  -> Channel 9
   S10: Nose       -> Channel 10
*/
