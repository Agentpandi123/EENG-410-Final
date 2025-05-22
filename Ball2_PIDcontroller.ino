#include <Wire.h>
#include <Servo.h>

/////////////////////// Inputs/Outputs ///////////////////////
int Analog_in = A0;
Servo myservo;  // Servo object

//////////////////////// Variables /////////////////////////
int Read = 0;
float distance = 0.0;
float elapsedTime, time, timePrev;
float distance_previous_error, distance_error;
int total_time = 5000;  // total time before neutral
int n = 100;
int period = 100; //100 ms read period

/////////////////// PID Constants ///////////////////////


int SERVO_MIN = 50;  // Lowest position (fully left)
int SERVO_MAX = 110;    // Highest position (fully right)

void setup() {
  Serial.begin(9600);  
  myservo.attach(9);
  //myservo.write(SERVO_MAX);  // Move servo to its neutral position at startup
  pinMode(Analog_in, INPUT);  
  time = millis();
  myservo.write(SERVO_MIN);  // Move servo to its highest position at startup

}
void loop() {
  if (millis() > time + period) {
    time = millis();    

    long sum = 0;
    for (int i = 0; i < n; i++) {
      sum += analogRead(Analog_in);
    }  
    float adc = sum / n;

    // Convert ADC value to distance (calibrated formula). 0-5 v range.  10 bits of resolution
  
    float distance_in = 2.4168 + 22.95 * pow(0.9896,adc);
    float distance_cm = 7.552444 + 30.66397 * pow(0.95488,adc);

  
    //output of Distance sensor for measuring impulse
    Serial.println(distance_cm);
  //delay(250);
    if (time > total_time) {
      myservo.write(87); // after 5 sec move servo to neutral to simulate a step +55 is tuning the angle to true neutral.  need to adjust
    }
  }
}
