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
int period = 50;  // Refresh rate of the loop (50ms)
int second_count = 0; //count # of periods until you get to 1 second

/////////////////// PID Constants ///////////////////////
float kp = 0.1814;  //proportional gain
float ki = 0;  //integral gain
float kd = 20;  //derivative gain
float distance_setpoint = 7.8; // Distance to the middle of the bar in cm
float PID_p, PID_i, PID_d, PID_total, PID_scale;

int SERVO_MIN = 110;  // Lowest position (fully left)
int SERVO_MAX = 70;    // Highest position (fully right)
int SERVO_HALF = 92;

void setup() {
  Serial.begin(9600);  
  myservo.attach(9);
  myservo.write((SERVO_MAX+SERVO_MIN)/2);  // Move servo to its neutral position at startup
  pinMode(Analog_in, INPUT);  
  time = millis();
}

void loop() {
  if (millis() > time + period) {
    time = millis();    
    distance = get_dist(100); // Get distance measurement
    second_count = second_count +  1;

    // Calculate error
    distance_error = distance_setpoint - distance;

    // Proportional term
    PID_p = kp * distance_error;

    // Derivative term
    float dist_difference = distance_error - distance_previous_error;     
    PID_d = kd * (dist_difference / period);
    
    // Integral term (only accumulates if error is within a range)
    if (-30 < distance_error && distance_error < 30) { ///was +-3
      PID_i = PID_i + (ki * distance_error);
    } else {
      PID_i = 0;
    }

    // Compute total PID output
    PID_total = PID_p + PID_i + PID_d;
    
    // Map PID output correctly (inverted range since 180° is min and 0° is max). first range can be adjusted based on PID values and distance.  scale needs integers
    //if (PID_total <= 0) {PID_scale = map(PID_total, -45, 0, SERVO_MIN, SERVO_HALF);} //-45
    //else if (PID_total > 0) {PID_scale = map(PID_total, 0, 35, SERVO_HALF, SERVO_MAX); } //35
    PID_scale = map(PID_total, -7.5, 5, SERVO_MIN, SERVO_MAX);

    // Limit PID output within servo range (signs flipped due to servo flip) use constrain() function
    if (PID_scale > SERVO_MIN) { PID_scale = SERVO_MIN; } // Prevent exceeding leftmost position 
    if (PID_scale < SERVO_MAX) { PID_scale = SERVO_MAX; } // Prevent exceeding rightmost position

    // Write to servo
    myservo.write(PID_scale);  

    // Save previous error
    distance_previous_error = distance_error;

    //Debugging Output
    if (second_count == 20){
      Serial.print("Distance: "); Serial.print(distance); Serial.print(" cm | ");
      Serial.print("Error: "); Serial.print(distance_error); Serial.print(" | ");
      Serial.print("P: "); Serial.print(PID_p); Serial.print(" | ");
      Serial.print("I: "); Serial.print(PID_i); Serial.print(" | ");
      Serial.print("D: "); Serial.print(PID_d); Serial.print(" | ");
      Serial.print("PID Total: "); Serial.print(PID_total); Serial.print(" | ");
      Serial.print("Servo Pos: "); Serial.println(PID_scale);
      delay(200);
      second_count = 0;
    }
  }
}

// Function to get distance from the Sharp IR sensor
float get_dist(int n) {
  long sum = 0;
  for (int i = 0; i < n; i++) {
    sum += analogRead(Analog_in);
  }  
  float adc = sum / n;

  // Convert ADC value to distance (calibrated formula)
    float distance_in = 2.4168 + 22.95 * pow(0.9896,adc);
    float distance_cm = 7.552444 + 30.66397 * pow(0.95488,adc) ;



  return distance_cm;
}
