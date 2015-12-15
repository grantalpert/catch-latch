#include <Servo.h>

////////////////////////////////////////////////
///                                          ///
///            VARIABLE CREATION             ///
///                                          ///
////////////////////////////////////////////////
// Debounce variables
boolean voltageTerminal = LOW; // Initial value of terminal
boolean pinBefore = LOW;
boolean pinCurrent = LOW;

// Pin variable creation
int pinSwitch = 8; // Button input pin
int terminal = 5; // Connected to the Vcc of the motor driver
const int IN1 = 6; // Motor driver input 1
const int IN2 = 11; // Motor driver input 2
const int sv = 3; // Servo output pin
const int ir = A0; // IR input pin

// Control variables
// first kp = 10, kd = 20
// p = 5, d = 20
// p = 5, d = 30
const int Ki = 0; // Integral control gain
const int Kp = 15; // Proportional control gain
const int Kd = 2; // Derivative control gain
const int d_ref = 18;// Desired distance from sensor (cm)

// Constraining variables
const int latch_max = 19; // Maximum distance to check for latching (cm)
const int latch_min = 15; // Minimum distance to check for latching (cm)
const int motor_max = 240; // Maximum pwm output to motor
const int motor_min = 70; // Minimum pwm output to motor
const int dist_max = 40; // Maximum distance to do control (cm)
const int dist_min = 6; // Minimum distance to do control (cm)
const int rev_max = 200; // Maximum pwm output for reverse

// Initial errors and outputs
int vo = 0; // Initial control output
int eI = 0; // Initial integral error
int e = 0; // Initial error
int eD = 0; // Initial derivative error

// Other variables
const int timeout = 20; // Time before timeout (s)
int i = 0;
int j = 0;
int k = 0;
int window = 25; // Set window size (ms)
int t1 = 0; // Initialize timing variables
int t2 = 0;
int latched = 0; // State of latching, 1 for latched
int prev_dist[5];
int rev = 0;

Servo servo; // Creates servo object

////////////////////////////////////////////////
///                                          ///
///              INITIAL SETUP               ///
///                                          ///
////////////////////////////////////////////////

void setup() {
  // Write pin modes
  pinMode(pinSwitch, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(terminal, OUTPUT);
  pinMode(ir, INPUT);

  // Attach servo object and write initial position (degrees)
  servo.attach(sv);
  servo.write(1);

  // Initialize serial port
  Serial.begin(9600);
}

////////////////////////////////////////////////
///                                          ///
///               DEBOUNCE                   ///
///                                          ///
////////////////////////////////////////////////

// Switch debounce function
boolean debounce (boolean last)
{
  boolean current = digitalRead(pinSwitch);
  // Checks if current pin state is different than last
  if (current != last)
  {
    delay(10); // If so, wait 10 ms to make sure it's still different
  }
  return digitalRead(pinSwitch); // Returns new state
}

////////////////////////////////////////////////
///                                          ///
///               MAIN LOOP                  ///
///                                          ///
////////////////////////////////////////////////

void loop() {
  //  int ct = millis(); // Turn off output after 15 seconds
  //  if (ct > timeout * 1000) {
  //    voltageTerminal = LOW;
  //    digitalWrite(terminal, voltageTerminal);
  //  }

  // Turn on or off the motor driver (by controlling the Vcc input)
  pinCurrent = debounce(pinBefore);
  // Toggle power after button press
  if (pinCurrent == HIGH && pinBefore == LOW) {
    voltageTerminal = !voltageTerminal;
  }
  pinBefore = pinCurrent; // Change state
  digitalWrite(terminal, voltageTerminal);

  // Calculating the distance from IR sensor
  double sum = 0;
  // Gather data for window ms
  for (int i = 0; i < window; i++) {
    sum = sum + analogRead(ir);
    delay(1);
  }
  double avg = sum / window;
  double avg_volt = avg * 5 / 1024;
  int dist = 11.65 * pow(avg_volt, -1.012); // Convert avg_volt to dist
  prev_dist[j] = dist; // Add this distance to previous distance array;
  j++; // Increment

  // This sets the value of latched variable
  if (dist < latch_max && dist > latch_min) {
    latched = do_latch(prev_dist, dist);
  }
  else {
    latched = 0;
  }

  // If it should latch, perform the latch function
  if (latched == 1) {
    latch(IN1, IN2);
  }
  else {
    motor(dist, IN1, IN2); // If not, keep doing control
  }

  // Reset the index of previous distance
  if (j >= 5) {
    j = 0;
  }
}

////////////////////////////////////////////////
///                                          ///
///              LATCH CHECK                 ///
///                                          ///
////////////////////////////////////////////////

// Checks the previous 5 distances to see if it should latch
bool do_latch(int prev_dist[5], int dist) {
  double sum_d = 0;
  for (int k = 0; k < 5; k++) {
    sum_d = sum_d + prev_dist[k];
    int d_avg = sum_d / 5; // Average the previous 5 distances
    if (d_avg == dist) { // If the average == the current distance
      return 1; // Latch!
    }
  }
}

////////////////////////////////////////////////
///                                          ///
///              MOTOR OUTPUT                ///
///                                          ///
////////////////////////////////////////////////

// Controls output to h-bridge
void motor(int dist, int IN1, int IN2) {
  // if the forward car is far away give it max power
  if (dist >= dist_max) {
    analogWrite(IN1, motor_max);
    analogWrite(IN2, 0);
  }
  // Control algorithm
  else if (dist > dist_min && dist < dist_max) {
    // Control equations
    eD = (dist - d_ref) - e;
    e = dist - d_ref;
    eI = eI + e; // Unused
    vo = Ki * eI + Kp * e + Kd * eD;
    // Brake motor if the car is too close
    if (vo < motor_min) {
      rev = motor_min - vo;
      rev = constrain(rev, 0, rev_max);
      analogWrite(IN1, 0);
      analogWrite(IN2, rev);
    }
    else {
      vo = constrain(vo, motor_min, motor_max);
      analogWrite(IN1, vo);
      analogWrite(IN2, 0);
    }
  }
  else {
    // Brake the motor if distance less than 6 cm
    analogWrite(IN1, 255);
    analogWrite(IN2, 255);
  }
}

////////////////////////////////////////////////
///                                          ///
///               LATCHING!                  ///
///                                          ///
////////////////////////////////////////////////

// Function which latches and brakes the control car
void latch(int IN1, int IN2) {
  //Latch downwards
  servo.write(15);
  delay(300);

  // Reverse
  analogWrite(IN1, 0);
  analogWrite(IN2, 155);
  delay(300);

  // Turn off motor output
  voltageTerminal = LOW;
  pinBefore = LOW;
  pinCurrent = LOW;
}

