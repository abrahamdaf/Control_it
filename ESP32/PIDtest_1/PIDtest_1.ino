#include <analogWrite.h>
#include <ESP32Encoder.h>

#define  IN3 33
#define  IN4 32
#define  ENA 27
#define  IN1 26
#define  IN2 25
#define  ENB 14
#define ENC1A 39
#define ENC1B 36
#define ENC2A 34
#define ENC2B 35

ESP32Encoder encoder;
ESP32Encoder encoder2;
// timer and flag for example, not needed for encoders
unsigned long encoder2lastToggled;
bool encoder2Paused = false;
const int ticksPerRev = 1800;
const float distancePerRev = 13.73;
const float distancePerTick = distancePerRev / ticksPerRev;
long lastCountRight = 0;
long lastCountLeft = 0;
int deltaRight, deltaLeft;

// Controler constants
double KpL = 4.48896448205661, KiL = 10.5072974958874, KdL = 0.130930013043834;
double KpR = 1.50311341005131, KiR = 5.79952212201727, KdR = 0.0499774666446061;

// Initialize variables with initial value
unsigned long currentTime  = 0;
unsigned long previousTime = 0;
double elapsedTime = 0.0;
double error_R = 0.0;
double error_L = 0.0;
double lastError_R = 0.0;
double lastError_L = 0.0;
double cumError_R = 0.0; 
double cumError_L = 0.0;
double rateError_L = 0.0;
double rateError_R = 0.0;
double Outputs[2];
int rpmRef = 50;


void setup (){
  Serial.begin(9600);
  // Set all pin as output
  pinMode (ENA, OUTPUT);
  pinMode (ENB, OUTPUT);
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode (IN3, OUTPUT);
  pinMode (IN4, OUTPUT);
  
  // Enable the weak pull down resistors

  //ESP32Encoder::useInternalWeakPullResistors=DOWN;
  // Enable the weak pull up resistors
  ESP32Encoder::useInternalWeakPullResistors = UP;

  // use pin 19 and 18 for the first encoder
  encoder.attachHalfQuad(ENC1A, ENC1B);
  // use pin 17 and 16 for the second encoder
  encoder2.attachHalfQuad(ENC2A, ENC2B);

  // set starting count value after attaching
  encoder.clearCount();

  // clear the encoder's raw count and set the tracked count to zero
  encoder2.clearCount();
  Serial.println("Encoder Start = " + String((int32_t)encoder.getCount()));
  // set the lastToggle
  encoder2lastToggled = millis();
  previousTime = millis();
  Stop();
  delay(1000);
  Outputs[0] = 50;
  Outputs[1] = 50;
}

long positionLeft  = -999;
long positionRight = -999;

void sendPower_Left(int power) {
  //Direccion motor A
  digitalWrite (IN1, LOW);
  digitalWrite (IN2, HIGH);
  analogWrite (ENA, power);
}

void sendPower_Right(int power) {
  // //Direccion motor B
  digitalWrite (IN3, LOW);
  digitalWrite (IN4, HIGH);
  analogWrite (ENB, power);
}

void Stop () {

  digitalWrite (IN1, LOW);
  digitalWrite (IN2, LOW);
  analogWrite (ENA, 0); 
  digitalWrite (IN3, LOW);
  digitalWrite (IN4, LOW);
  analogWrite (ENB, 0); 
}

void PID(int ref, float R, float L) {
  currentTime = millis();
  elapsedTime = ((double)(currentTime - previousTime)) / (1000);
  // Calculate current error
  error_R = ref - R;
  error_L = ref - L;
  // Cumulative error for integral discretation
  cumError_R += error_R * elapsedTime;
  cumError_L += error_L * elapsedTime;
  // Slope of error 
  rateError_R = (error_R - lastError_R) / elapsedTime;
  rateError_L = (error_L - lastError_L) / elapsedTime;
  // Evaluate PID formula
  double outputL = (KpL * error_L + KiL * cumError_L + KdL * rateError_L); // This controls velocity
  double outputR = (KpR * error_R + KiR * cumError_R + KdR * rateError_R); // This controls velocity
  Serial.println("Error Left " + String(error_L) + " Error Right " + String(error_R));
  // Assign lef and right output value
  Outputs[0] = outputL;
  Outputs[1] = outputR;
  // Refresh last error for each side
  lastError_R = error_R;
  lastError_L = error_L;
  previousTime = currentTime;
}

void loop () {
  if(Serial.available() && Serial.read()!= rpmRef){
    int rpmRef = Serial.read(); // Can be withing the range of 50-80 RPM
  }
  //Read encoder ticks and calculate RPM
  deltaRight = (int)encoder.getCount() - lastCountRight;
  deltaLeft = (int)encoder2.getCount() - lastCountLeft;
  lastCountRight = (int)encoder.getCount();
  lastCountLeft = (int)encoder2.getCount();
  float rpmRight = (float)(deltaRight * 600) / ticksPerRev;
  float rpmLeft = (float)(deltaLeft * 600) / ticksPerRev;
  // Call PID control function and determine outputs
  PID(rpmRef, rpmRight, rpmLeft);
  sendPower_Left((int)Outputs[0]);
  sendPower_Right((int)Outputs[1]);
  Serial.println("Ref"+String(rpmRef)+"RPM right = " + String(rpmRight) + " RPM left = " + String(rpmLeft));
  delay(100);
}
