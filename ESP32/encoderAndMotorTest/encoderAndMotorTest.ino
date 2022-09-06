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
const float distancePerTick = distancePerRev/ticksPerRev;
long lastCountRight = 0;
long lastCountLeft = 0;
int deltaRight, deltaLeft;
int potencia = 0;
void setup ()
{
 Serial.begin(9600);
 // Declaramos todos los pines como salidas
 pinMode (ENA, OUTPUT);
 pinMode (ENB, OUTPUT);
 pinMode (IN1, OUTPUT);
 pinMode (IN2, OUTPUT);
 pinMode (IN3, OUTPUT);
 pinMode (IN4, OUTPUT);
 // Enable the weak pull down resistors

 //ESP32Encoder::useInternalWeakPullResistors=DOWN;
  // Enable the weak pull up resistors
  ESP32Encoder::useInternalWeakPullResistors=UP;

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
}

long positionLeft  = -999;
long positionRight = -999;

void sendPower_A(int power){
 //Direccion motor A
 digitalWrite (IN1, LOW);
 digitalWrite (IN2, HIGH);
 analogWrite (ENA, power);
}

void sendPower_B(int power){
// //Direccion motor B
 digitalWrite (IN3, LOW);
 digitalWrite (IN4, HIGH);
 analogWrite (ENB, power);
}

//void Atras (int potencia)
//{
// 
// //Direccion motor A
// digitalWrite (IN1, HIGH);
// digitalWrite (IN2, LOW);
// analogWrite (ENA, potencia); //Velocidad motor A
//// Direccion motor B
// digitalWrite (IN3, HIGH);
// digitalWrite (IN4, LOW);
// analogWrite (ENB, potencia); //Velocidad motor B
//}
//void Adelante (int potencia)
//{
// //Direccion motor A
// digitalWrite (IN1, LOW);
// digitalWrite (IN2, HIGH);
// analogWrite (ENA, potencia); //Velocidad motor A
//// //Direccion motor B
// digitalWrite (IN3, LOW);
// digitalWrite (IN4, HIGH);
// analogWrite (ENB, potencia); //Velocidad motor B
//}
//void Turnleft_A (int potencia)
//{
// //Direccion motor A
// digitalWrite (IN1, HIGH);
// digitalWrite (IN2, LOW);
// analogWrite (ENA, potencia); //Velocidad motor A
//// //Direccion motor B
// digitalWrite (IN3, LOW);
// digitalWrite (IN4, HIGH);
// analogWrite (ENB, potencia); //Velocidad motor A
//}
//
//void TurnRight_A (int potencia){
// //Direccion motor A
// digitalWrite (IN1, LOW);
// digitalWrite (IN2, HIGH);
// analogWrite (ENA, potencia); 
//}
//
//void TurnRight_B(int potencia){
// //Direccion motor B
// digitalWrite (IN3, HIGH);
// digitalWrite (IN4, LOW);
// analogWrite (ENB, potencia); //Velocidad motor A
//}

void Parar (){
 //Direccion motor A
 digitalWrite (IN1, LOW);
 digitalWrite (IN2, LOW);
 analogWrite (ENA, 0); //Velocidad motor A
// //Direccion motor B
 digitalWrite (IN3, LOW);
 digitalWrite (IN4, LOW);
 analogWrite (ENB, 0); //Velocidad motor A
}

void loop (){
  //int rpmRef = 20;
  for(int i = 0; i < 256;i+=8){
    deltaRight = (int)encoder.getCount() - lastCountRight;
    deltaLeft = (int)encoder2.getCount() - lastCountLeft;
    lastCountRight = (int)encoder.getCount();
    lastCountLeft = (int)encoder2.getCount();
    float rpmRight = (float)(deltaRight*60)/ticksPerRev;
    float rpmLeft = (float)(deltaLeft*60)/ticksPerRev;
    sendPower_A(i);
    sendPower_B(i);
    Serial.println("RPM right = " + String(rpmRight) + " RPM left = " + String(rpmLeft) + " Duty Cycle = " + String(i));
    delay(1000);
    
  }
}
