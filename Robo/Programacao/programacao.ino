
#include <QTRSensors.h>

#define STANDBY          6           
#define MOTORLEFT_DIR_A  4          
#define MOTORLEFT_DIR_B  5         
#define MOTORLEFT_PWM    3          
#define MOTORRIGH_DIR_A  7    
#define MOTORRIGH_DIR_B  8           
#define MOTORRIGH_PWM    9           

#define NUM_SENSORS             8    
#define NUM_SAMPLES_PER_SENSOR  4   
#define EMITTER_PIN             13  


#define LEDPIN 10              

QTRSensorsAnalog qtra((unsigned char[])   {A0,A1, A2, A3, A4, A5, A6, A7}
, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);


unsigned int sensorValues[NUM_SENSORS];


unsigned long begintime;
unsigned long actualtime;

void setMotorLeft(int value)
{
  if ( value >= 0 )
  {
 
    digitalWrite(MOTORRIGH_DIR_A,HIGH);
    digitalWrite(MOTORRIGH_DIR_B,LOW);
  }
  else
  {
    
    digitalWrite(MOTORRIGH_DIR_A,LOW);
    digitalWrite(MOTORRIGH_DIR_B,HIGH);
    value *= -1;
  }


  analogWrite(MOTORRIGH_PWM,value);
}

void setMotorRigh(int value)
{  
  if ( value >= 0 )
  {
 
    digitalWrite(MOTORLEFT_DIR_A,HIGH);
    digitalWrite(MOTORLEFT_DIR_B,LOW);
  }
  else
  {
   
    digitalWrite(MOTORLEFT_DIR_A,LOW);
    digitalWrite(MOTORLEFT_DIR_B,HIGH);
    value *= -1;
  }    

  // Setea Velocidad
  analogWrite(MOTORLEFT_PWM,value);
}

void setMotors(int left, int righ)
{
  digitalWrite(STANDBY,HIGH);
  setMotorLeft(left);
  setMotorRigh(righ);
}

void setBrake(boolean left, boolean righ, int value)
{
   digitalWrite(STANDBY,HIGH);

  if ( left )
  {
    digitalWrite(MOTORRIGH_DIR_A,HIGH);
    digitalWrite(MOTORRIGH_DIR_B,HIGH);
    analogWrite (MOTORRIGH_PWM, value);
  }

  if ( righ )
  {
    digitalWrite(MOTORLEFT_DIR_A,HIGH);
    digitalWrite(MOTORLEFT_DIR_B,HIGH);
    analogWrite (MOTORLEFT_PWM, value);
  }
}

void imprimirsensores()
{
    Serial.print(" | ");
    Serial.print(sensorValues[0]); Serial.print(" | ");
    Serial.print(sensorValues[1]); Serial.print(" | ");
    Serial.print(sensorValues[2]); Serial.print(" | ");
    Serial.print(sensorValues[3]); Serial.print(" | ");
    Serial.print(sensorValues[4]); Serial.print(" | ");
    Serial.print(sensorValues[5]); Serial.print(" | ");
    Serial.print(sensorValues[6]); Serial.print(" | ");
    Serial.print(sensorValues[7]); Serial.println(" |");
}

#define CANT_P 1             
#define U_N    500             
boolean (*patrones[CANT_P])();  
int i = 0;                    

void avanzarSiguientePatron()
{
  ( i > CANT_P-1 ) ?  i = 0 : i++;
}

boolean detectarPatronActual()
{
  return patrones[i]();
}

void respuestaPatronActual()
{
  switch ( i )
  {
      case 0:         
          break;
  }
  avanzarSiguientePatron();
}

boolean patron_todonegro()
{
  return 
    ( sensorValues[0] < U_N ) && 
    ( sensorValues[1] < U_N ) && 
    ( sensorValues[2] < U_N ) && 
    ( sensorValues[3] < U_N ) && 
    ( sensorValues[4] < U_N ) && 
    ( sensorValues[5] < U_N ) && 
    ( sensorValues[6] < U_N ) && 
    ( sensorValues[7] < U_N );
}

boolean patron_todoblanco()
{
  return 
    sensorValues[0] > U_N && 
    sensorValues[1] > U_N && 
    sensorValues[2] > U_N && 
    sensorValues[3] > U_N && 
    sensorValues[4] > U_N && 
    sensorValues[5] > U_N && 
    sensorValues[6] > U_N && 
    sensorValues[7] > U_N;
}

void setup()
{
  pinMode(LEDPIN          ,OUTPUT);
  pinMode(STANDBY         ,OUTPUT);
  pinMode(MOTORRIGH_DIR_A ,OUTPUT);
  pinMode(MOTORRIGH_DIR_B ,OUTPUT);
  pinMode(MOTORRIGH_PWM   ,OUTPUT);
  pinMode(MOTORLEFT_DIR_A ,OUTPUT);
  pinMode(MOTORLEFT_DIR_B ,OUTPUT);
  pinMode(MOTORLEFT_PWM   ,OUTPUT);

  for ( int i=0; i<50; i++)     
  {
    digitalWrite(LEDPIN, HIGH);  
    delay(20);
    qtra.calibrate();           
    digitalWrite(LEDPIN, LOW); 
    delay(20);
  }

  digitalWrite(LEDPIN, LOW);     
  delay(5000);  
  setMotors(100, 100);
  delay(100);  
}

unsigned int position = 0;
int derivative = 0;
int proportional = 0;
int power_difference = 0;

int last_proportional;

float KP = 0.18;
float KD = 1.56;

int max = 230;

int RANGEBRAKE = 3500;

void loop()
{  
  position = qtra.readLine(sensorValues);
 
  proportional = ((int)position) - 3500;

  if ( proportional <= -RANGEBRAKE )
  {
    analogWrite(MOTORRIGH_PWM,20);
    digitalWrite(MOTORLEFT_DIR_A,HIGH);
    digitalWrite(MOTORLEFT_DIR_B,LOW); 
    digitalWrite(MOTORLEFT_DIR_A,LOW);
    digitalWrite(MOTORLEFT_DIR_B,HIGH);
    analogWrite(MOTORLEFT_PWM,20);
  }
  else 
  if ( proportional >= RANGEBRAKE )
  {
    analogWrite(MOTORRIGH_PWM,20);
    digitalWrite(MOTORLEFT_DIR_A,LOW);
    digitalWrite(MOTORLEFT_DIR_B,HIGH); 
    digitalWrite(MOTORLEFT_DIR_A,HIGH);
    digitalWrite(MOTORLEFT_DIR_B,LOW);
    analogWrite(MOTORLEFT_PWM,20);
  }
  

  derivative = proportional - last_proportional;

  last_proportional = proportional;

  int power_difference = ( proportional * KP ) + ( derivative * KD );

  
  if ( power_difference > max ) power_difference = max; 
  else if ( power_difference < -max ) power_difference = -max;
  
  
  ( power_difference < 0 ) ? setMotors(max+power_difference, max) : setMotors(max, max-power_difference);


  //Serial.print(","); Serial.println(power_difference);
  //imprimirsensores();
  

  //if ( detectarPatronActual() )
      //respuestaPatronActual();
}
