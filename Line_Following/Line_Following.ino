
#include <QTRSensors.h>  // Pololu QTR Library 


//Defining line sensor
#define NUM_SENSORS   4     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   2    // emitter is controlled by digital pin 2


//Line sensor declarations
//Sensors 1 through 6 are connected to digital pins 2 through 10, respectively (pin 3 is skipped and used for motor control)
QTRSensorsRC qtrrc((unsigned char[]) {4, 5, 6, 7}, NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];

int pwm_a = 3;  //PWM control on digital pin 3
int pwm_b = 11;  //PWM control on digital pin 11
int dir_a = 9;  //direction control on digital pin 9
int dir_b = 10;  //direction control on digital pin 10



// PID loop variables
float error=0;
float lastError=0;
float PV =0 ;
float kp = 0;
float ki = 0;
float kd =0;
int m1Speed=0;
int m2Speed=0;
int line_position=0;
int motorspeed=0;
int max_speed=250;

//Pin which triggers ultrasonic sound
const int pingPin = 13;
 
//Pin which delivers time to receive echo using pulseIn()
int inPin = 12;

//Range in cm to detect object
int safeZone = 4;

int redLed = 2;

void setup()
{
  pinMode(pwm_a, OUTPUT);  //Set control pins to be outputs
  pinMode(pwm_b, OUTPUT);
  pinMode(dir_a, OUTPUT);
  pinMode(dir_b, OUTPUT);

  analogWrite(pwm_a, 0);  //set both motors to run at (100/255 = 39)% duty cycle (slow)
  analogWrite(pwm_b, 0);
  

  
  Serial.begin(115200);   
  
  
  Serial.println("Calibrating sensor");
  // Calibrate line sensor
  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }
  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration
  
  // print calibration results
  int i;
  for (i = 0; i < 250; i++)  // the calibration will take a few seconds
  {
    qtrrc.calibrate(QTR_EMITTERS_ON);
    delay(20);
  }
  // print the calibration minimum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  
  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  Serial.println("Calibration Complete");
  delay(1000);
  
}

void loop()
{

  // Line sensor values
  // Read calibrated sensor values and obtain a measure of the line position from 0 to 5000
  // To get raw sensor values, call:
  //  qtrrc.read(sensorValues); 
  unsigned int line_position = qtrrc.readLine(sensorValues);

  // Print the sensor values as numbers from 0 to 1000, where 0 means maximum reflectance and
  // 1000 means minimum reflectance, followed by the line position
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
 //   Serial.print(sensorValues[i]);
 //   Serial.print('\t');
  }
  //Serial.println(); // uncomment this line if you are using raw values
  Serial.println(line_position); // comment this line out if you are using raw values
  
  //delay(250);
 
//raw duration in milliseconds, cm is the converted amount into a distance
  long duration, cm;
 
  //initializing the pin states
  pinMode(pingPin, OUTPUT);
  pinMode(redLed, OUTPUT);  
 
  //sending the signal, starting with LOW for a clean signal
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);
 
  //setting up the input pin, and receiving the duration in
  //microseconds for the sound to bounce off the object infront
  pinMode(inPin, INPUT);
  duration = pulseIn(inPin, HIGH);
 
  // convert the time into a distance
  cm = microsecondsToCentimeters(duration);
  
  if (cm > safeZone)
  {
    
    follow_line(line_position);
  }
  else
  {
    analogWrite(pwm_a, 0);
    analogWrite(pwm_b, 0);
    digitalWrite(redLed, HIGH);
    
  }

 
   
  
  delay(10);


}  // end loop


//Line following subroutine
// PD Control


void follow_line(int line_position)  //follow the line

{


   error = (float)line_position - 1500;
 
  // Set the motor speed based on proportional and derivative PID terms
  // kp is the a floating-point proportional constant (maybe start with a value around 0.5)
  // kd is the floating-point derivative constant (maybe start with a value around 1)
  // When doing PID, signs should be correct, else control loop is unstable
 
  kp=0.5;
  kd=1;
   
  PV = kp * error + kd * (error - lastError);
  lastError = error;
  
calc_turn();
  

} // end follow line  
  
  
long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}    

void calc_turn()
{
    if (PV > 250)
  {
    PV = 250;
  }
  
  if (PV < -250)
  {
    PV = -250;
  }
 
  m1Speed = max_speed + PV;
  m2Speed = max_speed - PV;
  
  if (m1Speed > max_speed){
   m1Speed = max_speed; 
  }
  if (m2Speed > max_speed){
   m2Speed = max_speed; 
  }
  if (m1Speed < 0){
   m1Speed = 0; 
  }
   if (m2Speed < 0){
   m2Speed = 0; 
  }
  analogWrite(pwm_a, m1Speed);
  analogWrite(pwm_b, m2Speed);
  
  // If error_value is less than zero calculate right turn speed values   
  if (PV < 0)
  {
  digitalWrite(dir_a, HIGH); 
  digitalWrite(dir_b, LOW);  
  

  Serial.println("Rotate Right\n");
  
  }
  // Iferror_value is greater than zero calculate left turn values
  else
  {
  digitalWrite(dir_a, LOW); 
  digitalWrite(dir_b, HIGH);  
  
 Serial.println("Rotate Left\n");
  }
}

