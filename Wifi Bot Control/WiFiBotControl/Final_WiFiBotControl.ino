#include <Adafruit_CC3000.h>
#include <ccspi.h>
#include <SPI.h>
#include <string.h>
#include "utility/debug.h"
#include <stdlib.h>
#include <StopWatch.h>

StopWatch MySW;
StopWatch SWarray[5];

String result;
int motorCommand[5];
// will be used to parse values from the json file on the server:  {"mode":"1","xval":"0","yval":"0","udlr":"","cmdVal":""}
int resultLength;
float leftSpeed, rightSpeed;

int counter = 0;
int port = 80;

#define leftMtrSpdPin 6  //PWM pin
#define rightMtrSpdPin 9 //PWM pin
#define leftMtrDirPin 7  
#define rightMtrDirPin 4 
#define LEDStatPin 2
#define LEDDataReceivedPin A5

// These are the interrupt and control pins
#define ADAFRUIT_CC3000_IRQ   3  // MUST be an interrupt pin!
// These can be any two pins
#define ADAFRUIT_CC3000_VBAT  5
#define ADAFRUIT_CC3000_CS    10
// Use hardware SPI for the remaining pins
// On an UNO, SCK = 13, MISO = 12, and MOSI = 11
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT,
                                         SPI_CLOCK_DIVIDER); // you can change this clock speed

#define WLAN_SSID       "HUAWEI-mFui"           // cannot be longer than 32 characters!
#define WLAN_PASS       "xxxxxxx"               // enter the password of the Wi-Fi router
// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
#define WLAN_SECURITY   WLAN_SEC_WPA2

#define IDLE_TIMEOUT_MS  3000      // Amount of time to wait (in milliseconds) with no data 
                                   // received before closing the connection.  If you know the server
                                   // you're accessing is quick to respond, you can reduce this value.
const unsigned long
dhcpTimeout     = 60L * 1000L, // Max time to wait for address from DHCP
connectTimeout  = 15L * 1000L, // Max time to wait for server connection
responseTimeout = 15L * 1000L; // Max time to wait for data from server                                   

// Page to fetch
#define WEBSITE      "192.168.100.2"
#define WEBPAGE      "/server.php"



uint32_t t;

// Local server IP, port, and repository (change with web server settings)

uint32_t ip = cc3000.IP2U32(192,168,100,2);  //Looking to webserver 

void setup(void)
{
  
  Serial.begin(115200);
  
  SWarray[0].start();
  result = "";
  
  pinMode(leftMtrSpdPin, OUTPUT);   
  pinMode(leftMtrDirPin, OUTPUT);
  pinMode(rightMtrDirPin, OUTPUT);
  pinMode(rightMtrSpdPin, OUTPUT);    
  pinMode(LEDStatPin, OUTPUT); 
  pinMode(LEDDataReceivedPin, OUTPUT); 
  pinMode(LEDMotortest, OUTPUT); //added
  Serial.println(F("Hello, CC3000!\n")); 

  Serial.print("Free RAM: "); Serial.println(getFreeRam(), DEC);
  
  /* Initialise the module */
  Serial.println(F("\nInitializing..."));
  if (!cc3000.begin())
  {
    Serial.println(F("Couldn't begin()! Check your wiring?"));
    while(1);
  }
  
  // Optional SSID scan
  // listSSIDResults();
  
  Serial.print(F("\nAttempting to connect to ")); Serial.println(WLAN_SSID);
  if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
    Serial.println(F("Failed!"));
    while(1);
  }
   
  Serial.println(F("Connected!"));
  
 // Check DHCP
  Serial.println(F("Requesting address from DHCP server..."));
  for(t=millis(); !cc3000.checkDHCP() && ((millis() - t) < dhcpTimeout); delay(1000));
  if(cc3000.checkDHCP()) {
    Serial.println(F("OK"));
  } else {
    Serial.println(F("failed"));
    return;
  }
  
  /* Display the IP address DNS, Gateway, etc. */  
  while (! displayConnectionDetails()) {
    delay(1000);
  }
}
void loop() {

 
 //Time of sending of date should match that of the polling frequency that is set for data display in WiFi Bot Control
 //If polling the data every 5 seconds, this information should be sent only every 5 seconds.
 
 //String timeSec = String((int) SWarray[0].elapsed()/1000);
 String timeSec = String((int) counter);
 String strDataToSend = "GET /arduinoPush.php?URLresult1=11&URLresult2=" + timeSec + " HTTP/1.1\r\nHost:" + WEBSITE + "\r\n";

  //START Sending data to WiFi Bot Control example
    Adafruit_CC3000_Client wwwSendData = cc3000.connectTCP(ip, port); 
    if (wwwSendData.connected()) {
      wwwSendData.println(strDataToSend);      
      wwwSendData.fastrprint(F("\r\n"));
        wwwSendData.println();
       Serial.println(F("COnnection successful,data sending"));
     } 
     else 
     {
      Serial.println(F("Data Send - Connection failed"));    
      return;
     }
    wwwSendData.close();
    
    while (wwwSendData.available()) {
      wwwSendData.read();
    }
    //end send data out~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //END Sending data to WiFi Bot Control example
 

 //Get data from WiFi Bot Control joystick movement.
  Adafruit_CC3000_Client www = cc3000.connectTCP(ip, port);  
  if (www.connected()) {
    www.fastrprint(F("GET "));
    www.fastrprint(WEBPAGE);
    www.fastrprint(F(" HTTP/1.1\r\n"));
    www.fastrprint(F("Host: ")); www.fastrprint(WEBSITE); www.fastrprint(F("\r\n"));
    www.fastrprint(F("\r\n"));
    www.println();
    Serial.println(F("Data Receive - Connection successful"));
   
  } else {
    Serial.println(F("Data Receive - Connection failed"));    
    return;
  }
  


  /* Read data until either the connection is closed, or the idle timeout is reached. */
  unsigned long lastRead = millis();
  while (www.connected() && (millis() - lastRead < IDLE_TIMEOUT_MS)) {
  //while (www.connected()) {
    while (www.available()) {
      char c = www.read();
      result = result + c;
      analogWrite(LEDDataReceivedPin, 150);  //flash LED to show data being received.
      
      // Delete HTTP headers
      //if(result.endsWith("Content-Type: text/html"))
      result.toLowerCase();
      if(result.endsWith("content-type: text/html"))
      {
        //if true, it will essentially clear out all the header content that came before
        //the actual motor values x,x,x,x,  later this is also trimmed as there are spaces around it
        result="";
      }
    }
    analogWrite(LEDDataReceivedPin, 0);
  }  
  www.close();
  
  
 // Format result and extract the variables
 format_result(motorCommand,result);
 
 //blink LED
 digitalWrite(LEDStatPin, HIGH);
 delay(50);
 digitalWrite(LEDStatPin, LOW);
 delay(50);
  
 // Print received values
 Serial.println("Result: " + String(result));
 Serial.println("Motor 1 speed: " + String(motorCommand[0]) + " and direction: " + String(motorCommand[2]));
 Serial.println("Motor 2 speed: " + String(motorCommand[1]) + " and direction: " + String(motorCommand[3]));
 Serial.println("Mode: " + String(motorCommand[0]));
 Serial.println("Motor 1 speed/dir: " + String(motorCommand[1]));
 Serial.println("Motor 2 speed/dir: " + String(motorCommand[2]));
 Serial.println("UDLRVal: " + String(motorCommand[3]));
 Serial.println("CommandVal: " + String(motorCommand[4]));
 // Send motor commands
send_motor_command(speed_motor1,direction_motor1,motorCommand[0],motorCommand[2]);
send_motor_command(speed_motor2,direction_motor2,motorCommand[1],motorCommand[3]);
send_motor_command(leftMtrSpdPin,leftMtrDirPin,motorCommand[0]);
send_motor_command(rightMtrSpdPin,rightMtrDirPin,motorCommand[1]);
 //flip through the mode values and react accordingly
 switch (motorCommand[0]) {
    case 1:
      //driving via joystick x,y values
      if (motorCommand[1] != 0 && motorCommand[2] != 0)
        {
          driveMotors(motorCommand[1],motorCommand[2]);
          
        } 
        else
        {
         analogWrite(leftMtrSpdPin, 0);   //drive the motor
         analogWrite(rightMtrSpdPin, 0);  //drive the motor
        }
      break;
    case 2:
      //driving by simple joystick U D L R
      
      if (motorCommand[3] != 0)
        {
          //move motors according to command
          
        } 
      
      
      break;
    case 3:
      //sending command values
      //write your own code here to react to command values 1 - 8
      break;  
    default:
         analogWrite(leftMtrSpdPin, 0);   //Stop the motor
         analogWrite(rightMtrSpdPin, 0);  //Stop the motor
     break;
  }
 // Reset result variable
 result = "";  
counter ++; 
}
//##########################################################################################################################

//parse the content and assign the mode, motor and command values
void format_result(int* array, String result) {
 
 result.trim();
 
 //remove the last comma
 //if (result.endsWith(","))
 //{
 //result = result.substring(0, result.length()-1);
 //}
 // result = result.substring(0, result.lastIndexOf(','));
 resultLength = result.length();
 //Serial.println(result);
 //Serial.print(" result length = ");
 //Serial.println(resultLength);
 
 int commaPosition;
 int i = 0;
 do
  {
      commaPosition = result.indexOf(',');
      
      if(commaPosition != -1)
      {
          //Serial.println( result.substring(0,commaPosition));
          array[i] = result.substring(0,commaPosition).toInt();

          //Serial.print("CommaPos: " );
          //Serial.print (commaPosition);
          //Serial.print("  MotorVals: " );
          //Serial.print(array[i]);
          i = i+1;
          result = result.substring(commaPosition+1, result.length());
          //Serial.print("  NextResult: " );
          //Serial.println(result);         
      }
      else
      {
         if(result.length() > 0) {
           Serial.println(result);
          }
      }     
   }
   while(commaPosition >=0);  
} 




void driveMotors(int xVal, int yVal)
{ 
  float xPct=1.0;
  int xAdj, yAdj;
  xAdj = map(abs(xVal), 0, 100, 100, 255);
  yAdj = map(abs(yVal), 0, 100, 100, 255);

  //approach - 
  //if Y is positive, both motors are set to move forward, else reverse. Left and Right from center will determine the speed of each motor.  At extremes will reverse each motor for fast turns
  //the value for X will determine the relative speed of each motor.
  
  //first determine the direction
  if (yVal >= 0)
  {
    //both motors moving fwd
    digitalWrite(leftMtrDirPin, HIGH);  
    digitalWrite(rightMtrDirPin, HIGH);
  }
   else
  {
   //both reversed
   digitalWrite(leftMtrDirPin, LOW); 
   digitalWrite(rightMtrDirPin, LOW); 
  }
  
  
  //now determine left / right
 
  if (xVal <= 0)
  {
    if (xVal < -70 && (yVal <= 40 || yVal >= -40))  //fast turn
    {
      digitalWrite(leftMtrDirPin, LOW);   //reverse the motor = faster turn
      leftSpeed = 150;  //something fast, but not too crazy
      rightSpeed = 150;
    }
    else
    {
      leftSpeed = (float)yAdj * ((float)(100 - abs(xVal)) / 100);
      rightSpeed = yAdj;
    }    
  }    
  else
  {
    if (xVal > 70 && (yVal <= 40 || yVal >= -40))  //fast turn
    {
      digitalWrite(rightMtrDirPin, LOW); //reverse the motor = faster turn
      leftSpeed = 150;
      rightSpeed = 200; //something fast, but not too crazy
    }
    else
    {
      leftSpeed = yAdj;
      rightSpeed = (float)yAdj * ((float)(100 - xVal) / 100);
    }
  }
  
   //Serial.print("yAdj: ");
   //Serial.print(yAdj);
   //Serial.print("  xVal: ");
   //Serial.print(xVal);
   //Serial.print("  leftspeed: ");
   //Serial.print(leftSpeed);
   //Serial.print("    rightspeed: ");
   //Serial.println(rightSpeed);
   //Serial.print("     100-lesvxal:");
   //Serial.print(100 - abs(xVal));
   //Serial.print("  100-lesvxal/100: ");
   //Serial.println((100 - abs(xVal))/100, DEC);
  
   //drive the motors  
   analogWrite(leftMtrSpdPin, (int)leftSpeed);
   analogWrite(rightMtrSpdPin,(int)rightSpeed);    
  
  //set the motors to off
  //analogWrite(leftMtrSpdPin, 0);    //drive the motor
  //analogWrite(rightMtrSpdPin, 0);   //drive the motor   
   //slight delay
  delay(50);
  leftSpeed=0, rightSpeed=0, xAdj=0, yAdj=0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
 
//use to map values to float.   
float mapf (float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


bool displayConnectionDetails(void)
{
  uint32_t ipAddress, netmask, gateway, dhcpserv, dnsserv;
  
  if(!cc3000.getIPAddress(&ipAddress, &netmask, &gateway, &dhcpserv, &dnsserv))
  {
    Serial.println(F("Unable to retrieve the IP Address!\r\n"));
    return false;
  }
  else
  {
    Serial.print(F("\nIP Addr: ")); cc3000.printIPdotsRev(ipAddress);
    Serial.print(F("\nNetmask: ")); cc3000.printIPdotsRev(netmask);
    Serial.print(F("\nGateway: ")); cc3000.printIPdotsRev(gateway);
    Serial.print(F("\nDHCPsrv: ")); cc3000.printIPdotsRev(dhcpserv);
    Serial.print(F("\nDNSserv: ")); cc3000.printIPdotsRev(dnsserv);
    Serial.println();
    return true;
  }
}
