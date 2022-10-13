//---------------------------------------------------------------------------------------------------------------------
//  Current Build Using:
//        Arduino Nano
//        Processor: ATMega328P (Old Bootloader)
//---------------------------------------------------------------------------------------------------------------------

#include <Servo.h>                                          //Servo library

Servo motor1, motor2, motor3, motor4, motor5, motor6;       //Declare members of Servo class

const int stp = 1500;                                       //Stop PWM value (thruster middle/stop)
int motor1Power = stp;                                      //Variables to set PWM values for thrusters (initialized at stop PWM)
int motor2Power = stp;                                      //Odd value to adjust for one thruster not centering (maybe change)
int motor3Power = stp;
int motor4Power = stp;
int motor5Power = stp;
int motor6Power = stp;
byte M1P, M2P, M3P, M4P, M5P, M6P;                          //Variables to handle radio translation
byte testByte;
const byte startByte = 1;
  
const int minAnalog = 2;                                    //Minimum analog value
const int maxAnalog = 253;                                  //Maximum analog value
const int minPWM = 1000;                                    //Minimum PWM value (thruster full "forwards")
const int maxPWM = 2000;                                    //Maximum PWM value (thruster full "reverse")

byte RadioStatus = 2;                                       //RadioStatus flag (0=disconnected, 1=connected)
unsigned long lastRecvTime = 0;                             //Variable to hold microseconds since last data receive (radio time out)
//---------------------------------------------------------------------------------------------------------------------
byte RadioTimeOut = 250;                                    //Threshold for radio time out (currently 1/4 second)
//---------------------------------------------------------------------------------------------------------------------

void setup() {                                              //Setup() is run once at power up or reset of arduino
  Serial.begin(9600);                                       //Start serial connection
  
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);                                      //Set pins for status LED
  pinMode(A2, OUTPUT);
  digitalWrite(A0, HIGH);
  digitalWrite(A1, LOW);                                    //Set status LED red
  digitalWrite(A2, LOW);
  delay(100);                                               //Wait 100 milliseconds
  digitalWrite(A0, LOW);
  digitalWrite(A1, HIGH);                                   //Set status LED green
  digitalWrite(A2, LOW);
  delay(100);
  digitalWrite(A0, LOW);
  digitalWrite(A1, LOW);                                    //Set status LED blue
  digitalWrite(A2, HIGH);
  
  motor1.attach(10);                                        //Set thruster pin connections
  motor2.attach(9);
  motor3.attach(6);
  motor4.attach(5);
  motor5.attach(3);
  motor6.attach(11);
}

void loop(){                                                //Loop() is run repeatedly while arduino is powered 
    CheckSerial();                                          //Run CheckRadioStatus() function (see below) 
   
    motor1.writeMicroseconds(motor1Power);                  //Send PWM signal to thrusters
    motor2.writeMicroseconds(motor2Power);
    motor3.writeMicroseconds(motor3Power);
    motor4.writeMicroseconds(motor4Power);
    motor5.writeMicroseconds(motor5Power);
    motor6.writeMicroseconds(motor6Power);

    //debug();
}

void CheckSerial(){                                         //CheckRadioStatus() function. Finite state machine switiches between drive modes
                                                            //Make sure radio is connected
  unsigned long now = millis();                             //Set now timer
    if (now - lastRecvTime > RadioTimeOut){                 //Check if radio has timed out
        RadioStatus = 2;                                    //Set status as disconnected 
      }   
      
    while (Serial.available() > 7){                         //While RF signal is available and more than 7 bytes available
        testByte = Serial.read();                           //Read one byte from serial buffer
        if(testByte == startByte){                          //Is it the start byte?
          digitalWrite(A2, HIGH);                           //If so we have ordered connection
          RadioStatus = Serial.read();                      //Read radio status byte from buffer
          M1P = Serial.read();
          M2P = Serial.read();                              //Read thruster byte value data from buffer
          M3P = Serial.read();                              //In order
          M4P = Serial.read();                              //One byte at a time
          M5P = Serial.read();
          M6P = Serial.read();
          lastRecvTime = millis();                          //Set last receive timer     
      }       
  }  

      if (RadioStatus == 2){                                //If radio disconnected (maybe change to emergency surfacing routine?)
        digitalWrite(A0, HIGH);
        digitalWrite(A1, LOW);

        motor1Power = stp;                                  //Variables to set PWM values for thrusters (initialized at stop PWM)
        motor2Power = stp;                                  //Odd value to adjust for one thruster not centering (maybe change)
        motor3Power = stp;
        motor4Power = stp;
        motor5Power = stp;  //motor5Power = 2000; ?(should ROV surface on its own in the event of radio disconnect?)?
        motor6Power = stp;     
        }
          
      else if (RadioStatus == 3){                           //Radio connected
        digitalWrite(A0, LOW);
        digitalWrite(A1, HIGH);
        
        motor1Power = map(int(M1P), minAnalog, maxAnalog, minPWM, maxPWM); //Variables to set PWM values for thrusters (initialized at stop PWM)
        motor2Power = map(int(M2P), minAnalog, maxAnalog, minPWM, maxPWM); //Odd value to adjust for one thruster not centering (maybe change)
        motor3Power = map(int(M3P), minAnalog, maxAnalog, minPWM, maxPWM); //"Byte" values are translated into "int" values for PWM range
        motor4Power = map(int(M4P), minAnalog, maxAnalog, minPWM, maxPWM);
        motor5Power = map(int(M5P), minAnalog, maxAnalog, minPWM, maxPWM);
        motor6Power = map(int(M6P), minAnalog, maxAnalog, minPWM, maxPWM);
      } 
    }

void debug(){
  Serial.print(RadioStatus);
  Serial.print(", ");
  Serial.print(motor1Power);
  Serial.print(", ");
  Serial.print(motor2Power);
  Serial.print(", ");
  Serial.print(motor3Power);
  Serial.print(", ");
  Serial.print(motor4Power);
  Serial.print(", ");
  Serial.print(motor5Power);
  Serial.print(", ");
  Serial.println(motor6Power);

  delay(20);
}
