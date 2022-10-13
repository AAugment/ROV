//---------------------------------------------------------------------------------------------------------------------
//  Current Build Using:
//        Arduino Nano
//        Processor: ATMega328P
//---------------------------------------------------------------------------------------------------------------------

#include <nRF24L01.h>                                       //Nrf24L01 library
#include <RF24.h>                                           //RF library to manage radio communication
#include <SPI.h>                                            //SPI library

const byte RF_CE_PIN = 9;                                   //Variable to set RF CE pin for SPI interface
const byte RF_CSN_PIN = 10;                                 //Variable to set RF CSN pin for SPI interface
RF24 radio(RF_CE_PIN, RF_CSN_PIN);                          //Declare member of RF24 class and set CE and CSN pin numbers
const byte RFAddress[5] = {'!','R','O','V','!'};            //Create RF address (needed for RF connection)

const byte startByte = 1;                                   //Byte value to signify start of serial data stream
byte radioStatus = 2;                                       //Radio status variable
byte M1P, M2P, M3P, M4P, M5P, M6P;                          //Byte variables to handle motor values
const byte stp = 127;                                       //Stop value

struct RFBuf{                                               //Struct data type to handle parsing of ROV control data
     byte RFlag;                                            //Radio Flag used so ROV knows radio is connected
     byte MP1;                                              //Value of motor1Power
     byte MP2;                                              //Value of motor2Power
     byte MP3;                                              //Value of motor3Power
     byte MP4;                                              //Value of motor4Power
     byte MP5;                                              //Value of motor5Power
     byte MP6;                                              //Value of motor6Power
    };
  typedef struct RFBuf RFBuff;                              //Finalize definition of Struct
  RFBuff RFBuffer;                                          //Declare member of Struct

void setup() {
    Serial.begin(9600);
    radio.begin();                                         //Initialize radio
    radio.setDataRate(RF24_2MBPS);                         //Set the RF data rate (maybe change to 250KBPS)
    radio.setAutoAck(false);                               //Set auto acknowledge to false
    radio.setPALevel(RF24_PA_MAX);                         //Set RF power level to max for max range
    radio.openReadingPipe(1, RFAddress);                   //Set as receiver, set channel, and set RF address                
    radio.startListening();                                //Start RF reception
}

void loop(){
  passData();                                              //Run  passData function
  //debug();
}

void passData() {                                          //passData function
    if ( radio.available() ) {                             //Check radio buffer
        radio.read( &RFBuffer, sizeof(RFBuffer) );         //If data in buffer read it

        radioStatus = RFBuffer.RFlag;                      //Read radio buffer into struct
        M1P = RFBuffer.MP1;
        M2P = RFBuffer.MP2;
        M3P = RFBuffer.MP3;
        M4P = RFBuffer.MP4;
        M5P = RFBuffer.MP5;
        M6P = RFBuffer.MP6;


        Serial.write(startByte);                           //Write data to serial buffer
        Serial.write(radioStatus);                         //One byte at a time
        Serial.write(M1P);                                 //In order
        Serial.write(M2P);  
        Serial.write(M3P);
        Serial.write(M4P);
        Serial.write(M5P);
        Serial.write(M6P);
    }
    else{
      radioStatus = 2;                                     //If radio disconnected
      M1P = stp;                                           //Set thruster values to stop
      M2P = stp;
      M3P = stp;
      M4P = stp;
      M5P = stp;
      M6P = stp;
    }
  }

void debug(){                                              //Debug data to serial monitor
  Serial.print(radioStatus);
  Serial.print(", ");
  Serial.print(M1P);
  Serial.print(", ");
  Serial.print(M2P); 
  Serial.print(", ");
  Serial.print(M3P);
  Serial.print(", ");
  Serial.print(M4P);
  Serial.print(", ");
  Serial.print(M5P); 
  Serial.print(", ");
  Serial.println(M6P); 
}
