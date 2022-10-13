//---------------------------------------------------------------------------------------------------------------------
//  code inside these brackets                               //These are all just comments
//    "//----------"
//  are the main points for
//  fine tuning adjustments
//---------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------
//  Current Build Using:
//        Arduino Uno                                     Operation Modes
//
//                                            /\            Triangle:
//                  Camera                   /||\               T1 and T3 on left joystick Y axis
//                  _______                   ||                T2 and T4 on right joystick Y axis
//              T1 |       | T2            "Forwards"           T5 and T6 (Z axis) controlled with trim and triggers 
//                 |       |                                Circle:
//              T5 |  ROV  | T6            "Reverse"            T1 and T4 on left joystick Y axis
//                 |       |                  ||                T2 and T3 on right joystick Y axis
//              T3 |_______| T4              \||/               T5 and T6 (Z axis) controlled with triggers
//                                            \/            Square:
//                                                              T1, T2, T3, and T4 control using left joystick X and Y
//          D-Pad controls Z axis trim                          Right joystick X axis inlfluences turning
//              25 steps up and down                            T5 and T6 (Z axis) controlled with trim and triggers 
//              Up to trim up by one step                   Share:
//              Down to trim down by one step                   T1, T2, T3, and T4 turned off
//              Left to zero out trim                           T5 and T6 (Z axis) controlled independently with triggers
//                                                              R1 and L1 reverse thruster directions 
//          Options button controls governor                Cross:
//              Variable governor set to 100% power default     All thrusters stop
//              Each press of options steps power down 20%
//              Resets from 20% power to 100%
//---------------------------------------------------------------------------------------------------------------------

#include <PS4BT.h>                                           //PS4  Bluetooth library
#include <nRF24L01.h>                                        //Nrf24L01 library
#include <RF24.h>                                            //RF library to manage radio communication
#include <usbhub.h>                                          //USB hub library
#include <SPI.h>                                             //SPI library 

const byte RF_CE_PIN = 5;                                    //Variable to set RF CE pin for SPI interface
const byte RF_CSN_PIN = 6;                                   //Variable to set RF CSN pin for SPI interface
const byte RFAddress[5] = {'!', 'R', 'O', 'V', '!'};         //Create RF address (needed for RF connection)
RF24 radio(RF_CE_PIN, RF_CSN_PIN);                           //Declare member of RF24 class and set CE and CSN pin numbers

USB Usb;                                                     //Declare member USB class
//USBHub Hub1(&Usb);                                         //Some dongles have a hub inside 
BTD Btd(&Usb);                                               //Declare member of BTD class
PS4BT PS4(&Btd);                                             //Declare member of PS4BT class and initialize BT connection (use this if controller has already paired)
//PS4BT PS4(&Btd, PAIR);                                     //Declare member of PS4BT class and initialize BT pairing (use this to pair new controller)

int lastPrint = 0;                                           //Print timer variable
const int printDelay = 500;                                  //Print delay

byte RadioStatus = 3;                                        //RadioStatus flag (0=disconnected, 1=connected)
enum state_enum {STOP, DRIVE, DIVE, INDEPENDENT, FREE};      //Create strings for finite state labels
uint8_t state = STOP;                                        //Initialize state variable to STOP state

const byte stp = 127;                                        //Stop analog value
const byte minAnalog = 0;                                    //Minimun Analog value
const byte maxAnalog = 255;                                  //Maximun Analog value
//---------------------------------------------------------------------------------------------------------------------
const byte deadZoneLow = 117;                                //Dead zone thresholds to compensate for controller joystick centering
const byte deadZoneHigh = 137;                               
//---------------------------------------------------------------------------------------------------------------------
byte outAnalogMin = 2;                                       //Variables to handle adjustable governor output
byte outAnalogMax = 253;

byte leftJoyX = stp;                                         //Declare variables to hold analog sensor data from the controller
byte leftJoyY = stp;                                         //Initialize to stop value
byte rightJoyX = stp;
byte rightJoyY = stp;
byte leftTrigger = stp;
byte rightTrigger = stp;
byte zTriggerRight = stp;
byte zTriggerLeft = stp;

byte leftJoyXReverse = stp;                                  //Declare variables to hold analog sensor data inversions
byte leftJoyYReverse = stp;                                  //Initialize to stop value
byte rightJoyXReverse = stp;
byte rightJoyYReverse = stp;
byte leftTriggerReverse = stp;
byte rightTriggerReverse = stp;
byte zTriggerRightReverse = stp;
byte zTriggerLeftReverse = stp;

byte motor1Power = stp;                                      //Variables to set PWM values for thrusters
byte motor2Power = stp;                                      //Odd value to adjust for one thruster not centering (maybe change)
byte motor3Power = stp;
byte motor4Power = stp;
byte motor5Power = stp;  
byte motor6Power = stp;  

byte Free1Power = stp;                                       //Variables to handle Free driving mode calculations
byte Free2Power = stp;
byte Free3Power = stp;
byte Free4Power = stp;     
                              
//---------------------------------------------------------------------------------------------------------------------
byte governorMin = 0;                                        //Variables to set variable governor values 
byte governorMax = 5;
const byte governorModifier = 25;                            //Governor step amount (127 /5 steps = 25)
//---------------------------------------------------------------------------------------------------------------------
byte governorStatus = governorMin;                           //Governor status variable

byte zAxis = stp;                                            //Trigger controlled Z axis variable
byte ascend = stp;                                           //ascend/descend map triggers to Z axis
byte descend = stp;
byte zTrimMin = 0;                                           //Trim min/max values
byte zTrimMax = 50;
byte zTrimMid = zTrimMax/2;                                  //Trim zero point
byte zTrim = zTrimMid;                                       //Z trim counter
byte zTrimmed = 127;                                         //Z trim counter mapped to analog value
byte zAxisReverse = stp;                                     //zAxis inversions
byte zTrimmedReverse = stp;

struct RFBuf {                                               //Struct data type to handle parsing of ROV control data
  byte RFlag;                                                //Radio Flag used so ROV knows radio is connected
  byte MP1;                                                  //Value of motor1Power
  byte MP2;                                                  //Value of motor2Power
  byte MP3;                                                  //Value of motor3Power
  byte MP4;                                                  //Value of motor4Power
  byte MP5;                                                  //Value of motor5Power
  byte MP6;                                                  //Value of motor6Power
};
typedef struct RFBuf RFBuff;                                 //Finalize definition of struct
RFBuff RFBuffer;                                             //Declare member of struct

void setup() {                                               //Setup is run once at power up or reset of arduino
  Serial.begin(9600);                                        //Start serial connection and set baud rate
  
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);                                       //Set pins for status LED
  pinMode(A2, OUTPUT);
  digitalWrite(A0, HIGH);
  digitalWrite(A1, LOW);                                     //Set status LED red
  digitalWrite(A2, LOW);
  delay(100);                                                //Wait 100 milliseconds
  digitalWrite(A0, LOW);
  digitalWrite(A1, HIGH);                                    //Set status LED green
  digitalWrite(A2, LOW);
  delay(100);
  digitalWrite(A0, LOW);
  digitalWrite(A1, LOW);                                     //Set status LED blue
  digitalWrite(A2, HIGH);

  radio.begin();                                             //Initialize radio
  radio.setDataRate(RF24_2MBPS);                             //Set the RF data rate (maybe change to 250KBPS)
  radio.setAutoAck(false);                                   //Set auto acknowledge to false
  radio.setPALevel(RF24_PA_MAX);                             //Set RF power level to max for max range
  radio.openWritingPipe(RFAddress);                          //Set as transmitter and set RF address

#if !defined(__MIPSEL__)                                     //Needed for USB connection
  while (!Serial);                                           //Pause while serial connection initializes
#endif
  if (Usb.Init() == -1) {                                    //Check for USB initialization error
    Serial.print("USB Connection Error");                    //Print error message to serial monitor
    while (1) {                                              //Stop everything (Maybe change this)
      digitalWrite(A0, HIGH);
      digitalWrite(A1, HIGH);                                //Set all LEDs high
      digitalWrite(A2, HIGH);
      delay(50);                                             //Flash all LEDs on and off (Flashing indicates error in USB connection)
      digitalWrite(A0, LOW);
      digitalWrite(A1, LOW);                                 //Set all LEDs low
      digitalWrite(A2, LOW);
      delay(50);
    }
  }
  Serial.println("USB Host Connected");                      //USB connection successful print to serial monitor
}

void loop() {                                                //Loop() is run repeatedly while arduino is powered
  Usb.Task();                                                //Run Usb.Task() method from USB library (Needed to establish USB connection)

  CheckDriveState();                                         //Run CheckDriveState() function (see  below)
  RadioOut();                                                //Run RadioOut() function (see  below)

  //JoyDebug();                                              //Uncomment to see controller joystick data in serial monitor
  //ButtonDebug();                                           //Uncomment to see controller button data in serial monitor
  //RadioDebug();                                            //Uncomment to see contents of radio buffer in serial monitor
}

void CheckDriveState() {
  if (PS4.connected()) {                                     //Check to make share sure PS4 controller is connected (If connected then this block will execute)
    leftJoyX = PS4.getAnalogHat(LeftHatX);                   //Set variables to hold analog sensor data from the joysticks
    leftJoyY = PS4.getAnalogHat(LeftHatY);
    rightJoyX = PS4.getAnalogHat(RightHatX);
    rightJoyY = PS4.getAnalogHat(RightHatY);
    leftTrigger = PS4.getAnalogButton(L2);                   //Set analog trigger variables
    rightTrigger = PS4.getAnalogButton(R2); 

    //---------------------------------------------------------------------------------------------------------------------
    if (PS4.getButtonClick(UP)&&(zTrim < zTrimMax)) {        //Set Z trim counter if D-pad Up or Down pressed
      zTrim += 1;
    }
    if (PS4.getButtonClick(DOWN)&&(zTrim > zTrimMin)) {
      zTrim -= 1;
    }

    zTrim = constrain(zTrim, zTrimMin, zTrimMax);            //constrain Z trim counter to usable range
    zTrimmed = map(zTrim, zTrimMin, zTrimMax, minAnalog, maxAnalog);//Map Z trim counter to analog values
    if (PS4.getButtonClick(LEFT)) {                          //Zero out trim
      zTrim = zTrimMid;
      zTrimmed =  127;
    }
    //---------------------------------------------------------------------------------------------------------------------

    leftJoyXReverse = map(leftJoyX, minAnalog, maxAnalog, maxAnalog, minAnalog); //Set variables to hold reverse analog sensor data from the controller
    leftJoyYReverse = map(leftJoyY, minAnalog, maxAnalog, maxAnalog, minAnalog);
    rightJoyXReverse = map(rightJoyX, minAnalog, maxAnalog, maxAnalog, minAnalog);
    rightJoyYReverse = map(rightJoyY, minAnalog, maxAnalog, maxAnalog, minAnalog);
    leftTriggerReverse = map(leftTrigger, minAnalog, maxAnalog, maxAnalog, minAnalog);
    rightTriggerReverse = map(rightTrigger, minAnalog, maxAnalog, maxAnalog, minAnalog);
    
    zTriggerLeft = map(leftTrigger, minAnalog, maxAnalog, 127, 255); //Variable to map trigger values to zAxis range
    zTriggerRight = map(rightTrigger, minAnalog, maxAnalog, 127, 255);
    zTriggerLeftReverse = map(leftTrigger, minAnalog, maxAnalog, 127, 0); //Inversions
    zTriggerRightReverse = map(rightTrigger, minAnalog, maxAnalog, 127, 0);
    
    //---------------------------------------------------------------------------------------------------------------------
    ascend = rightTrigger;                                   //Mostly just naming controller use
    descend = leftTriggerReverse;                            //Could be redesigned to better incorporate Z trim
    zAxis = (ascend + descend) / 2;                          //Takes input from both triggers adjusts into single value
    //---------------------------------------------------------------------------------------------------------------------
    
    zAxisReverse = map(zAxis, minAnalog, maxAnalog, maxAnalog, minAnalog); //Inversions
    zTrimmedReverse = map(zTrimmed, minAnalog, maxAnalog, maxAnalog, minAnalog);

    //---------------------------------------------------------------------------------------------------------------------
    
    if (PS4.getButtonClick(OPTIONS)){                        //Press Options button to toggle speed governor
      governorStatus += 1;                                   //Change governor status to next position
      if(governorStatus >= governorMax){                     //Roll governor status back to off position
        governorStatus = governorMin;
      }
    } 

    if(governorStatus > 0){
      outAnalogMax = (maxAnalog - (governorModifier)*(governorStatus)); //Set governor regulated analog values
      outAnalogMin = (minAnalog + (governorModifier)*(governorStatus));
    }
    else{
      outAnalogMax = 253;                                    //Turn governor off
      outAnalogMin = 2;
    }
       
    //---------------------------------------------------------------------------------------------------------------------

    switch (state) {                                         //Switch case finite state machine

      case STOP:                                             //Stop state [X button]
        digitalWrite(A0, LOW);
        digitalWrite(A1, LOW);                               //Turn off all LEDs (no lights indicates STOP state)
        digitalWrite(A2, LOW);
        
        //---------------------------------------------------------------------------------------------------------------------
        motor1Power = stp;                                   //Variables to set values for thrusters (initialized at stop)
        motor2Power = stp;                                   //Odd value to adjust for one thruster not centering (maybe change)
        motor3Power = stp;
        motor4Power = stp;
        motor5Power = stp;
        motor6Power = stp;
        //---------------------------------------------------------------------------------------------------------------------

        if (PS4.getButtonClick(TRIANGLE)) {                  //Switch States
          state = DRIVE;
        }
        else if (PS4.getButtonClick(CIRCLE)) {
          state = DIVE;
        }
        else if (PS4.getButtonClick(SHARE)) {
          state = INDEPENDENT;
        }
        else if (PS4.getButtonClick(SQUARE)) {
          state = FREE;
        }
        break;                                               //Break to run loop again

      case DRIVE:                                            //Lateral thruster matching (Tank Drive Mode) [Triangle button]
        digitalWrite(A0, LOW);
        digitalWrite(A1, HIGH);                              //Turn on just Green LED (Green indicates Drive state)
        digitalWrite(A2, LOW);
        
        //---------------------------------------------------------------------------------------------------------------------
        motor1Power = leftJoyY;                              //Set which controller inputs will control which thruster
        motor2Power = rightJoyYReverse;                      //These can be adjusted
        motor3Power = leftJoyY;
        motor4Power = rightJoyYReverse;
        
        if ((rightTrigger > 20) || (leftTriggerReverse < 235)) {
          motor5Power = zAxisReverse;                        //Use trigger values when either trigger depressed
          motor6Power = zAxis;                                
        }
        else {
          motor5Power = zTrimmedReverse;                     //Otherwise use Z trim value
          motor6Power = zTrimmed;
        }
        //---------------------------------------------------------------------------------------------------------------------


        if (PS4.getButtonClick(CROSS)) {                     //Switch States
          state = STOP;
        }
        else if (PS4.getButtonClick(CIRCLE)) {
          state = DIVE;
        }
        else if (PS4.getButtonClick(SHARE)) {
          state = INDEPENDENT;
        }
        else if (PS4.getButtonClick(SQUARE)) {
          state = FREE;
        }
        break;                                               //Break to run loop again

      case DIVE:                                             //Diagonal thruster matching (Crab Walk Mode) [Circle button]
        digitalWrite(A0, LOW);
        digitalWrite(A1, LOW);                               //Turn on just Blue LED (Blue indicates Dive state)
        digitalWrite(A2, HIGH);
        
        //---------------------------------------------------------------------------------------------------------------------
        motor1Power = leftJoyY;                              //Set which controller inputs will control which thruster
        motor2Power = rightJoyYReverse;                      //These can be adjusted
        motor3Power = rightJoyY;
        motor4Power = leftJoyYRever;se;

        motor5Power = zAxisReverse;                          //zAxis trigger control
        motor6Power = zAxis;
        //---------------------------------------------------------------------------------------------------------------------

        if (PS4.getButtonClick(CROSS)) {                     //Switch States
          state = STOP;
        }
        else if (PS4.getButtonClick(TRIANGLE)) {
          state = DRIVE;
        }
        else if (PS4.getButtonClick(SHARE)) {
          state = INDEPENDENT;
        }
        else if (PS4.getButtonClick(SQUARE)) {
          state = FREE;
        }
        break;                                               //Break to run loop again

      case INDEPENDENT:                                      //Z-axis thrusters controlled seperately (Barrel Roll Mode) [Share button]
        digitalWrite(A0, HIGH);
        digitalWrite(A1, LOW);                               //Turn on just Red LED (Red indicates INDEPENDENT state)
        digitalWrite(A2, LOW);
        
        //---------------------------------------------------------------------------------------------------------------------
        motor1Power = stp;                                   //Set which controller inputs will control which thruster
        motor2Power = stp;                                   //These will need to be  adjusted
        motor3Power = stp;
        motor4Power = stp;
        
        if(PS4.getButtonPress(R1)) {                         //Check R1 and L1 buttons and map z triggers accordingly
          motor5Power = zTriggerRightReverse;
        }
        else {
          motor5Power = zTriggerRight;
        }
        if(PS4.getButtonPress(L1)) {
          motor6Power = zTriggerLeft;
        }
        else {
          motor6Power = zTriggerLeftReverse;
        }
        //---------------------------------------------------------------------------------------------------------------------

        if (PS4.getButtonClick(CROSS)) {                     //Switch States
          state = STOP;
        }
        else if (PS4.getButtonClick(TRIANGLE)) {
          state = DRIVE;
        }
        else if (PS4.getButtonClick(CIRCLE)) {
          state = DIVE;
        }
        else if (PS4.getButtonClick(SQUARE)) {
          state = FREE;
        }
        break;                                               //Break to run loop again

      case FREE:                                             //Left joystick dominated (Free Movement Mode) [Square button]
       digitalWrite(A0, HIGH);
       digitalWrite(A1, HIGH);                               //Turn on just Red LED (Red indicates INDEPENDENT state)
       digitalWrite(A2, HIGH);
       
       //--------------------------------------------------------------------------------------------------------------------- 
        if(leftJoyYReverse > stp) {                          //Left joystick forward
            if(leftJoyX > deadZoneHigh) {                    //Check leftJoyX deadzone
              Free1Power = leftJoyY;                         //Set which controller inputs will control which thruster
              Free2Power = (127 + (leftJoyYReverse - leftJoyX));                      
              Free3Power = (127 - (leftJoyYReverse - leftJoyX));
              Free4Power = leftJoyYReverse;
             }
            else if(leftJoyX < deadZoneLow) {                //Check leftJoyX deadzone
              Free1Power = (127 - (leftJoyYReverse - leftJoyXReverse)); 
              Free2Power = leftJoyYReverse;                  //Set which controller inputs will control which thruster     
              Free3Power = leftJoyY;
              Free4Power = (127 + (leftJoyYReverse - leftJoyXReverse));
             }
            else {
              Free1Power = leftJoyY;                         //Set which controller inputs will control which thruster
              Free2Power = leftJoyYReverse;                      
              Free3Power = leftJoyY;
              Free4Power = leftJoyYReverse;
            }
        }
        
        else if(leftJoyYReverse < stp) {                     //Left joystick backward
           if(leftJoyX > deadZoneHigh){                      //Check leftJoyX deadzone
              Free1Power = (127 + (leftJoyY - leftJoyX));    //Set which controller inputs will control which thruster                              
              Free2Power = leftJoyYReverse;                      
              Free3Power = leftJoyY;
              Free4Power = (127 - (leftJoyY - leftJoyX));
             }
            else if(leftJoyX < deadZoneLow) {                //Check leftJoyX deadzone
              Free1Power = leftJoyY;                         //Set which controller inputs will control which thruster
              Free2Power = (127 - (leftJoyY - leftJoyXReverse));                      
              Free3Power = (127 + (leftJoyY - leftJoyXReverse));
              Free4Power = leftJoyYReverse;
             }
            else{  
              Free1Power = leftJoyY;                         //Set which controller inputs will control which thruster
              Free2Power = leftJoyYReverse;                      
              Free3Power = leftJoyY;
              Free4Power = leftJoyYReverse;
            }
          }            

        else{                                                //If leftJoyY is centered
          if((rightJoyX > deadZoneHigh)||(rightJoyX < deadZoneLow)) {
                motor1Power = rightJoyXReverse;              //Set which controller inputs will control which thruster
                motor2Power = rightJoyXReverse;                      
                motor3Power = rightJoyXReverse;
                motor4Power = rightJoyXReverse;
          }
          else{                                              //If both joysticks centered
            motor1Power = stp;
            motor2Power = stp;
            motor3Power = stp;
            motor4Power = stp;
          }
        }
        
      if(rightJoyX < deadZoneLow){                           //Check rightJoyX dead zone
          motor1Power = 127 - (Free1Power - rightJoyXReverse); //Set which controller inputs will control which thruster
          motor2Power = 127 + (Free2Power - rightJoyXReverse); //rightJoyX adjusts leftJoy values                     
          motor3Power = 127 - (Free3Power - rightJoyXReverse);
          motor4Power = 127 + (Free4Power - rightJoyXReverse);
        }
      else if(rightJoyX > deadZoneHigh){                     //Check rightJoyX dead zone
          motor1Power = 127 + (Free1Power - rightJoyXReverse); //Set which controller inputs will control which thruster
          motor2Power = 127 - (Free2Power - rightJoyXReverse); //rightJoyX adjusts leftJoy values                    
          motor3Power = 127 + (Free3Power - rightJoyXReverse);
          motor4Power = 127 - (Free4Power - rightJoyXReverse);
        }
      else{
          motor1Power = Free1Power;                          //Set which controller inputs will control which thruster
          motor2Power = Free2Power;                      
          motor3Power = Free3Power;
          motor4Power = Free4Power;
        }

      if ((rightTrigger > 20) || (leftTriggerReverse < 235)) {
          motor5Power = zAxisReverse;                        //Use trigger values when either trigger depressed
          motor6Power = zAxis;                                
        }
        else {
          motor5Power = zTrimmedReverse;                     //Otherwise use Z trim value
          motor6Power = zTrimmed;
        }
       //---------------------------------------------------------------------------------------------------------------------
        
        if (PS4.getButtonClick(CROSS)) {                     //Switch States
          state = STOP;
        }
        else if (PS4.getButtonClick(TRIANGLE)) {
          state = DRIVE;
        }
        else if (PS4.getButtonClick(CIRCLE)) {
          state = DIVE;
        }
        else if (PS4.getButtonClick(SHARE)) {
          state = INDEPENDENT;
        }
        break;     
    }
    
    if (((leftJoyY > deadZoneLow)&&(leftJoyY < deadZoneHigh))&&((leftJoyX > deadZoneLow)&&(leftJoyX < deadZoneHigh))&&((rightJoyX < deadZoneHigh)&&(rightJoyX > deadZoneLow))) {
          motor1Power = stp;
          motor2Power = stp;                                 //Check joystick dead zones and stop motors if inside thresholds
          motor3Power = stp;
          motor4Power = stp;
      }
  }

  else {                                                     //If PS4 controller NOT connected then this block will execute
    //---------------------------------------------------------------------------------------------------------------------
    motor1Power = stp;                                       //Set motor signals to stop position
    motor2Power = stp;                                       //Odd value to adjust for one thruster not centering (maybe change)
    motor3Power = stp;                                       //Maybe change this block to emergency surfacing
    motor4Power = stp;
    motor5Power = stp;  //motor5Power = 255; ??(should ROV surface on its own in the event of controller disconnect?)?
    motor6Power = stp;
    //---------------------------------------------------------------------------------------------------------------------
  }
}

void RadioOut() {                                            //Radio send function 
  motor1Power = map(motor1Power, minAnalog, maxAnalog, outAnalogMin, outAnalogMax);
  motor2Power = map(motor2Power, minAnalog, maxAnalog, outAnalogMin, outAnalogMax); //Variable governor analog value adjustments
  motor3Power = map(motor3Power, minAnalog, maxAnalog, outAnalogMin, outAnalogMax);
  motor4Power = map(motor4Power, minAnalog, maxAnalog, outAnalogMin, outAnalogMax);
  motor5Power = map(motor5Power, minAnalog, maxAnalog, outAnalogMin, outAnalogMax);
  motor6Power = map(motor6Power, minAnalog, maxAnalog, outAnalogMin, outAnalogMax);

  motor1Power = constrain(motor1Power, outAnalogMin, outAnalogMax);
  motor2Power = constrain(motor2Power, outAnalogMin, outAnalogMax); //Constrain analog values to governor range
  motor3Power = constrain(motor3Power, outAnalogMin, outAnalogMax);
  motor4Power = constrain(motor4Power, outAnalogMin, outAnalogMax);
  motor5Power = constrain(motor5Power, outAnalogMin, outAnalogMax);
  motor6Power = constrain(motor6Power, outAnalogMin, outAnalogMax);
  
  RFBuffer.RFlag = RadioStatus;                              //Set radio status flag                          
  RFBuffer.MP1 = motor1Power;                                //Set motor1Power struct value
  RFBuffer.MP2 = motor2Power;                                //Set motor2Power struct value
  RFBuffer.MP3 = motor3Power;                                //Set motor3Power struct value
  RFBuffer.MP4 = motor4Power;                                //Set motor4Power struct value
  RFBuffer.MP5 = motor5Power;                                //Set motor5Power struct value
  RFBuffer.MP6 = motor6Power;
  radio.writeFast(&RFBuffer, sizeof(RFBuffer));              //Write contents of struct to RF buffer
}

void JoyDebug() {                                            //Debug for controller joystick data
  unsigned long now = millis();                              //Set now timer
  if ((now - lastPrint) >= (printDelay)) {                   //Check printDelay
    if (PS4.connected()) {
      Serial.print(" LeftJoy:");                             //Write to serial monitor with some labels formatting for legibility
      Serial.print(PS4.getAnalogHat(LeftHatX));
      Serial.print(", ");
      Serial.print(PS4.getAnalogHat(LeftHatY));
      Serial.print(" RightJoy:");
      Serial.print(PS4.getAnalogHat(RightHatX));
      Serial.print(", ");
      Serial.print(PS4.getAnalogHat(RightHatY));
      Serial.print("   L2:");
      Serial.print(PS4.getAnalogButton(L2));
      Serial.print(" R2:");
      Serial.print(PS4.getAnalogButton(R2));
      Serial.print("   Pitch:");
      Serial.print(PS4.getAngle(Pitch));
      Serial.print(" Roll:");
      Serial.println(PS4.getAngle(Roll));

      lastPrint = millis();                                  //Set lastPrint variable
    }
  }
}
void  ButtonDebug() {                                        //Debug for controller Button data
  unsigned long now = millis();                              //Set now timer
  if ((now - lastPrint) >= (printDelay)) {                   //Check printDelay
    if (PS4.connected()) {
      Serial.print("   Buttons: PS:");
      Serial.print(PS4.getButtonPress(PS));
      Serial.print(" Share:");
      Serial.print(PS4.getButtonPress(SHARE));
      Serial.print(" Options:");
      Serial.print(PS4.getButtonPress(OPTIONS));
      Serial.print(" Touch:");
      Serial.print(PS4.getButtonPress(TOUCHPAD));
      Serial.print(" Up:");
      Serial.print(PS4.getButtonPress(UP));
      Serial.print(" Left:");
      Serial.print(PS4.getButtonPress(LEFT));
      Serial.print(" Down:");
      Serial.print(PS4.getButtonPress(DOWN));
      Serial.print(" Right:");
      Serial.print(PS4.getButtonPress(RIGHT));
      Serial.print(" Triangle:");
      Serial.print(PS4.getButtonPress(TRIANGLE));
      Serial.print(" Square:");
      Serial.print(PS4.getButtonPress(SQUARE));
      Serial.print(" Cross:");
      Serial.print(PS4.getButtonPress(CROSS));
      Serial.print(" Circle:");
      Serial.print(PS4.getButtonPress(CIRCLE));
      Serial.print(" L1:");
      Serial.print(PS4.getButtonPress(L1));
      Serial.print(" L2:");
      Serial.print(PS4.getButtonPress(L2));
      Serial.print(" L3:");
      Serial.print(PS4.getButtonPress(L3));
      Serial.print(" R1:");
      Serial.print(PS4.getButtonPress(R1));
      Serial.print(" R2:");
      Serial.print(PS4.getButtonPress(R2));
      Serial.print(" R3:");
      Serial.println(PS4.getButtonPress(R3));

      lastPrint = millis();
    }
  }
}

void RadioDebug() {                                          //Debug for RF struct data
  unsigned long now = millis();                              //Set now timer
  if ((now - lastPrint) >= (printDelay)) {                   //Check printDelay
    Serial.print(RFBuffer.RFlag);
    Serial.print(", ");
    Serial.print(governorStatus);
    Serial.print(", ");
    Serial.print(state);
    Serial.print(", ");
    Serial.print(RFBuffer.MP1);
    Serial.print(", ");
    Serial.print(RFBuffer.MP2);
    Serial.print(", ");
    Serial.print(RFBuffer.MP3);
    Serial.print(", ");
    Serial.print(RFBuffer.MP4);
    Serial.print(", ");
    Serial.print(RFBuffer.MP5);
    Serial.print(", ");
    Serial.print(RFBuffer.MP6);
    Serial.print(", ");
    Serial.print(zTrim);
    Serial.print(", ");
    Serial.print(zTrimmed);
    Serial.print(", ");
    Serial.println(zAxis);

    lastPrint = millis();
  }
}

/**************************************************************************************************
 ******************************************A list of the possible inputs from the PS4 controller***
 ***PS4 Library Controller Inputs******************************************************************
 *********************************************For reference and design of control interface********                     .
 **************************************************************************************************

  PS4.getAnalogHat(LeftHatX)
  PS4.getAnalogHat(LeftHatY)
  PS4.getAnalogHat(RightHatX)
  PS4.getAnalogHat(RightHatY)
  PS4.getAnalogButton(L2)
  PS4.getAnalogButton(R2)

  PS4.getButtonClick(PS)
  PS4.getButtonClick(SHARE)
  PS4.getButtonClick(OPTIONS)
  PS4.getButtonClick(TOUCHPAD)
  PS4.getButtonClick(UP)
  PS4.getButtonClick(LEFT)
  PS4.getButtonClick(DOWN)
  PS4.getButtonClick(RIGHT)
  PS4.getButtonClick(TRIANGLE)
  PS4.getButtonClick(SQUARE)
  PS4.getButtonClick(CROSS)
  PS4.getButtonClick(CIRCLE)
  PS4.getButtonClick(L1)
  PS4.getButtonClick(L2)
  PS4.getButtonClick(L3)
  PS4.getButtonClick(R1)
  PS4.getButtonClick(R2)
  PS4.getButtonClick(R3)

  PS4.getButtonPress(PS)
  PS4.getButtonPress(SHARE)
  PS4.getButtonPress(OPTIONS)
  PS4.getButtonPress(TOUCHPAD)
  PS4.getButtonPress(UP)
  PS4.getButtonPress(LEFT)
  PS4.getButtonPress(DOWN)
  PS4.getButtonPress(RIGHT)
  PS4.getButtonPress(TRIANGLE)
  PS4.getButtonPress(SQUARE)
  PS4.getButtonPress(CROSS)
  PS4.getButtonPress(CIRCLE)
  PS4.getButtonPress(L1)
  PS4.getButtonPress(L2)
  PS4.getButtonPress(L3)
  PS4.getButtonPress(R1)
  PS4.getButtonPress(R2)
  PS4.getButtonPress(R3)

  PS4.getAngle(Pitch)
  PS4.getAngle(Roll)

  PS4.setRumbleOn(PS4.getButtonClick(UP))
  PS4.setRumbleOn(RumbleLow)
  PS4.setRumbleOn(RumbleHigh)

*/
