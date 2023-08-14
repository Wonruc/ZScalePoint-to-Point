//                           Arduino Uno/Nano L298 Motor Shield Automatic Point-to-Point DC Train Control
//
//                                            Copyright 2023 Matthew Curnow
//                            
//                                                  Updated 14-Aug-2023
//
//  This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License
//  as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
//  of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License along with this program.
//  If not, see <https://www.gnu.org/licenses/>.
//
//  ================================================== Description =============================================================
//  
//  This program is intended to utilize an Arduino Nano, Uno or Mega micro controller, a L298P or L298N motor shield, and IR sensors
//  to automate the operation of a Point-to-Point DC powered model railroad.  The motor shield acts as a PWM DC power supply to
//  the tracks.  The analog input of a 10 ohm potentiometer is used to adjust the motor shields PWM/voltage sent to the tracks 
//  for speed control.  When a train enters a station, the IR sensor is triggered, the train slows to a stop, waits for a 
//  defined interval, the polarity of the track may be changed as needed to send the train to the next station, and the train 
//  is set to accelerate out of the station.  This code includes short protection. The constants used to identify over
//  current, start speed, acceleration and deceleration  were established using 3V Rokuhan Shorty Z scale trains for the 
//  Showcase Miniatures trolley and may need to be adjusted for larger trains at higher voltages. If a train fails to arrive at
//  a station after one minute, a stall is assumed and track power is turned off.  Faults can be cleared by triggering any
//  station IR sensor. 
//
//  =================================================== Notes and Layout Setup =================================================
//
//  There are several use cases to that this code accomidates.  Point to point layouts, Loop layouts with station stops, and
//  switchback. The code is defaulted to point to point, however by setting the layout type, these additional scenarios can be
//  used.
//
//    Loop A to A, 
//      ___A___        ___A___
//     /       \      /       \
//     \_______/  or  \___B___/
//
//
//    Point-to-Point or Linear: A to B or multistation A - C - D - B
//
//
//      Switchback: A to B reversing at C
//
//                  A______C
//                    B__/
//
//    The only core difference is which stations change direction or activate a turnout
//   _________________________________________________________________________
//  |Layout Type: |Loop (0)   | Point-to-Point (1) | Switchback (2)           |
//  |-------------------------------------------------------------------------|
//  |Station A    | Stop Only | Direction East     | Direction East           |
//  |             |           |                    | Turnout Check Main       |
//  |-------------------------------------------------------------------------|
//  |Station B    | Stop Only | Direction West     | Direction East           |
//  |             |           |                    | Turnout Check Divergent  |
//  |-------------------------------------------------------------------------|
//  |Station C    | Stop Only | Stop Only          | Direction West           |
//  |             |           |                    | Turnout Switch Direction |
//  |-------------------------------------------------------------------------|
//  |Station D    | Stop Only | Stop Only          | Stop Only                |
//  |_____________|___________|____________________|__________________________|
// 
//*************************************************************************
//*                  LAYOUT SETUP CONFIG                                  *
//*                                                                       *
const int LAYOUTTYPE = 1;                 // Layout Types: 0 = Loop, 1 = point to point, 2 = Switchback
const int NUMBEROFSTATIONS = 2;           // Range of 1 to 4.
//*                                                                       *
//*************************************************************************
//
//  =============================================== Pin Map =====================================================================
//
//  Digital Pins
//
//  D2 Vibrate Motor (Optional for stalls)
//  D3 Motor Shield Channel A PWM
//  D4 Station C (Optional)
//  D5 Station A 
//  D6 Station B (Optional)
//  D7 Station D (Optional)
//  D8 Motor Shield Channel B Break (Turnout - Optional)
//  D9 Motor Shield Channel A Break
//  D10 Fault Light (Optional)
//  D11 Motor Shield Channel B PWM (Tunrout - Optional)
//  D12 Motor Shield Channel A Direction
//  D13 Motor Shield Channel B Direction (Tunout - Optional)
//
//  Analog Pins:
//
//  A0 Channel A Current
//  A1 Channel B Current
//  A2 Potentiometer
//  A3 Open
//  A4 IC2 OLED Display (Optional)
//  A5 IC2 OLED Diaplay (Optional)
//  A6 Nano - Mega
//  A7 Nano - Mega
//
// =============================================================================================================================

/*//To use a display, uncomment lines 103-138, 216, and 612 - 648.  Code was written for HiLetgo 128x64 4 pin I2C display. 
//This code is for an 128x64 OLED screen. You will need to install the Adafruit GFX library, Adafruit SSD1306 library, and likely Adafruit BusIO library
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO/Nano:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
#define OLED_RESET     -1 // Reset pin # (or "-1" on 4 pin screen with no reset pin)
#define SCREEN_ADDRESS 0x3C //See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32. HiLetgo 128x64 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//These are the OLED screen images

const unsigned char Broke [] PROGMEM = {
  // Fault image, 32x8px
  0xf9, 0x0f, 0xf1, 0x1f, 0x81, 0x0d, 0xb1, 0x04, 0x82, 0x8d, 0xb1, 0x04, 0xf2, 0x8f, 0xf1, 0x04, 
  0x84, 0x4e, 0x71, 0x04, 0x87, 0xcd, 0xb1, 0x04, 0x88, 0x27, 0xe1, 0x04, 0x88, 0x23, 0xc1, 0xe4
};

const unsigned char Station [] PROGMEM = {
  // 'Station, 16x8px
  0x3f, 0xf8, 0x7f, 0xfc, 0xff, 0xfe, 0x44, 0x94, 0x44, 0x94, 0x47, 0x9c, 0xff, 0xfe, 0x55, 0x55
};

const unsigned char Trolley [] PROGMEM = {
  // 'Trolley, 32x8px
  0x0f, 0xff, 0xff, 0xf0, 0x1f, 0xff, 0xff, 0xf8, 0x0a, 0xa8, 0x15, 0x50, 0x0a, 0xaa, 0x55, 0x50, 
  0x1f, 0xfa, 0x5f, 0xf8, 0x3f, 0xf8, 0x1f, 0xfc, 0x51, 0x8f, 0xf1, 0x8a, 0x86, 0x60, 0x06, 0x61
};
*/
const int CURRENTREFRESH = 10;           // Interval for current sensor refresh in ms.
const int SENSORREFRESH = 50;            // Interval for IR sensor refresh in ms.
const int DISPLAYREFRESH =1000;          // Interval for IR sensor refresh in ms.
const int SERIALREFRESH = 5000;           // Interval for serial monitor printing in ms.
const int IRDETECTDELAY = 3000;          // Delay after the train departs a station to not re-trigger the IR sensor in ms.
const int CURRENTMAX = 60;               // Maximum A0 current sense value to trigger short protection.
const int STARTSPEED =10;                 // Min PWM start speed. To adjust, set to 0. Observe slowest "Speed" on serial monitor where the train moves reliably.
const int POTMAX = 1020;                  // Maximum mv of the potentiometer for max voltage to tracks
const int POTMIN = 50;                    // Minimum mv of the potentiometer for 0 volts to tracks
const int ACCELTIME= 1000;                // Acceleration Time in ms. 
const int BREAKTIME = 300;                // Breaking Time in ms. 

unsigned long CurrentRefreshMillis = 0;   // Will store last time current values were updated.
unsigned long SensorMillis = 0;           // Will store last time speed sensor values were updated.
unsigned long SerialMillis = 0;           // Will store last time serial monitor printed.
unsigned long DisplayMillis = 0;           // Will store last time serial monitor printed.
unsigned long RateChangeMillis = 0;       // Will store last accel/decell loop time.
unsigned long StationMillis = 0;          // Will store last time a train stopped or started at a station.
unsigned long StationWait = 0;            // Stores how long to wait at a station

int CurrentA;                             // Stores the Averaged Current Value for short evaluation.
int CurrentReadA;                         // Stores the Current Value of Channel A (A0 input).
int CurrentHighA;                         // Stored the Maximum detected current Value of Channel A durring a read cycle 
int CurrentReadings;                      // Stores the number of current readings performed in the current read cycle.
int CurrentB;                             // Stores the Averaged Current Value for short evaluation.
int CurrentReadB;                         // Stores the Current Value of Channel A (A0 input).
int CurrentHighB;                         // Stores the Maximum detected current Value of Channel A durring a read cycle
int Stall;                                // Stores the Maximum detected current when leaving a station to detect stalls.
int Direction = 0;                        // Stores the train direction (0 = East, left to right; 1 = West, right to left).
int SpeedVal = 0;                         // Stores the mapped potentiometer analog value (A4 input).
int Speed = 0;                            // Stores the PWM 0 to 255 value.
int TargetSpeed = 0;                      // Stores the Target Speed. Code will adjust the Speed to match the Target Speed. 
int SpeedChange = 0;                      // Used to track speed change requests.
int Fault = 0;                            // If a fault is detected, this will be set to 1. 
int Arrive = 0;                           // Used to track arrival at a station to trigger stopping.
int Wait = 0;                             // Used to track waiting at a station.
int Depart = 0;                           // Used to track that the train is in station and stores the station number
int Accel;                                // Used to store calculated acceleration step interval in ms.
int TurnoutDirection = 2;                 // Used to track turnout direction, 0 = main, 1 = divergent
int Thumper = 0;                          // Used to track vibation motor activation.

// The following are station specific variables. Station 1 and 2 are the ends of the line.

// Station A

const long STATION1WAIT = 10000;          // Wait time in ms at Station 1. Max is 60 seconds unless timeout adjusted below.



// Station B

const long STATION2WAIT = 10000;          // Wait time in ms at Station 2.  Max is 60 seconds unless timeout adjusted below.



// Station C (Assumed to be between stations A and B if used).

const long STATION3WAIT = 10000;          // Wait time in ms at Station 3.  Max is 60 seconds unless timeout adjusted below.
const long STATION3SWAIT = 3000;          // Short wait time in ms at Station 3 for switchback.  Max is 60 seconds unless timeout adjusted below.


// Station D (Assumed to be between stations A and B if used).

const long STATION4WAIT = 10000;          // Wait time in ms at Station 4.  Max is 60 seconds unless timeout adjusted below.



// =====================================================================================================================


void setup() {


  
  Serial.begin(9600);

  // initialize and clear display
//  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);

  

  //Initilize Digital Pins
  pinMode(2,OUTPUT);                      //Vibrate Motor.
  pinMode(3,OUTPUT);                      //Motor Shield PWM A - Speed pin.
  pinMode(4,INPUT);                      //Station C Sensor
  pinMode(5,INPUT);                      //Station C Sensor 
  pinMode(6,INPUT);                      //Station C Sensor
  pinMode(7,INPUT);                      //Station C Sensor
  pinMode(8,OUTPUT);                    //Motor Shield Channel B Brake: Power on/off. 
  pinMode(9,OUTPUT);                      //Motor Shield Brake A - track power on/off.
  pinMode(10,OUTPUT);                      //Fault Light
  pinMode(11,OUTPUT);                   //Motor Shield Channel B PWM: Voltage Control.
  pinMode(12,OUTPUT);                     //Motor Shield A Direction.
  pinMode(13,OUTPUT);                   //Motor Shield Channel B Direction: Turnout Direction.
  
  // Motor Shield Channel B - Used for turnout control assuming automated two wire (Kato/Rokuhan) turnouts. Enable for turnout control.
  // ****** Warning ******
  // The included turnout code has over current protection, however you should test turnouts cautiously. It is set to a 5 ms pulse. The turnouts will need a 12 volt VIN. 
  // The higher voltage will require a 330 Ohm resistor inline to the track to reduce the voltage for Rokuhan Shorty locomotives.   
  // I had a loose header connection where the break command did not send and it melted the turnout, so I added the overcurrent protection.
  // Use at your own risk.  Test proper function by observing the motor shield LEDs before connecting the turnout. The channel B LEDs should only flash for a moment. The current sense takes 500 to 1000 ms to trip the over protection.     

  //Track power is default on. To default off, enable this code. Trigger start with a station IR sensor.  
  //digitalWrite(9,HIGH);
  digitalWrite(2,LOW);
  digitalWrite(10,LOW);
 
}


void loop() {
  
  // This code updates the current sensor variables at the defined interval.
  // Current readings are noisy because of the PWM frequency, and you may read at random off times giving 0 readings. To counter this, 50 readings are taken.  
  // The maximum reading over those readings is the current value used to average into the new value.
  if( millis() - CurrentRefreshMillis >= CURRENTREFRESH ) {
    
    CurrentRefreshMillis = millis();              //Save the last time current values were updated.
    
    //Inputs A0 and A1 are current sensors for the motor shield channels A and B respectively.
    CurrentReadA = analogRead(A0);
    CurrentReadB = analogRead(A1);
    if ( CurrentReadA > CurrentHighA) { CurrentHighA = CurrentReadA; }
    if ( CurrentReadB > CurrentHighB) { CurrentHighB = CurrentReadB; }
    CurrentReadings++;

    if(CurrentReadings >= 50) { 
      CurrentA = (CurrentA + CurrentHighA ) / 2 ;  //Current for short detection, new values averaged into previous.
      CurrentB = (CurrentB + CurrentHighB ) / 2 ;  //Current for short detection, new values averaged into previous.
      CurrentReadings = 0;
      CurrentHighA = 0;
      CurrentHighB = 0;
      if (Fault == 0 && Thumper == 0 && Depart == 0 && millis() - StationMillis < IRDETECTDELAY){ //Stall Detection Current
        if ( CurrentA > Stall) { Stall = CurrentA; }
      }
    }

    
  }
  //=====================================
 
  // This code updates the potentiometer value at the defined interval and initiates speed changes.
  if( millis() - SensorMillis >= SENSORREFRESH) {
    
    SensorMillis = millis();              //Save the last time sensors were updated.
    
    //Inputs A0 and A1 are current sensors for the motor shield.
    SpeedVal=map(analogRead(A2),POTMIN,POTMAX,0,255);     //This maps the 0-1v A2 channel to a 0-255(256) value.
    
    // If the potentiometer is below the minimum start speed, a 0 value is given.
    if(SpeedVal < STARTSPEED ){  
      SpeedVal = 0;
    } else if(SpeedVal > 255){
      SpeedVal = 255;
    }

    // This updates the TargetSpeed if a change is observed in the potentiometer, the speed isn't being adjusted, and a train isn't stopping at a station.  
    if(TargetSpeed != SpeedVal && SpeedChange == 0 && Wait == 0){  
      TargetSpeed = SpeedVal; 
      SpeedChange = 1;                                 
      RateChange();                                    // Calculates Accel loop interval value for the requested speed change.
    }
  }
  //==========================

  // This code adjusts the Speed to the TargetSpeed by 1 every Accel interval 
  if(Speed < TargetSpeed && Speed < 255 && millis() - RateChangeMillis > Accel ) {
    Speed++;                            // Advance speed by 1.
    analogWrite(3, Speed);              // Write speed to D3.
    PMWFreq();                          // Calculate PWM timer frequency. Function determines if adjustment is needed.
    RateChangeMillis = millis();        // Reset interval counter.
  }
  else if(Speed > TargetSpeed && Speed > 0 && millis() - RateChangeMillis > Accel ){
    Speed--;                            // Retards speed by 1.
    analogWrite(3, Speed);              // Write speed to D3.
    PMWFreq();                          // Calculate PWM timer frequency. Function determines if adjustment is needed.
    RateChangeMillis = millis();        // Reset interval counter.
  }
  // If a train arrived at a station, TargetSpeed was set to 0.  Once speed is 0, Arrive is cleared to proceed to Wait interval. 
  else if(Speed == 0 && Arrive == 1){
    Arrive = 0;                        
  } 

  //Once the Speed reaches the TargetSpeed, clears the adjustment lock.
  else if (Speed == TargetSpeed && SpeedChange == 1){
    SpeedChange = 0;
  }
  //==========================

  // This code monitors for track short protection and cuts power if a short is detected.
  if(CurrentA > CURRENTMAX && Fault == 0){ 
    PowerOffA();
    Serial.print("Short: Power Off: Current:"); Serial.println(CurrentA);
    Fault = 1; //Fault value set to stop loop.
    //Track power can be restored by triggering any IR sensor
  }
  //==========================

    // This code monitors for track short protection and cuts power if a short is detected.
  if(CurrentB > 50 && Fault == 0){ 
    PowerOffA();
    digitalWrite(8,HIGH);
    digitalWrite(13,LOW);
    digitalWrite(11,0);
    Serial.print("Turnout Short. Current:"); Serial.println(CurrentB);
    Fault = 1; //Fault value set to stop loop.
    //Track power can be restored by triggering any IR sensor
  }
  //==========================

  // This code turns off track power if a train fails to arrive at a station after 61 seconds
  if (Fault == 0 && Depart == 0 && millis() - StationMillis >= 61000){
    PowerOffA();
    Serial.println("Stall or Derailment");
    Fault = 1; //Fault value set to stop loop.
    //Track power can be restored by triggering any IR sensor
  }
  //==========================

  
  // This code activates a vibration motor if the current reads 0 after departing a station (stall) to attempt to correct the stall.  
  if (Stall < 5 && Fault == 0 && Thumper == 0 && Depart == 0 && millis() - StationMillis > 1000 && millis() - StationMillis < IRDETECTDELAY){
  digitalWrite(2,HIGH);
  delay(1000);
  digitalWrite(2,LOW);
  Thumper = 1;
  }
  //==========================

  // This code determins if a station sensor has been triggered and assigns which station the train is at.
  if(Depart == 0 && millis() - StationMillis >= IRDETECTDELAY && digitalRead(5) == LOW){
    Depart = 1;
    StationArrive();
  }
  else if(Depart == 0 && millis() - StationMillis >= IRDETECTDELAY && NUMBEROFSTATIONS > 1 && digitalRead(6) == LOW){
    Depart = 2;
    StationArrive();
  }
  else if(Depart == 0 && millis() - StationMillis >= IRDETECTDELAY && NUMBEROFSTATIONS > 2 && digitalRead(4) == LOW){
    Depart = 3; 
    StationArrive();
  }
  else if(Depart == 0 && millis() - StationMillis >= IRDETECTDELAY && NUMBEROFSTATIONS == 4 && digitalRead(7) == LOW){
    Depart = 4; 
    StationArrive();
  }

  //==========================
  

  // This code waits for Arrive to clear, powers off the track, begins the wait interval counting, and clears Wait. 
  if ( Arrive == 0 && Wait == 1 && Depart > 0 ){        
    PowerOffA();
    StationMillis = millis();
    Wait = 0;
    switch (Depart) {
      case 1: 
        StationWait = STATION1WAIT;
        if (LAYOUTTYPE > 0){
          Direction = 0; 
          DirectionEast();
        }
        if (LAYOUTTYPE == 2 && TurnoutDirection != 0){
          TurnoutMain();
        }
        break;
      case 2:
        StationWait = STATION2WAIT;
        if (LAYOUTTYPE > 0){
          Direction = 1; 
          DirectionWest();
        } 
        if (LAYOUTTYPE == 2){
          Direction = 1; 
          DirectionEast();
          if (TurnoutDirection != 1){
            TurnoutDivergent();
          }
        } 
        break;
      case 3:
      StationWait = STATION3WAIT; 
      if (LAYOUTTYPE == 2){
        StationWait = STATION3SWAIT;
        if(Direction == 1){
          DirectionWest();
          TurnoutMain();
        } 
        if(Direction == 0){
          DirectionWest(); 
          TurnoutDivergent();
        }
      }       
      break;
      case 4: 
      StationWait = STATION4WAIT; break;

      default: break;
    } 
  } 

  // This code clears the station for departure, and accelerates the train.
  if ( Arrive == 0 && Wait == 0 && Depart > 0 && millis() - StationMillis >= StationWait ){          
    StationMillis = millis();
    Depart = 0;
    Stall = 0;
    Speed = STARTSPEED;
    PowerOnA();
    TargetSpeed = SpeedVal;
    SpeedChange = 1;
    RateChange();
    // Note: If the potentiometer is set below the STARTSPEED, the train will likely re-trigger the station when adjusted. 
  }
  //==========================

  
  
  // This code prints key values to the serial monitor for debugging at the SERIALREFRESH interval.
  if (millis() - SerialMillis >= SERIALREFRESH) {

    SerialMillis = millis();  //Saves the last time serial monitor was updated.
   
    Serial.print("Direction:"); Serial.println(Direction);
    Serial.print("Spped Pot:"); Serial.println(analogRead(A2));
    Serial.print("Speed:"); Serial.println(Speed);
    Serial.print("Current A:"); Serial.println(CurrentA);
    Serial.print("Depart:"); Serial.println(Depart);
    Serial.println("===================");
  } 
  //=========================

  // This code prints key values to the serial monitor for debugging at the SERIALREFRESH interval.
  if (millis() - DisplayMillis >= DISPLAYREFRESH) {
    DisplayMillis = millis();  //Saves the last time serial monitor was updated.
    OledUpdate();
  } 
  //=========================

}


// =========================================== Non Loop Functions ======================================================



// This function adjusts the PMW frequency used for Pins 3 & 11 (timer adjustment) 
void PMWFreq(){
/*
  Setting   Divisor   Frequency (Hz)
    0x01       1      31372.55   <-- Quiet but stalls at slow speed
    0x02       8       3921.16
    0x03      32        980.39
    0x04      64        490.20   <-- DEFAULT - Noisy
    0x05     128        245.10
    0x06     256        122.55
    0x07    1024         30.64   <-- More like "Snail" speed controller

    TCCR2B = (TCCR2B & 0b11111000) | <setting>; < -- Format for changes
*/
  #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)

    //this will compile for Arduino UNO, Nano, Pro, and older boards
    if(Speed <= 100 && TCCR2B != 7){
      TCCR2B = TCCR2B & 0b11111000 | 0x07;
    }
    else if(Speed > 100 && TCCR2B != 1){
      TCCR2B = TCCR2B & 0b11111000 | 0x01;
    }

  #elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

    //this will compile for Arduino Mega
    if(Speed <= 100 && TCCR2A != 7){
      TCCR2A = TCCR2A & 0b11111000 | 0x07;
    }
    else if(Speed > 100 && TCCR2A != 1){
      TCCR2A = TCCR2A & 0b11111000 | 0x01;
    }

  #else

  #error “Unsupported board selected!”

  #endif

}



// This function activates when a station sensor is triggered and brings the train to a stop.
void StationArrive() {
  Arrive = 1;
  Wait = 1;
  Fault = 0;
  Thumper = 0;
  TargetSpeed = 0;
  SpeedChange = 1;
  RateChange();
}


// Turns track power off.
void PowerOffA() {
  digitalWrite(12,LOW); 
  digitalWrite(9,HIGH);
}


// Turns track power on.
void PowerOnA() {
  digitalWrite(9,LOW);
}


// Turns turnout to main.
void TurnoutMain() {
    digitalWrite(11,255);
    digitalWrite(8,LOW);
    digitalWrite(13,LOW);
    //digitalWrite(13,LOW);
    delay(5);
    digitalWrite(8,HIGH);
    digitalWrite(13,LOW);
    digitalWrite(11,0);
    TurnoutDirection = 0;
}

// Turns turnout divergent.
void TurnoutDivergent() {
    digitalWrite(11,255);
    digitalWrite(8,LOW);
    //digitalWrite(13,LOW);
    digitalWrite(13,HIGH);
    delay(5);
    digitalWrite(8,HIGH);
    digitalWrite(13,LOW);
    digitalWrite(11,0);
    TurnoutDirection = 1;
}


// Sets direction East: Left to Right.
// Old railroading convention.  Trains heading towards San Francisco are westbound, regardless of actual direction of travel.
void DirectionEast() {
  digitalWrite(12,LOW);
}


// Sets direction West: Right to Left.
void DirectionWest() {
  digitalWrite(12,HIGH);
}


// Calculates the acceleration or deceleration interval values to use for a requested speed change
void RateChange() {

   if(Speed < TargetSpeed){
      Accel = ACCELTIME / (TargetSpeed - Speed);
   }

   else if(TargetSpeed < Speed){
      Accel = BREAKTIME / (Speed - TargetSpeed);
   }
   //This should only occur if you adjust the speed to 0 and trigger a station manually.
   else if(Speed == TargetSpeed){
     Serial.println("URA Failure");
   }

}

// Updates values shown on the OLED display
void OledUpdate() {
/*
  display.clearDisplay(); // clear display
  display.setRotation(2);
  if(Fault == 1){    
    display.drawBitmap(47,0,Broke,32,8,WHITE);
  } else if(Depart == 1){
    display.drawBitmap(0,0,Station,16,8,WHITE);
    display.drawBitmap(17,0,Trolley,32,8,WHITE);
  } else if(Depart == 2){
      display.drawBitmap(79,0,Trolley,32,8,WHITE);
      display.drawBitmap(111,0,Station,16,8,WHITE);
  } else if(Depart == 0 && Direction == 0){
      display.setTextSize(1);      // text size
      display.setTextColor(WHITE); // text color
      display.setCursor(40, 2);    // position to display
      display.println("~");
      display.drawBitmap(47,0,Trolley,32,8,WHITE);
  } else if(Depart == 0 && Direction == 1){
      display.drawBitmap(47,0,Trolley,32,8,WHITE);
      display.setTextSize(1);      // text size
      display.setTextColor(WHITE); // text color
      display.setCursor(82, 2);    // position to display
      display.println("~");
  }
  display.drawFastHLine(0,9,127,WHITE);
  display.setTextSize(2);      // text size
  display.setTextColor(WHITE); // text color
  display.setCursor(0, 17);    // position to display
  if(Fault > 0 | Depart > 0){
    display.println("Speed:0");
  } else {
    display.print("Speed:"); display.println(map(Speed,0,255,0,100));
  }
  display.setCursor(0, 40);    // position to display
  display.print("Current:"); display.println(CurrentA);
  display.display();
*/ 
}
