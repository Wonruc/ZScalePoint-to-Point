//                           Arduino Uno/Nano L298 Motor Shield Automatic Point-to-Point DC Train Control
//
//                                            Copyright 2023 Matthew Curnow
//                            
//                                                  Updated 16-Jul-2023
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
//  ===========================================================================================================================
//  
//  This program is intended to utilize an Arduino Uno or Nano micro controller, a L298P or L298N motor shield, and IR sensors
//  to automate the operation of a Point-to-Point DC powered model railroad.  The motor shield acts as a PWM DC power supply to
//  the tracks.  The analog input of a 10 ohm potentiometer is used to adjust the motor shields PWM/voltage sent to the tracks 
//  for speed control.  When a train enters a station, the IR sensor is triggered, the train slows to a stop, waits for a 
//  defined interval, the polarity of the track may be changed as needed to send the train to the next station, and the train 
//  is set to accelerate out of the station.  This code includes short protection. The constants used to identify over
//  current, start speed, acceleraation and deceleration  were established using 3V Rokuhan Shorty Z scale trains for the 
//  Showcase Miniatures trolley and may need to be adjusted for larger trains at higher voltages. If a train fails to arrive at
//  a station after one minute, a stall is assumed and track power is turned off.  Faults can be cleared by triggering any
//  station IR sensor.
//
//  ===========================================================================================================================


const long CURRENTREFRESH = 10;           // Interval for current sensor refresh in ms.
const long SENSORREFRESH = 100;           // Interval for IR sensor refresh in ms.
const long SERIALREFRESH = 5000;          // Interval for serial monitor printing in ms.
const long IRDETECTDELAY = 2000;          // Delay after the train departs a station to not re-trigger the IR sensor in ms.
const int CURRENTMAX = 70;                // Maximum A0 current sense value to trigger short protection.
const int STARTSPEED =15;                 // Min PWM start speed. To adjust, set to 0. Observe slowest "Speed" on serial monitor where the train moves reliably.
const int POTMAX = 1020;                  // Maximum mv of the potentiometer for max voltage to tracks
const int POTMIN = 50;                    // Minimum mv of the potentiometer for 0 volts to tracks
const int ACCELTIME= 2000;                // Acceleration Time in ms. 
const int BREAKTIME = 300;                // Breaking Time in ms. 

unsigned long CurrentRefreshMillis = 0;   // Will store last time sensor values were updated.
unsigned long SensorMillis = 0;           // Will store last time sensor values were updated.
unsigned long SerialMillis = 0;           // Will store last time serial monitor printed.
unsigned long RateChangeMillis = 0;       // Will store last accel/decell loop time.
unsigned long StationMillis = 0;          // Will store last time a train stopped or started at a station.
unsigned long StationWait = 0;            // Stores how long to wait at a station

int CurrentA;                             // Stores the Averaged Current Value for short evaluation.
int CurrentReadA;                         // Stores the Current Value of Channel A (A0 input).
int CurrentHighA;                         // Stored the Maximum detected current Value of Channel A durring a read cycle 
int CurrentReadingsA;                     // Stores the number of current readings performed in the current read cycle.
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
int Voltage = 0;


// The following are station specific variables. Station 1 and 2 are the ends of the line.

// Station 1

const long STATION1WAIT = 10000;          // Wait time at Station 1. Max is 60 seconds unless timeout adjusted below.
int Station1;                             // Stores Station 1 IR sensor value in mV (A2 Input)


// Station 2

const long STATION2WAIT = 10000;          // Wait time at Station 2.  Max is 60 seconds unless timeout adjusted below.
int Station2;                             // Stores Station 2 IR sensor value in mV (A3 Input)


// Station 3 (Assumed to be between stations 1 and 2 if used).  Copy code for additional stations 

//const long STATION3WAIT = 500;          // Wait time at Station 3.  Max is 60 seconds unless timeout adjusted below.
//int Station3;                             // Stores Station 3 IR sensor value in mV (A5 Input)


// =====================================================================================================================



void setup() {

  // Motor Shield Channel A - Used for track power
  Serial.begin(9600);
  pinMode(3,OUTPUT);                      //Motor Shield PWM A - Speed pin. 
  pinMode(9,OUTPUT);                      //Motor Shield Brake A - track power on/off.
  pinMode(12,OUTPUT);                     //Motor Shield A Direction.
  
  // Motor Shield Channel B - Used for turnout control assuming automated two wire (Kato/Rokuhan) turnouts. Enable for turnout control.
  // ****** Warning ******
  // The included turnout code has no over current protection. It is set to a 5 ms pulse. The turnouts will need a 12 volt VIN. 
  // The higher voltage will require a 330 Ohm resistor inline to the track to reduce the voltage for Rokuhan Shorty locomotives.   
  // I had a loose header connection where the break command did not send and it melted the turnout. 
  // Use at your own risk.  Test proper function by observing the motor shield LEDs before connecting the turnout.    
  //pinMode(11,OUTPUT);                   //Motor Shield Channel B PWM: Voltage Control. 
  //pinMode(8,OUTPUT);                    //Motor Shield Channel B Brake: Power on/off.
  //pinMode(13,OUTPUT);                   //Motor Shield Channel B Direction: Turnout Direction.
  
  //Track power is default on. To default off, enable this code. Trigger start with a station IR sensor.  
  //digitalWrite(9,HIGH);
  
}


void loop() {
  
  // This code updates the current sensor variables at the defined interval.
  // Current readings are noisy because of the PWM frequency, and you may read at random off times giving 0 readings. To counter this, 50 readings are taken.  
  // The maximum reading over those readings is the current value used to average into the new value.
  if( millis() - CurrentRefreshMillis >= CURRENTREFRESH ) {
    
    CurrentRefreshMillis = millis();              //Save the last time current values were updated.
    
    //Inputs A0 and A1 are current sensors for the motor shield channels A and B respectively.
    CurrentReadA = analogRead(A0);
    if ( CurrentReadA > CurrentHighA) { CurrentHighA = CurrentReadA; }
    CurrentReadingsA++;

    if(CurrentReadingsA >= 50) { 
      CurrentA = (CurrentA + CurrentHighA ) / 2 ;  //Current for short detection, new values averaged into previous.
      CurrentReadingsA = 0;
      CurrentHighA = 0;
    }

    
  }
 
  // This code updates the sensor variables at the defined interval and initiates speed changes.
  if( millis() - SensorMillis >= SENSORREFRESH) {
    
    SensorMillis = millis();              //Save the last time sensors were updated.
    
    //Inputs A0 and A1 are current sensors for the motor shield.
    Station1=analogRead(A2);              //Station 1 IR sensor.
    Station2=analogRead(A3);              //Station 2 IR sensor.
    SpeedVal=map(analogRead(A4),POTMIN,POTMAX,0,255);     //This maps the 0-1v A4 channel to a 0-255(256) value.
    
    // If the potentiometer is below the minimum start speed, a 0 value is given.
    if(SpeedVal < STARTSPEED ){  
      SpeedVal = 0;
    } else if(SpeedVal > 255){
      SpeedVal = 255;
    }
    //Station3=analogRead(A5);            //Station 3 IR sensor. Enable if using station 3.
    //Arduino Nano A6 A7 available for additional sensors.

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

  // This code monitors for short protection and cuts power if a short is detected.
  if(CurrentA > CURRENTMAX && Fault == 0){ 
    PowerOffA();
    Serial.println((String)"Short Detected, Track Power Off: Current Value of:" + CurrentA);
    Fault = 1; //Fault value set to stop loop.
    //Track power can be restored by triggering any IR sensor
  }
  //==========================

  // This code turns off track power if a train fails to arrive at a station after 61 seconds
  if (Fault == 0 && Depart == 0 && millis() - StationMillis >= 61000){
    PowerOffA();
    Serial.println("Train didn't arrive at station. Likely a stall or derailment.");
    Fault = 1; //Fault value set to stop loop.
    //Track power can be restored by triggering any IR sensor
  }
  //==========================

  // This code determins if a station sensor has been triggered and assigns which station the train is at.
  if(Depart == 0 && millis() - StationMillis >= IRDETECTDELAY && Station1 < 100){
    Depart = 1;
    StationArrive();
  }
  else if(Depart == 0 && millis() - StationMillis >= IRDETECTDELAY && Station2 < 100){
    Depart = 2;
    StationArrive();
  }
//  else if(Depart == 0 && millis() - StationMillis >= IRDETECTDELAY && Station3 < 100){ Depart = 3; StationArrive(); }

  //==========================
  

  // This code waits for Arrive to clear, powers off the track, begins the wait interval counting, and clears Wait. 
  if ( Arrive == 0 && Wait == 1 && Depart > 0 ){        
    PowerOffA();
    StationMillis = millis();
    Wait = 0;
    switch (Depart) {
      case 1: StationWait = STATION1WAIT; Direction = 0; DirectionEast();break;
      case 2: StationWait = STATION2WAIT; Direction = 1; DirectionWest();break;
//      case 3: StationWait = STATION3WAIT; 

/*      // Code if station 3 is a switchback.  You also change case 2 to DirectionEast().
        if(Direction == 1){
          DirectionWest();
          //TurnoutMain();
        } 
        if(Direction == 0){
          DirectionWest(); 
          //TurnoutDivergent();
        }
*/        
//      break;
      
//      case 4: StationWait = STATION4WAIT; break;
//      case 5: StationWait = STATION5WAIT; break;
      default: break;
    } 
  } 

  // This code clears the station for departure, and accelerates the train.
  if ( Arrive == 0 && Wait == 0 && Depart > 0 && millis() - StationMillis >= StationWait ){          
    StationMillis = millis();
    Fault = 0; //Clear any faults.
    Depart = 0;
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
    
//    Serial.println((String)"Millis: " + (millis() - SerialMillis));
    SerialMillis = millis();  //Saves the last time serial monitor was updated.
   
//    Serial.println((String)"Direction: " + Direction);
//    Serial.println((String)"Speed Pot:" + analogRead(A4));
//    Serial.println((String)"SpeedVal:" + SpeedVal);
    Serial.println((String)"Speed:" + Speed);
//    Serial.println((String)"TargetSpeed: " + TargetSpeed);
//    Serial.println((String)"Accel: " + Accel);
//    Serial.println((String)"Sensor 1: " + Station1);
//    Serial.println((String)"Sensor 2: " + Station2);
//    Serial.println((String)"Channel A Current: " + CurrentA);
//    Serial.println((String)"Arrive:" + Arrive);
//    Serial.println((String)"Wait: " + Wait);
//    Serial.println((String)"Depart: " + Depart);
//    Serial.println((String)"StationWait: " + StationWait);
    Serial.println("===================");
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
  if(Speed <= 127 && TCCR2B != 7){
    TCCR2B = TCCR2B & 0b11111000 | 0x07;
  }
  else if(Speed > 127 && TCCR2B != 1){
    TCCR2B = TCCR2B & 0b11111000 | 0x01;
  }
}



// This function activates when a station sensor is triggered and brings the train to a stop.
void StationArrive() {
  Arrive = 1;
  Wait = 1;
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
/*
// Turns turnout divergent.
void TurnoutDivergent() {
    digitalWrite(11,255);
    digitalWrite(8,LOW);
    //digitalWrite(13,LOW);
    digitalWrite(13,HIGH);
    delay(5);
    digitalWrite(8,HIGH);
    digitalWrite(13,LOW);
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
}
*/


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



  
  
