/****************************************
 * 
 *    Finite State Machines
 *    http://digitaldiy.io/articles/mcu-programming/general-programming/500-finite-state-machines#.WXds7HXyveR
 * 
 * 
 */


#include <Arduino.h>
#include "HX711.h"
#include <Wire.h>
#include "I2C_Anything.h"

#define BREW_SWITCH 10
#define PUMP_CONTROL 9
#define PIEZO 5

const byte SLAVE_TWO_ADDRESS =  43;   // slave address of mass sender
volatile double mass;               // 4 bytes        Values for sending
volatile int _brewSwitchState;       // 2 bytes        over I2C bus
volatile boolean extractionComplete;// 1 byte         to Master

volatile double preinfuseTime = 0;      // 4 bytes
volatile double extractionTime = 0;     // 4 bytes

//int switchState;
//int lastSwitchState;

int numPulses = 3;
int pulseDuration = 1000;
int pulseGap = 2000;
long unsigned int maxShotTime = 50000;
int brewSwitchState;

HX711 scale;
double reading;
//------------------------------------------------

void setup() {
  Serial.begin(115200);
  Wire.begin(SLAVE_TWO_ADDRESS);
  Wire.onRequest(requestEvent);

  pinMode(BREW_SWITCH, INPUT);
  pinMode(PUMP_CONTROL, OUTPUT);
  pinMode(PIEZO, OUTPUT);

  scale.begin(A0, A1);
  scale.set_scale();
  scale.tare();
                                                                    
  double getUnits = scale.get_units(10);                            // take 10 readings
  //double parameter = getUnits / 56.70;
  double parameter = 1283.95;   
  scale.set_scale(parameter);
  scale.tare();
  reading = 0.00;

  
}
//------------------------------------------------

void loop(){
  //static unsigned long startTime = 0;     // used to measure 
  //static unsigned long elapsedTime = 0;   // used to measure total elapsed time
  static int state = 1;                   // initial state is 1, the "idle" state.
  static unsigned long ts = 0;            // To store the "current" time in for delays.  
  static unsigned long timeNow = 0;
  static int counter1 = 0;                   // used for determining number of preinfusion pulses
  static int counter2 = 0;                // used for determining number of data points (default = ???)
  static int switchState;
  static int lastSwitchState;
  //unsigned long timeLast = 0;
    

  switch (state) {
    
    case 1:
      counter1 = 0;
      counter2 = 0;      
      extractionTime = 0;                           // initialise for 
      preinfuseTime = 0;                            // preinfuse & extraction
      timeNow = 0;
      
      //Serial.println(F("State 1: waiting for Brew Switch"));

      readBrewSwitchState();
            
      if ((_brewSwitchState == 1) && (lastSwitchState == 0)){   // if switch has been activated
        scale.tare();                       // tare scales        
        extractionComplete = false;
        
        
        Serial.println(F("Brew Switch ON !!!"));
        state = 2;                                    // Now move to state 2
        Serial.println(F("Moving to State 2"));                
      }
      else {                                    
        state = 1;                                    // otherwise do nothing, wait for switch
        
      }
      lastSwitchState = _brewSwitchState;                      // remember for next time through the loop
      break;
    
    case 2:     
      Serial.println("State 2: Turn Pump On");      
      ts = millis(); 
      Serial.print(F("Time State 2 : \t\t"));      Serial.println(ts);
      state = 3;                          // move to next state
      Serial.println(F("Moving to State 3"));
                           // take a note of the time
      digitalWrite(PUMP_CONTROL, HIGH);   // turn pump on
      break;

    case 3:
      timeNow = millis();        
        //Serial.println(F("State 3: Preinfuse pulse"));
        Serial.print(F("Time State 3 :\t\t"));  Serial.println(timeNow);
        if (timeNow - ts  >= pulseDuration){   // pulseDuration default = 1000ms
          preinfuseTime += pulseDuration;         // increment preinfuse time for printing to LCD
          Serial.print(F("Preinfuse Time : ")); Serial.println(preinfuseTime); Serial.print(F("\n"));
          state = 4;
          Serial.println(F("Moving to State 4"));
        }        
        else {
          state = 3;
          //if(timeNow - ts >= 250)
          //  Serial.println(F("Remaining State 3"));
        }      
      break;

    case 4:
      Serial.println(F("State 4: Begin Preinfuse Pulse Gap"));
      digitalWrite(PUMP_CONTROL, LOW);        // turn off pump
      ts = millis();                          // take a note of the time
      state = 5;
      Serial.println(F("Moving to State 5"));
      break;

    case 5:                        
      timeNow = millis();
      
      readBrewSwitchState();
      
      if ((_brewSwitchState == 0) && (lastSwitchState == 1)){   // if switch has been turned OFF --- FLUSH MODE
        state = 1;
        digitalWrite(PUMP_CONTROL, LOW);                                     // turn pump off
        extractionComplete = true;  
        Serial.println(F("Extraction ended by user"));
      }      

      else{
        //Serial.print(F("State 5: Measure pulse gap 2 sec, count number of pulses: ")); Serial.println(counter1);
        
        if (timeNow - ts >= pulseGap) {      // pulseGap default = 2000ms
          Serial.print(F("Pulse gap time elapsed: ")); Serial.println(timeNow - ts); Serial.print(F("\n"));
          preinfuseTime += pulseGap;            // keep track of elapased time
          counter1++;                         // counter for numPulses
          state = 6;
          Serial.println(F("Moving to State 6"));
        }
        else {
          state = 5;
          //Serial.println(F("Remaining State 5"));
        }
      }
      lastSwitchState = _brewSwitchState;
      break;
      
      
    case 6:
      if (counter1 < numPulses) {         // numPulses default = 3
        state = 2;
        Serial.println(F("Moving back to State 2 for more preinfuse pulses"));
      }
      else {
        state = 7;
        Serial.println(F("Finished Preinfusion: Moving to State 7"));        
      }            
      break;

    case 7:      
      readBrewSwitchState();
      
      if(_brewSwitchState == 0 && lastSwitchState == 1) {   // brew switch turned off
        Serial.print(F("\n\n FINAL  EXTRACTION  TIME : ")); Serial.println(extractionTime + ((pulseDuration + pulseGap) * numPulses)); Serial.println(F("\n\n"));
        Serial.println(F("Returning to State 1"));
        state = 1;
      }

      else {
        Serial.print("Total Preinfuse Time : "); Serial.println(preinfuseTime);
      
        Serial.println(F("State 7: Extracting"));
        digitalWrite(PUMP_CONTROL, HIGH);   // turn on pump
        ts = millis();                    // take note of time
        state = 8;
        Serial.println(F("Moving to State 8"));
      }
      lastSwitchState = _brewSwitchState;
        break;

    case 8:            
      //Serial.println(F("State 8: Measure 1/2 sec for data interval"));
      timeNow = millis();
       
      if (timeNow - ts >= 500) {          // reporting data every 1/2 sec
        Serial.println(F("\n 1/2 sec passed\n"));
        state = 9;               
        extractionTime += 500;              // add 1/2 sec to extractionTime          
        Serial.println(F("Moving to State 9"));           
      }
      else {                              // keep waiting for 1/2 sec to pass
        state = 8;
        //Serial.println(F("Remain State 8"));
      }
      break;
      
           
    case 9:
      Serial.println(F("State 9: print data"));
      Serial.print(F("\nExtraction Elapsed Time : ")); Serial.println(extractionTime); Serial.print(F("\n"));
      mass = scale.get_units();
      Serial.print(F("\t\tTotal mass: ")); Serial.println(mass);    Serial.print(F("\n"));// and mass
      counter2++;           // counter2 measures num of 1/2 sec units
      Serial.print(F("Counter 2 : \t\t\t"));  Serial.println(counter2); Serial.println(F("\n"));
      state = 10;
      Serial.println(F("Moving to State 10"));
      break;

    case 10:      
      if (counter2 < (maxShotTime / 500)) {                           
        Serial.print(F("State 10: counting data points:\t"));           
        Serial.print(F("Number of data points: \t")); Serial.println(counter2);
        Serial.println(F("\nMoving back to State 7"));
        state = 7;
      }
      else {
        state = 11;        
        Serial.println(F("Max Shot Time Elapsed:\t Moving to State 11"));        
      }
      break;

    case 11:                                                            ///  if we have exceeded max shot timr.
      Serial.print(F("State 11 \n"));                                     // auto shut off & alarm sounded
      digitalWrite(PUMP_CONTROL, LOW);
      Serial.print(F("TURN BREW SWITCH OFF!!!!!!!!!!!!\t\t"));Serial.println("Sound Buzzer");
      Serial.print(F("\n\nTotal Extraction Time : "));  Serial.println(extractionTime); Serial.print(F("\n"));
      ts = millis();
      tone(PIEZO, 1000);      // pin, freq
      state = 12;  
      Serial.println(F("Moving to State 12"));  
      //}
      //extractionTime += (timeNow - ts);
      //ts = millis();      
      break;
      

    case 12:
      timeNow = millis();
      //Serial.println(F("State 12: waiting for 2 sec buzzer ON interval"));
      if (timeNow - ts >= 2000) {  
        Serial.print(F("Moving to State 13 \t"));              // 2 sec has passed, move on      
        state = 13;
      }
      else {
        state = 12;                                           // wait for 2 sec to pass
        //Serial.println(F("Remaining State 12"));
      }
      break;

    case 13:
      Serial.println(F("State 13: Turn Buzzer OFF"));
      noTone(PIEZO);    
      ts = millis();
      state = 14;
      Serial.println(F("Moving to State 14"));
      break;

    case 14:
      timeNow = millis();
      if (timeNow - ts >= 3000) {                 // 3 sec has passed, so move on        
        Serial.println(F("Finished waiting for Buzzer Timer, going to state 15"));      
        state = 15;        
      }
      else {
        state = 14;                                 // keep waiting for 3 sec to pass
        //Serial.println(F("State 14: Waiting for 3 sec"));
        //Serial.println(F("Remaining State 14"));
      }
      break;

    case 15:
      readBrewSwitchState();

      if(_brewSwitchState == 0 && lastSwitchState == 1){      // switch has now been turned off
        state = 1;   
        Serial.println(F("User has turned off Brew Switch, so returning to state 1, from FINAL state 15"));    
      }
      else {              // switch is still on
        state == 11;      
        Serial.println(F("Still waiting for user to turn off brew switch"));          
      } 
      lastSwitchState = _brewSwitchState;
      break;


    default:
      break;

  }   // end switch statement

} // end loop



void requestEvent() {
  I2C_writeAnything(_brewSwitchState);
  I2C_writeAnything(mass);
  I2C_writeAnything(extractionComplete);
  I2C_writeAnything(preinfuseTime);
  I2C_writeAnything(extractionTime);

  Serial.print(F("/n/nBrewSwitchState : "));        Serial.println(_brewSwitchState);
  Serial.print(F("Mass : \t\t\t"));                 Serial.println(mass);
  Serial.print(F("Extraction complete ??\t\t\t"));  Serial.println(extractionComplete);
  Serial.print(F("Preinfuse Time : \t\t"));         Serial.println(preinfuseTime);
  Serial.print(F("Extraction Time : \t\t"));        Serial.println(extractionTime);
  

  
}

void readBrewSwitchState(){
  for (int i = 0; i < 10; i++) {
    brewSwitchState += digitalRead(BREW_SWITCH);    // read brewSwitch 10 times
    delay(10);                                      // to prevent false positives or negatives
  }
  Serial.print(F("10 readings: ..."));
  Serial.println(brewSwitchState);
      
  if (brewSwitchState > 5){
    _brewSwitchState = 1;                         // brew switch is on
    extractionComplete = false;                   // so extraction is not yet complete
    //Serial.println(F("Brew Switch ON"));
  }
  else {
    _brewSwitchState = 0;                         // otherwise brew switch is off
    extractionComplete = true;                    // so extraction is now complete
  }

  brewSwitchState = 0;                            // reset for next time  
  
}
/*
 #include <Arduino.h>

#include "HX711.h"
#include <Wire.h>
#include "I2C_Anything.h"

#define BREW_SWITCH 10
#define PUMP_CONTROL 9

const byte SLAVE_TWO_ADDRESS =  43;   // slave address of mass sender
volatile double mass;               // 4 bytes        Values for sending
volatile int brewSwitchState;       // 2 bytes        over I2C bus
volatile boolean extractionComplete;// 1 byte         to Master

int numPulses = 3;
int pulseDuration = 1000;
int pulseGap = 2000;
int maxShotTime = 40000;

int test = 0;
unsigned long timeNow = 0;
unsigned long timeLast = 0;
unsigned long readingTimeNow = 0;
unsigned long readingTimeLast = 0;
unsigned long startTime;
unsigned long endTime;


HX711 scale;
double reading;


void setup() {
  Serial.begin(115200);
  Wire.begin(SLAVE_TWO_ADDRESS);
  Wire.onRequest(requestEvent);

  pinMode(BREW_SWITCH, INPUT);
  pinMode(PUMP_CONTROL, OUTPUT);

  scale.begin(A0, A1);
  scale.set_scale();
  scale.tare();

  double getUnits = scale.get_units(10);
  //double parameter = getUnits / 56.70;
  double parameter = 1283.95;   
  scale.set_scale(parameter);
  scale.tare();
  reading = 0.00;

}


void loop() {
  brewSwitchState = digitalRead(BREW_SWITCH);
  //Serial.print("Switch state : ");
  //Serial.println(brewSwitchState);

  if (brewSwitchState) {           // if brew switch ON
    scale.tare();   // tare scale ready for extraction
    startTime = millis();
    Serial.print("Start time : ");
    Serial.println(startTime);

    for (int i = 0; i < numPulses; i++) {
      extractionComplete = false;

      Serial.println("Pulsing pump for 1 sec");
      digitalWrite(PUMP_CONTROL, HIGH);           // preinfusion
      delay(pulseDuration);                       // routine
      Serial.println("Pump off for 2 sec");
      digitalWrite(PUMP_CONTROL, LOW);
      delay(pulseGap);
    }

    timeLast = millis();
    timeNow = timeLast;         // get the  time before loop starts
    readingTimeNow = timeLast;

    while ((timeNow - timeLast) <= 60000) {  // run pump for 40s or until      
      digitalWrite(PUMP_CONTROL, HIGH);     // user turns off brew switch
      extractionComplete = false;
      //brewSwitchState = digitalRead(BREW_SWITCH);
      brewSwitchState = 0;
      for (int i = 0; i < 10; i++) {
        brewSwitchState += digitalRead(BREW_SWITCH);
        delay(10);
      }
      //Serial.print("10 state: ");
      //Serial.println(brewSwitchState);
      if (brewSwitchState > 5){
        brewSwitchState = 1;
      }
      else {
        brewSwitchState = 0;
      }
      //brewSwitchState = test;
      
      if (!brewSwitchState) {
        Serial.println(brewSwitchState);
        Serial.println("BREW SWITCH OFF!!!!!");
        digitalWrite(PUMP_CONTROL, LOW);  // turn pump off and
        extractionComplete = true;        // tell master we are finished
        Serial.println("Extraction Complete");
        unsigned long timeCompleted = millis();     
        Serial.print("Time completed: ");
        Serial.println(timeCompleted);  
        Serial.print("Total time: ");
        Serial.println((timeCompleted - startTime) / 1000.0); // + (pulseDuration + pulseGap) * numPulses);
        Serial.print("Total mass: ");
        Serial.println(scale.get_units());
        delay(1000);
        break;                          // break out of while loop
      }
      
      readingTimeNow = millis();
      //startTime = readingTimeNow;

      if ((readingTimeNow - readingTimeLast) >= 500) { //read mass twice every second
        readingTimeNow = millis();
        reading = scale.get_units();
        mass = reading;       // read into volatile for sending on I2C
        Serial.print("Mass : ");
        Serial.println(reading);
        readingTimeLast = readingTimeNow;
      }
      //Serial.println("Pump running");
      timeNow = millis();

    }
    //digitalWrite(PUMP_CONTROL, LOW);   // turn off Pump
  }

  else {
    digitalWrite(PUMP_CONTROL, LOW);
    brewSwitchState = 0;
    mass = 0.0;
    //Serial.println("Pump off");
  }
}

void requestEvent() {
  I2C_writeAnything(brewSwitchState);
  I2C_writeAnything(mass);
  I2C_writeAnything(extractionComplete);
}

*/
