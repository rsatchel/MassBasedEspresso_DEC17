//////////////////////////////////////////////
//       Arduino Rotary Encoder Menu        //
//                 v1.0                     //
//           http://www.educ8s.tv           //
/////////////////////////////////////////////

//  https://create.arduino.cc/projecthub/nickthegreek82/arduino-menu-on-a-nokia-5110-lcd-using-a-rotary-encoder-f62f0f?f=1

//  page = 1 is the  menu items
//  page == 2 is adjusting values


/*******************************************

   MENU CURRENTLY WORKING WITH 8 ELEMENTS, NEED TO FIX CONTRAST

 * ****************************************/

//      http://educ8s.tv/arduino-rotary-encoder-menu/

#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <ClickEncoder.h>
#include <TimerOne.h>
#include "RTClib.h"
#include <Wire.h>
#include "I2C_Anything.h"

#define BACKLIGHT_PIN 7

const byte SLAVE_ONE_ADDRESS = 42;    // slave one is the PID module
const byte SLAVE_TWO_ADDRESS = 43;    // slave two is the HX711 and Brew Switch Module

RTC_DS1307 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

//*********************************
//*
// * Make these local variables
//********************************
unsigned long timeNowRTC = 0;
unsigned long timeLastRTC = 0;
unsigned long timeNowTempRequest = 0;
unsigned long timeLastTempRequest = 0;
//***************************************


/*******************************************************************
    In the menu system:
        page 1 is the menu items themselves
        page 2 is the values of the parameters associated with the menu items

        page 2 is selected by using the button in a menu item
 ******************************************************************/
int menuitem = 1;
int frame = 1;                        // initialise menu traversal variables
int page = 1;
int lastMenuItem = 1;

String menuItem1 = "MaxShotTime";
String menuItem2 = "Pulse Length";
String menuItem3 = "Pulse Gap";
String menuItem4 = "Num Pulses";
String menuItem5 = "Boiler Temp";
String menuItem6 = "Contrast";
String menuItem7 = "Light: ON";
String menuItem8 = "ResetDefaults";

// default values /////////////
boolean backlight = true;
int contrast = 35;
int maxShotTime = 50;
float boilerTemp = 210.0;
float pulseGap = 2.0;
int numPulses = 3;
float pulseLength = 1.0;
/////////////////////////////
boolean up = false;
boolean down = false;
boolean middle = false;

ClickEncoder *encoder;
int16_t last, value;

Adafruit_PCD8544 display = Adafruit_PCD8544(5, 4, 3); //Download the latest Adafruit Library in order to use this constructor


//************************************************************************************

void setup() {

  Serial.begin(115200);
  Serial.println("TESTING!!!\n\n\n");
  display.begin();
  display.clearDisplay();
  setContrast();
  
  if (! rtc.begin()) {
    Serial.println(F("Couldn't find RTC"));

    // PRINT THIS TO LCD!!!
    // Does not detect missing RTC, maybe due to the other devices on the I2C bus

    while (1);
  }
  if (! rtc.isrunning()) {

    // NEED TO PRINT THIS TO LCD

    Serial.println(F("RTC is NOT running!"));
    Serial.println(F("Starting RTC now..."));
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(5, 0);
    display.print("I2C");
    display.setCursor(5, 25);
    display.print("FAIL");
    display.display();
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  Wire.begin();       // begin I2C operations

  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  pinMode(BACKLIGHT_PIN, OUTPUT);
  turnBacklightOn();

  encoder = new ClickEncoder(A1, A0, A2);
  encoder->setAccelerationEnabled(false);

//  display.begin();
//  display.clearDisplay();
//  setContrast();

  Timer1.initialize(1000);
  Timer1.attachInterrupt(timerIsr);

  last = encoder->getValue();
}
//******************************************************************

//-------------------------------------------------------------------------
double temp;    // used for receiving I2C data
double output;  // from SLAVE ONE

// used for receiving I2C data from SLAVE TWO
double mass;                // 4 bytes
int _brewSwitchState;       // 2 bytes
boolean extractionComplete; // 1 byte
double preinfuseTime;       // 4 bytes
double extractionTime;      // 4 bytes

int _preinfuseTime;
int _extractionTime;
//-------------------------------------------------------------------------

double lastMass;
double lastPreinfuseTime;
double lastExtractionTime;
int _lastBrewSwitchState = 0;
boolean lastExtractionComplete;
/**********************************************************/

//******************************************************************

void loop() {
  timeNowTempRequest = millis();
  if ((timeNowTempRequest - timeLastTempRequest) >= 1000) {
    Serial.print(timeNowTempRequest / 1000); Serial.print("\t");
    requestMassAndSwitchState();
    requestTemp();
    timeLastTempRequest = timeNowTempRequest;
  }
  drawMenu();
  readRotaryEncoder();
  ClickEncoder::Button b = encoder->getButton();
  if (b != ClickEncoder::Open) {
    switch (b) {
      case ClickEncoder::Clicked:
        middle = true;
        break;
    }
  }

  if (up && page == 1 ) {      // if moving up and at top of frame, move up one frame
    up = false;
    if (menuitem == 2 && frame == 2)  {
      frame--;
    }
    if (menuitem == 3 && frame == 3)  {
      frame--;
    }
    if (menuitem == 4 && frame == 4)  {
      frame--;
    }
    if (menuitem == 5 && frame == 5)  {
      frame--;
    }
    if (menuitem == 6 && frame == 6)  {
      frame--;
    }

    lastMenuItem = menuitem;
    menuitem--;
    if (menuitem == 0)        // if moving up and passing top menu item, go back to it
    {
      menuitem = 1;
    }
    
  } else if (up && page == 2 && menuitem == 1) { //We have turned the Rotary Encoder Counter Clockwise
    up = false;                           // while on page 2, so change parameters
    maxShotTime--;                        // decrement menuitem1 parameter - maxShotTime
    if (maxShotTime <= 0) {
      maxShotTime = 0;
    }
  }
  else if (up && page == 2 && menuitem == 2) {
    up = false;
    pulseLength -= 0.5;                        // decrement menuitem2 parameter - pulseLength
    if (pulseLength <= 0) {
      pulseLength = 0;
    }
  }
  else if (up && page == 2 && menuitem == 3 ) {
    up = false;
    pulseGap -= 0.5;                         // decrement menuitem3 parameter - pulseGap
    if (pulseGap <= 0) {
      pulseGap = 0;
    }
  }
  else if (up && page == 2 && menuitem == 4 ) {
    up = false;
    numPulses--;                        // decrement menuitem4 parameter - numPulses
    if (numPulses <= 0) {
      numPulses = 0;
    }
  }
  else if (up && page == 2 && menuitem == 5 ) {
    up = false;
    boilerTemp -= 0.25;                 // decrement menuitem5 parameter - numPulses
    if (boilerTemp <= 50.0) {
      boilerTemp = 50.0;
    }
  }
  else if (up && page == 2 && menuitem == 6 ) {
    up = false;
    contrast--;                         // decrement menuitem6 parameter - screen contrast
    if (contrast <= 0) {
      contrast = 0;
    }
    if (contrast >= 100) {
      contrast = 100;
    }
    setContrast();
  }


                  //*************************************
  if (down && page == 1) //We have turned the Rotary Encoder Clockwise
  {

    down = false;
    if (menuitem == 3 && lastMenuItem == 2) {
      frame ++;
    } 
    else  if (menuitem == 4 && lastMenuItem == 3) {
      frame ++;
    }
    else  if (menuitem == 5 && lastMenuItem == 4) {
      frame ++;
    }
    else if (menuitem == 6 && lastMenuItem == 5 && frame != 5)  {
      frame++;
    }
    else if (menuitem == 7 && lastMenuItem == 6 && frame != 6)  {
      frame++;
    }
    else if (menuitem == 8 && lastMenuItem == 7 && frame != 6)  {
      frame++;
    }

    lastMenuItem = menuitem;
    menuitem++;
    if (menuitem == 9)  {       // we only have 8 menu items, so if we get to 9, go back
      menuitem--;
    }


  } else if (down && page == 2 && menuitem == 1) {    //We have turned the Rotary Encoder Clockwise
    down = false;                                     // while on page 2, so change parameters
    maxShotTime++;
  }
  else if (down && page == 2 && menuitem == 2) {
    down = false;
    pulseLength+=0.5;
  }
  else if (down && page == 2 && menuitem == 3 ) {
    down = false;
    pulseGap += 0.5;
  }
  else if (down && page == 2 && menuitem == 4 ) {
    down = false;
    numPulses++;
  }
  else if (down && page == 2 && menuitem == 5 ) {
    down = false;
    boilerTemp += 0.25;
  }
  else if (down && page == 2 && menuitem == 6 ) {
    down = false;
    contrast++;
    if(contrast <= 0)  {
      contrast = 0;
    }
    else if (contrast >= 100) {
      contrast = 100;
    }
    setContrast();
  }

                //*******************************

  if (middle) //Middle Button is Pressed
  {
    middle = false;
    /*************************************
     * 
     * Backlight control & Reset Defaults dont have any parameters,
     * so just run their methods
     * 
     *************************************/

    
    if (page == 1 && menuitem == 7) // Backlight Control
    {
      if (backlight)
      {
        backlight = false;
        menuItem7 = "Light: OFF";
        turnBacklightOff();
      }
      else
      {
        backlight = true;
        menuItem6 = "Contrast";    //"Light: ON";
        menuItem7 = "Light: ON";
        turnBacklightOn();
      }
    }

    if (page == 1 && menuitem == 8) // Reset
    {
      resetDefaults();
    }

        // all the other menu items have parameters so goto page 2 to change them
    else if (page == 1 && menuitem <= 6) {
      page = 2;
    }

        // if button pressed while on page 2, go back to page 1
    else if (page == 2)
    {
      page = 1;
    }
  }
}

//*******************************************************************

void drawMenu()
{

  if (page == 1)
  {
    display.setTextSize(1);
    display.clearDisplay();
    display.setTextColor(BLACK, WHITE);
    display.setCursor(0, 0);

    ds1307RTC();

    display.drawFastHLine(0, 10, 83, BLACK);

    //*****************************************
    if (menuitem == 1 && frame == 1)        // frame 1 contains items 1,2,3
    {
      displayMenuItem(menuItem1, 15, true);
      displayMenuItem(menuItem2, 25, false);    
      displayMenuItem(menuItem3, 35, false);
    }
    else if (menuitem == 2 && frame == 1)
    {
      displayMenuItem(menuItem1, 15, false);
      displayMenuItem(menuItem2, 25, true);     
      displayMenuItem(menuItem3, 35, false);
    }
    else if (menuitem == 3 && frame == 1)
    {
      displayMenuItem(menuItem1, 15, false);
      displayMenuItem(menuItem2, 25, false);
      displayMenuItem(menuItem3, 35, true);
    }
    //*******************************************
    else if (menuitem == 2 && frame == 2)       // frame 2 contains items 2,3,4
    {
      displayMenuItem(menuItem2, 15, true);
      displayMenuItem(menuItem3, 25, false);
      displayMenuItem(menuItem4, 35, false);
    }
    else if (menuitem == 3 && frame == 2)
    {
      displayMenuItem(menuItem2, 15, false);
      displayMenuItem(menuItem3, 25, true);
      displayMenuItem(menuItem4, 35, false);
    }
    else if (menuitem == 4 && frame == 2)
    {
      displayMenuItem(menuItem2, 15, false);
      displayMenuItem(menuItem3, 25, false);
      displayMenuItem(menuItem4, 35, true);
    }
    //**********************************************
    else if (menuitem == 3 && frame == 3)       // frame 3 contains items 3,4,5
    {
      displayMenuItem(menuItem3, 15, true);
      displayMenuItem(menuItem4, 25, false);
      displayMenuItem(menuItem5, 35, false);
    }
    else if (menuitem == 4 && frame == 3)
    {
      displayMenuItem(menuItem3, 15, false);
      displayMenuItem(menuItem4, 25, true);
      displayMenuItem(menuItem5, 35, false);
    }

    else if (menuitem == 5 && frame == 3)
    {
      displayMenuItem(menuItem3, 15, false);
      displayMenuItem(menuItem4, 25, false);
      displayMenuItem(menuItem5, 35, true);
    }
    //************************************************
    else if (menuitem == 4 && frame == 4)         // frame 4 contains items 4,5,6
    {
      displayMenuItem(menuItem4, 15, true);
      displayMenuItem(menuItem5, 25, false);
      displayMenuItem(menuItem6, 35, false);
    }
    else if (menuitem == 5 && frame == 4)
    {
      displayMenuItem(menuItem4, 15, false);
      displayMenuItem(menuItem5, 25, true);
      displayMenuItem(menuItem6, 35, false);
    }

    else if (menuitem == 6 && frame == 4)
    {
      displayMenuItem(menuItem4, 15, false);
      displayMenuItem(menuItem5, 25, false);
      displayMenuItem(menuItem6, 35, true);
    }
    //********************************************
    else if (menuitem == 5 && frame == 5)         // frame 5 contains items 5,6,7
    {
      displayMenuItem(menuItem5, 15, true);
      displayMenuItem(menuItem6, 25, false);
      displayMenuItem(menuItem7, 35, false);
    }
    else if (menuitem == 6 && frame == 5)
    {
      displayMenuItem(menuItem5, 15, false);
      displayMenuItem(menuItem6, 25, true);
      displayMenuItem(menuItem7, 35, false);
    }

    else if (menuitem == 7 && frame == 5)
    {
      displayMenuItem(menuItem5, 15, false);
      displayMenuItem(menuItem6, 25, false);
      displayMenuItem(menuItem7, 35, true);
    }
    //********************************************
    else if (menuitem == 6 && frame == 6)         // frame 6 contains items 6,7,8
    {
      displayMenuItem(menuItem6, 15, true);
      displayMenuItem(menuItem7, 25, false);
      displayMenuItem(menuItem8, 35, false);
    }
    else if (menuitem == 7 && frame == 6)
    {
      displayMenuItem(menuItem6, 15, false);
      displayMenuItem(menuItem7, 25, true);
      displayMenuItem(menuItem8, 35, false);
    }
    else if (menuitem == 8 && frame == 6)
    {
      displayMenuItem(menuItem6, 15, false);
      displayMenuItem(menuItem7, 25, false);
      displayMenuItem(menuItem8, 35, true);
    }

    display.display();    // write menu to display
  }
  else if (page == 2 && menuitem == 1){
    displayIntMenuPage(menuItem1, maxShotTime);
  }
  else if (page == 2 && menuitem == 2)  {
    displayFloatMenuPage(menuItem2, pulseLength);
  }
  else if (page == 2 && menuitem == 3)  {
    displayFloatMenuPage(menuItem3, pulseGap);
  }
  else if (page == 2 && menuitem == 4)  {
    displayIntMenuPage(menuItem4, numPulses);
  }
  else if (page == 2 && menuitem == 5)  {
    displayFloatMenuPage(menuItem5, boilerTemp);
  }
  else if (page == 2 && menuitem == 6)  {
    displayIntMenuPage(menuItem6, contrast);
    Serial.print("Contrast:\t");  Serial.println(contrast);
  }



}
/************************************************************/
void resetDefaults()
{
  boilerTemp = 210.0;
  contrast = 35;
  maxShotTime = 45;
  pulseLength = 1;
  pulseGap = 2;
  numPulses = 3;
  setContrast();
  backlight = true;
  menuItem7 = "Light: ON";
  turnBacklightOn();

}
/*****************************************************************/
void setContrast()  {
  display.setContrast(contrast);
  display.display();
}
/*****************************************************************/
void turnBacklightOn()  {
  digitalWrite(BACKLIGHT_PIN, LOW);
}
/*****************************************************************/
void turnBacklightOff() {
  digitalWrite(BACKLIGHT_PIN, HIGH);
}
/*****************************************************************/
void timerIsr() {
  encoder->service();
}
/*****************************************************************/
void displayIntMenuPage(String menuItem, int value) {
  display.setTextSize(1);
  display.clearDisplay();
  display.setTextColor(BLACK, WHITE);
  display.setCursor(7, 0);
  display.print(menuItem);
  display.drawFastHLine(0, 10, 83, BLACK);
  display.setCursor(5, 15);

  if (menuItem == menuItem4) {            // if numPulses print "Pulses"
    display.print("Pulses");
  }
  else if (menuItem == menuItem5) {       // if brewTemp use degrees
    display.print("Fahrenheit");
  }
  else if (menuItem == menuItem6) {        // if contrast print "%"
    display.print("%");
  }
  else display.print("Seconds");            // others all use seconds

  display.setTextSize(2);
  display.setCursor(5, 25);                 // write to display
  display.print(value);
  display.setTextSize(2);
  display.display();
}
/*****************************************************************/
void displayFloatMenuPage(String menuItem, float value) {
  display.setTextSize(1);
  display.clearDisplay();
  display.setTextColor(BLACK, WHITE);
  display.setCursor(7, 0);
  display.print(menuItem);
  display.drawFastHLine(0, 10, 83, BLACK);
  display.setCursor(5, 15);

  if (menuItem == menuItem4) {              // if numPulses print "Pulses"
    display.print("Pulses");
  } 
  else if (menuItem == menuItem5) {         // if brewTemp use degrees
    display.print("Fahrenheit");
  }
  else if (menuItem == menuItem6) {         // if contrast print "%"
    display.print("%");
  }
  else display.print("Seconds");            // others all use seconds

  display.setTextSize(2);
  display.setCursor(5, 25);                 // display value and its units
  display.print(value);
  display.setTextSize(2);
  display.display();
}
/*****************************************************************/

void displayStringMenuPage(String menuItem, String value)
{
  display.setTextSize(1);
  display.clearDisplay();
  display.setTextColor(BLACK, WHITE);
  display.setCursor(7, 0);
  display.print(menuItem);
  display.drawFastHLine(0, 10, 83, BLACK);
  display.setCursor(5, 15);
  display.print("Value");
  display.setTextSize(2);
  display.setCursor(5, 25);
  display.print(value);
  display.setTextSize(2);
  display.display();
}
/*****************************************************************/

void displayMenuItem(String item, int position, boolean selected)
{
  if (selected)
  {
    display.setTextColor(WHITE, BLACK);
  } else
  {
    display.setTextColor(BLACK, WHITE);
  }
  display.setCursor(0, position);
  display.print(">" + item);
}
/*****************************************************************/

void readRotaryEncoder()
{
  value += encoder->getValue();

  if (value / 2 > last) {
    last = value / 2;
    down = true;
    Serial.println(F("DOWN"));
    delay(150);
  } else   if (value / 2 < last) {
    last = value / 2;
    up = true;
    Serial.println(F("UP"));
    delay(150);
  }
}
/*****************************************************************/

void ds1307RTC() {
  timeNowRTC = millis();
  if ((timeNowRTC - timeLastRTC) >= 500) {      // check RTC 5 times per second
    DateTime now = rtc.now();
    String today = "";
    int dayOfWeek = now.dayOfTheWeek();
    switch (dayOfWeek) {
      case 1:
        today = "Mon";
        break;
      case 2:
        today = "Tue";
        break;
      case 3:
        today = "Wed";
        break;
      case 4:
        today = "Thu";
        break;
      case 5:
        today = "Fri";
        break;
      case 6:
        today = "Sat";
        break;
      case 7:
        today = "Sun";
        break;
      default:
        today = "ERR";
        break;

    }

    //display.print(now.month());display.print("/"); display.print(now.day()); display.print(" ");
    display.print(now.day());  display.print("  ");
    display.print(today); display.print("  ");
    display.print(now.hour()); display.print(":");

    int mins = now.minute();

    if (mins <= 9) {                                // if mins is less than 10, add a leading 0
      display.print("0"); display.print(mins);
    }
    else display.print(mins);
  }
}

/*****************************************************************/

void requestTemp() {
  //Serial.print(F("\n\nMillis start: "));
  //Serial.println(millis());
  int n = Wire.requestFrom(SLAVE_ONE_ADDRESS, 8);  // request 8 bytes (2 doubles) from slave one
  //Serial.print(F("n = \t")); Serial.print(n); Serial.println(F("\n"));
  if (n == 8) {
    I2C_readAnything(temp);
    I2C_readAnything(output);

    //Serial.print(F("Temp : "));
    Serial.print(temp);
    //Serial.print(F("Output : "));
    Serial.print("\t");
    Serial.println( output);
    //Serial.print(F("\n\nMillis End: "));              //   2 milliseconds to run this method
    //Serial.println(millis());
  }
  else Serial.println(F("FAIL TEMP"));
}
/*****************************************************************/
/*
void sendTempPreinfuseData(float temp, float pulseLength, float pulseGap,int numPulses){
  Wire.beginTransmission(SLAVE_ONE_ADDRESS);
  Wire.write(temp);
  Wire.write(pulseLength);
  Wire.write(pulseGap);
  Wire.write(numPulses);
  Serial.print("Sending: "); Serial.print(temp);  Serial.println("°F");
  Wire.endTransmission();
}
*/
/*****************************************************************/

void requestMassAndSwitchState() {
  int n = Wire.requestFrom(SLAVE_TWO_ADDRESS, 15);  // request 15 bytes (1 double, 1 int, 1 boolean, 1 double, 1 double) from SLAVE TWO
  if (n == 15) {
    I2C_readAnything(_brewSwitchState);
    I2C_readAnything(mass);
    I2C_readAnything(extractionComplete);
    I2C_readAnything(preinfuseTime);
    I2C_readAnything(extractionTime);

    _preinfuseTime = (int) (preinfuseTime / 1000.0);
    _extractionTime = (int) (extractionTime / 1000.0);


    Serial.print(F("Switch state : "));
    Serial.println(_brewSwitchState);
    if(_brewSwitchState == 1){
      // tell PID output = 5000
      //Wire.write
    }
    Serial.print(F("Mass : "));
    Serial.println(mass);
    Serial.print(F("Extraction complete?  "));
    Serial.println(extractionComplete);
    Serial.print(F("Preinfuse Time : "));
    Serial.println(_preinfuseTime);
    Serial.print(F("Extraction Time : "));
    Serial.println(_extractionTime);
    Serial.println(F("\n"));



  }
  else {
    Serial.println(F("FAIL MASS"));
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(5, 0);
    display.print("I2C");
    display.setCursor(5, 25);
    display.print("FAIL");
    display.display();
  }
}

/*****************************************************************/

// printFloat prints out the float 'value' rounded to 'places' places after the decimal point
void printFloat(float value, int places) {
  // this is used to cast digits
  int digit;
  float tens = 0.1;
  int tenscount = 0;
  int i;
  float tempfloat = value;

  // make sure we round properly. this could use pow from <math.h>, but doesn't seem worth the import
  // if this rounding step isn't here, the value  54.321 prints as 54.3209

  // calculate rounding term d:   0.5/pow(10,places)
  float d = 0.5;
  if (value < 0)
    d *= -1.0;
  // divide by ten for each decimal place
  for (i = 0; i < places; i++)
    d /= 10.0;
  // this small addition, combined with truncation will round our values properly
  tempfloat +=  d;

  // first get value tens to be the large power of ten less than value
  // tenscount isn't necessary but it would be useful if you wanted to know after this how many chars the number will take

  if (value < 0)
    tempfloat *= -1.0;
  while ((tens * 10.0) <= tempfloat) {
    tens *= 10.0;
    tenscount += 1;
  }


  // write out the negative if needed
  if (value < 0)
    Serial.print('-');

  if (tenscount == 0)
    Serial.print(0, DEC);

  for (i = 0; i < tenscount; i++) {
    digit = (int) (tempfloat / tens);
    Serial.print(digit, DEC);
    tempfloat = tempfloat - ((float)digit * tens);
    tens /= 10.0;
  }

  // if no places after decimal, stop now and return
  if (places <= 0)
    return;

  // otherwise, write the point and continue on
  Serial.print('.');

  // now write out each decimal place by shifting digits one by one into the ones place and writing the truncated value
  for (i = 0; i < places; i++) {
    tempfloat *= 10.0;
    digit = (int) tempfloat;
    Serial.print(digit, DEC);
    // once written, subtract off that digit
    tempfloat = tempfloat - (float) digit;
  }
}

/*****************************************************************/

void beep(unsigned char delayms) {
  analogWrite(9, 20);      // Almost any value can be used except 0 and 255
  // experiment to get the best tone
  delay(delayms);          // wait for a delayms ms
  analogWrite(9, 0);       // 0 turns it off
  delay(delayms);          // wait for a delayms ms
}



