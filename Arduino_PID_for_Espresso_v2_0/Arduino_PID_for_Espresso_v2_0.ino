/*
********************************************************
  Arduino PID for Espresso V2.0 by Ross Satchell 2013  *
********************************************************
  This code is intended for single boiler espresso machines such as the Rancilio Silvia (I have been using one as a test machine)
  However it can be easily adapted for double boiler machines by setting up separate PID calls for each boiler. Of course seperate thermocouples and MAX31855's would be required
  It can be adapted for pretty much any application where a PID controller and a menu system is needed
  
 The code for this has been borrowed from several sources and modified to suit this purpose.
 It is a work in progress and will continue to be updated as time permits around my studies.
 
 I would like to thank Brett Beauregard http://brettbeauregard.com/blog for writing the PID library and answering my questions, 
 Konstantine Polanski for combining Brett's Autotune front end and backend for easy acquisition of initial tuning parameters   https://groups.google.com/forum/?fromgroups#!topic/diy-pid-control/4bm_SQrPe8c
 Adafruit for writing the MAX31855 library https://github.com/adafruit/Adafruit-MAX31855-library
 Alexander Brevig http://alexanderbrevig.github.io/ for writing the original menubackend library, 
 Giuseppe Di Cillo http://www.coagula.org/ for modifying the menubackend library to return to root,
 the Arduino community for providing the LCD library http://arduino.cc/en/Reference/LiquidCrystal?from=Tutorial.LCDLibrary  and for helping me solve problems,
 and my friend Daniel Croft for lending me his Rancilio Silvia, helping me along the way and committing code to Github for me!
 
 For those not very familiar with PID, I highly recommend Brett's Beginners posts http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
  
 This code has been tested in Arduino IDE V1.0 
 For modification and debugging, uncomment the Serial.print lines to print to the Serial Window to make changes easier to detect
 
 The LCD (HD44780 compatible) is connected in 4-bit mode with connections as follows:-
 * LCD Enable pin to digital pin 11
 * LCD D4 pin to digital pin 5
 * LCD D5 pin to digital pin 4
 * LCD D6 pin to digital pin 3
 * LCD D7 pin to digital pin 2
 * LCD R/W pin to ground
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)
 
 To get initial tuning parameter via the Autotune function, join the diy-pid-control google group and see Konstantine's post (linked above, dated Feb 28 2013) for instructions on how to use the Autotune
 frontend and backend.  I plan to integrate my code into the Autotune backend as time allows to simplify things. Currently, I use the autotune to get my parameters, then load them into this code at Lines 118-120.
 */


#include <LiquidCrystal.h>
#include <Adafruit_MAX31855.h>
#include <MenuBackend.h>
#include <Wire.h>
#include <PID_v1.h>
#define RelayPin 7    // SSR connected to Arduino digital 7

int thermoDO = 3;    // MAX31855 Data to Arduino digital 3
int thermoCS = 4;    // MAX31855 Chip Select to Arduino digital 4
int thermoCLK = 5;    // MAX31855 Clock to Arduino digital 5

int right;    // boolean for right button
int left;      // boolean for left button
int enter;    // boolean for enter button
int escape;    // boolean for esc button
int setBrewTemp; //boolean variabke for storing whether user is in Brew Set Point part of menu 
int setSteamTemp;  //boolean variabke for storing whether user is in Steam Set Point part of menu 
int setOffsetBool; // boolean for storing whether user is in Set Offset part of menu
int setTempScale; // boolean for storing whether user is in Set TempScale part of menu
int setBrewing;  // boolean for storing whether user is in Brew Espresso part of menu
int setSteaming;  // boolena for storing whewther user is in Steam Milk part of menu

double brewTempSetting = 200.00; //variable for allowing setting of brew setpoint when in Set Brew Temp part of menu
double steamTempSetting = 250.00; //variable for allowing setting of steam setpoint when in Set Brew Temp part of menu
int intBrewTempSetting;  // int for displaying brewTempSetting in integer for to save lcd space (MAY NOT BE USED)
int intSteamTempSetting;  // int for displaying steamTempSetting in integer for to save lcd space (MAY NOT BE USED)
double offsetSetting;  // value for changing/displaying temp offset from boiler to grouphead 

const int buttonPinLeft = 13; // pin for the Up button
const int buttonPinRight = 10; // pin for the Down button
const int buttonPinEsc = 12; // pin for the Esc button
const int buttonPinEnter = 11; // pin for the Enter button

int lastButtonPushed = 0;    // initialise 
int lastButtonEnterState = LOW; // the previous reading from the Enter input pin
int lastButtonEscState = LOW; // the previous reading from the Esc input pin
int lastButtonLeftState = LOW; // the previous reading from the Left input pin
int lastButtonRightState = LOW; // the previous reading from the Right input pin

long lastEnterDebounceTime = 0; // the last time the output pin was toggled
long lastEscDebounceTime = 0; // the last time the output pin was toggled
long lastLeftDebounceTime = 0; // the last time the output pin was toggled
long lastRightDebounceTime = 0; // the last time the output pin was toggled
long debounceDelay = 300; // the debounce time

//Menu variables
MenuBackend menu = MenuBackend(menuUsed,menuChanged);
//initialize menu items
MenuItem menu1Item1 = MenuItem("Brew Espresso");  // select this to load preset brewing setpoint
MenuItem menu1Item2 = MenuItem("Steam Milk");    // select this to load preset steam stepoint
MenuItem menu1Item3 = MenuItem("Brew Temp");    // select this and move to child to change brew setpoint
MenuItem menuItem3SubItem1 = MenuItem("Set Brew Point");  // select this to change brew setpoint
MenuItem menu1Item4 = MenuItem("Steam Temp");            // select this and move to child to change steam setpoint
MenuItem menuItem4SubItem1 = MenuItem("Set Steam Point");  // select this to change steam setpoint
MenuItem menu1Item5 = MenuItem("Offset");                  // select this & move to child to change boiler to group temp offset
MenuItem menuItem5SubItem1 = MenuItem("Set Offset");        // select this to change boiler to group temp offset
MenuItem menu1Item6 = MenuItem("Scale");                    // select this & move to child to change temp scale
MenuItem menuItem6SubItem1 = MenuItem("Set Scale");         // select this to change temp scale

long unsigned lastMillis;
unsigned long nowMillis;
double minutes;

Adafruit_MAX31855 thermocouple(thermoCLK, thermoCS, thermoDO);    // initialise MAX31855 chip

const int numReadings = 10;      // number of readings in array. Increase if slower more stable readings are reqd, reduce if faster readings are reqd

double readings[numReadings];      // the readings from thermocouple
int index = 0;                  // the index of the current reading
double total = 0.0;                  // the running total
double averageF = 0.0;                // the average      
double tempReadF = 0.0;              // temp read in F
double tempReadC = 0.0;              // temp read in C

LiquidCrystal lcd(14, 15, 16, 17, 18, 19);  // initialize the library with the numbers of the interface pins


double Setpoint, Input, Output;    //Define PID variables we'll be connecting to

float Kp = 16.16;    //User inserts PID values obtaines from Autotune here
float Ki = 0.14;    //
float Kd = 480.10;  //

PID myPID(&Input, &Output, &Setpoint,Kp, Ki, Kd, DIRECT);  //Specify the links and initial tuning parameters


int WindowSize = 1000;    // I have found for espresso machines that a 1000ms window does well
unsigned long windowStartTime;  // used in calculating PID output pulse length
double timeNowMins;                // used for logging time in Serial Window


void setup() {
   
  Serial.begin(9600);
  pinMode(buttonPinLeft, INPUT);    //setup input buttons
  pinMode(buttonPinRight, INPUT);
  pinMode(buttonPinEnter, INPUT);
  pinMode(buttonPinEsc, INPUT);
  
  for (int thisReading = 0; thisReading < numReadings; thisReading++)      // initialize all the readings to 0: 
    readings[thisReading] = 0;    


  lcd.begin(16, 2);        // set up the LCD's number of rows and columns: 
  lcd.clear();              // clear LCD
  lcd.setCursor(0,0);        // set cursor to column 1, row 1
  lcd.print("Arduino PID");    //SPLASH
  lcd.setCursor(4,1);          //SCREEN
  lcd.print("for Espresso");
  delay(3000);          // delay for dramatic effect!
  lcd.clear();          // clear lcd
  
  windowStartTime = millis();    // set window to current millis value
  Setpoint = 200;  //Default Setpoint for Rancilio Silvia for approx 200F at the group head
  offsetSetting = 12.00;
  myPID.SetOutputLimits(0, WindowSize);   //tell the PID to range between 0 and the full window size
  myPID.SetMode(AUTOMATIC);    //turn the PID on
  
  //configure menu structure
menu.getRoot().add(menu1Item1);
menu1Item1.addRight(menu1Item2).addRight(menu1Item3).addRight(menu1Item4).addRight(menu1Item5).addRight(menu1Item6);
menu1Item3.add(menuItem3SubItem1);
menu1Item4.add(menuItem4SubItem1);
menu1Item5.add(menuItem5SubItem1); 
menu1Item6.add(menuItem6SubItem1);

menu.toRoot();    // go to root of menu
  
}

void loop() {
  tempRead();    // call temp reading method
  
  if ((millis() - lastMillis) >= 125)  {    //perform calcPID 8 times per second
  calcPID();    // call PID calculation
  readButtons(); //  button reading and navigation split in two procedures because 
  navigateMenus(); //in some situations, the buttons are used for other purpose (eg. to change some settings)
  changeValues(); // scans buttons and menu states
  }
 
}  // end loop

void tempRead()  {
   double c = thermocouple.readCelsius();
   if (isnan(c)) {    //  if c is not a number
     //Serial.println("Something wrong with thermocouple!"); // uncomment if using serial window
     lcd.setCursor(0,0);
     lcd.print("Thermocouple Err");
   } else {
    // Serial.print("C = "); 
    // Serial.println(c);
   }

   tempReadF = thermocouple.readFarenheit();  // read temp in degrees F
   tempReadC = c;                            // temp in degrees C
    
   total= total - readings[index];         
   readings[index] = tempReadF;         // read from the sensor: 
   total= total + readings[index];     // add the reading to the total:    
   index = index + 1;                 // advance to the next position in the array:         
   if (index >= numReadings)          // if we're at the end of the array...
   index = 0;                         // ...wrap around to the beginning:                          
   averageF = total / numReadings;    // calculate the average:
    
   //Serial.print("Internal Temp = ");        // uncomment these lines if checking MAX31855 
   //Serial.println(thermocouple.readInternal());     // internal chip temp via Serial Window
   //Serial.print("Temp ");                    // uncomment these lines if checking MAX31855
   Serial.print(averageF);                  // hot junction temp via Serial Window
   Serial.print("\t");
   lcd.setCursor(9,0);
   lcd.print(averageF);      // print temp to upper RHS of LCD
   lcd.print((char)223);          // print degree symbol
   
   
}
   
void calcPID()  {
  Input = averageF; // PID input is measured averaged temp
  myPID.Compute(); //PID calculation
  
  /************************************************
  * turn the output pin on/off based on pid output
  ************************************************/
  unsigned long now = millis();
  if(now - windowStartTime>WindowSize)    //time to shift the Relay Window
    { 
    windowStartTime += WindowSize;
    }
  if(Output > now - windowStartTime) digitalWrite(RelayPin,HIGH);
  else digitalWrite(RelayPin,LOW);
  
  Serial.print(Output);    // uncomment these lines if
  Serial.print("\t");      // logging data to  
  timeNowMins = millis() / 60000.0;    //Serial 
  Serial.println(timeNowMins);        // Window
}

void menuChanged(MenuChangeEvent changed){

MenuItem newMenuItem=changed.to; //get the destination menu

lcd.setCursor(0,1); //set the start position for lcd printing to the second row, first column

if(newMenuItem.getName()==menu.getRoot()){
  lcd.setCursor(0,0);
  lcd.print("       ");
  lcd.setCursor(0,1);
  lcd.print("Menu            ");
  setBrewTemp = false; 
  setSteamTemp = false;
  setOffsetBool = false;
  setTempScale = false;

}else if(newMenuItem.getName()=="Brew Espresso"){
    lcd.setCursor(0,0);
    lcd.print("       ");
    lcd.setCursor(0,1);
    lcd.print("Brew Espresso   ");
    setBrewTemp = false;
    setSteamTemp = false;
    setBrewing = true;
    setSteaming = false;
    setOffsetBool = false;
    setTempScale = false;

}else if(newMenuItem.getName()=="Steam Milk")  {
    lcd.setCursor(0,0);
    lcd.print("       ");
    lcd.setCursor(0,1);
    lcd.print("Steam Milk      ");
    setBrewTemp = false;
    setSteamTemp = false;
    setOffsetBool = false;
    setTempScale = false;
    setSteaming = true;
    setBrewing = false;

}else if(newMenuItem.getName()=="Brew Temp"){
    lcd.setCursor(0,0);
    lcd.print("       ");
    lcd.setCursor(0,0);
    lcd.print(brewTempSetting);
    lcd.setCursor(0,1);
    lcd.print("Brew Temp       "); 
    setBrewTemp = false;
    setSteamTemp = false;
    setOffsetBool = false;
     setTempScale = false;


}else if (newMenuItem.getName()=="Set Brew Point"){
    lcd.setCursor(0,0);
    lcd.print("       ");
    lcd.setCursor(0,1);  
    lcd.print("Set Brew Point  ");
    lcd.setCursor(0,0);
    lcd.print(brewTempSetting);
    setBrewTemp = true;
    setSteamTemp = false;
    setOffsetBool = false;
    setTempScale = false;
    
}else if (newMenuItem.getName()=="Steam Temp")  {
    lcd.setCursor(0,0);
    lcd.print("       ");
    lcd.setCursor(0,1);
    lcd.print("Steam Temp      ");
    lcd.setCursor(0,0);
    lcd.print(steamTempSetting);
    setBrewTemp = false;
    setSteamTemp = false;
    setOffsetBool = false;
    setTempScale = false;
  
}else if (newMenuItem.getName()=="Set Steam Point")  {
    lcd.setCursor(0,0);
    lcd.print("       ");
    lcd.setCursor(0,0);
    lcd.print(steamTempSetting);
    lcd.setCursor(0,1);
    lcd.print("Set Steam Point ");
    setBrewTemp = false;
    setSteamTemp = true;
    setOffsetBool = false;
    setTempScale = false;


}else if(newMenuItem.getName()=="Offset")  {
    lcd.setCursor(0,0);
    lcd.print("       ");
    lcd.setCursor(0,0);
    lcd.print(offsetSetting);
    lcd.setCursor(0,1);
    lcd.print("Offset          ");
    setBrewTemp =false;
    setSteamTemp = false;
    setOffsetBool = false;
    setTempScale = false;
  
}else if(newMenuItem.getName()=="Set Offset")  {
    lcd.setCursor(0,0);
    lcd.print("       ");
    lcd.setCursor(0,1);
    lcd.print("Set Offset");
    lcd.setCursor(0,0);
    lcd.print(offsetSetting);
    setBrewTemp =false;
    setSteamTemp = false;
    setOffsetBool = true;
    setTempScale = false;
    
}else if(newMenuItem.getName()=="Scale")  {   
    lcd.setCursor(0,0);
    lcd.print("       ");
    lcd.setCursor(0,1);
    lcd.print("Scale  "); 
    setBrewTemp =false;
    setSteamTemp = false;
    setOffsetBool = false;
    setTempScale = false;
    
}else if(newMenuItem.getName()=="Set Scale")  {   
    lcd.setCursor(0,0);
    lcd.print("       ");
    lcd.setCursor(0,1);
    lcd.print("Set Scale");
    setBrewTemp =false;
    setSteamTemp = false;
    setOffsetBool = false;
    setTempScale = true;
    
}else  {
    setBrewTemp =false;
    setSteamTemp = false;
    setTempScale = false;
    }
  }
  
void menuUsed(MenuUseEvent used) {
  
  if ((setBrewTemp == true) && (enter == true))  {
     //Serial.print("brew temp selected = ");  // uncomment for Serial 
     //Serial.println(brewTempSetting);        // Window debugging
     Setpoint = brewTempSetting + offsetSetting;
     menu.toRoot();
     
  }else if ((setSteamTemp == true) && (enter == true))  {
     //Serial.print("Steam temp Selected = ");
     //Serial.println(steamTempSetting);
     Setpoint = steamTempSetting;
     menu.toRoot();
     
    
  }else if ((setOffsetBool == true) && (enter == true))  {
    //Serial.print("Offset now set to ");
    //Serial.println(offsetSetting);
    Setpoint = brewTempSetting + offsetSetting;
    menu.toRoot();
    
  
  }else if ((setBrewing == true) && (enter == true))  {
    Setpoint = brewTempSetting + offsetSetting;
    lcd.setCursor(0,0);
    lcd.print("       ");
    lcd.setCursor(0,0);
    lcd.print(Setpoint);
    //Serial.print("ESPRESSO ");
    //Serial.println(Setpoint);
    menu.toRoot();
    
  }else if ((setSteaming == true) && (enter == true))  {
    Setpoint = steamTempSetting;
    lcd.setCursor(0,0);
    lcd.print("       ");
    lcd.setCursor(0,0);
    lcd.print(Setpoint);
    //Serial.print("STEAMING ");
    //Serial.println(Setpoint);
    menu.toRoot();
    
  
  
  }else{
    
     menu.toRoot();
     Setpoint = brewTempSetting + offsetSetting;
  }
}

void readButtons(){ //read buttons status
int reading;
int buttonEnterState=LOW; // the current reading from the Enter input pin
int buttonEscState=LOW; // the current reading from the input pin
int buttonLeftState=LOW; // the current reading from the input pin
int buttonRightState=LOW; // the current reading from the input pin

//Enter button
// read the state of the switch into a local variable:
reading = digitalRead(buttonPinEnter);

// check to see if you just pressed the enter button 
// (i.e. the input went from LOW to HIGH), and you've waited 
// long enough since the last press to ignore any noise: 

// If the switch changed, due to noise or pressing:
if (reading != lastButtonEnterState) {
// reset the debouncing timer
lastEnterDebounceTime = millis();
} 

if ((millis() - lastEnterDebounceTime) > debounceDelay) {
// whatever the reading is at, it's been there for longer
// than the debounce delay, so take it as the actual current state:
buttonEnterState=reading;
lastEnterDebounceTime=millis();
}

// save the reading. Next time through the loop,
// it'll be the lastButtonState:
lastButtonEnterState = reading;


//Esc button 
// read the state of the switch into a local variable:
reading = digitalRead(buttonPinEsc);

// check to see if you just pressed the Down button 
// (i.e. the input went from LOW to HIGH), and you've waited 
// long enough since the last press to ignore any noise: 

// If the switch changed, due to noise or pressing:
if (reading != lastButtonEscState) {
// reset the debouncing timer
lastEscDebounceTime = millis();
} 

if ((millis() - lastEscDebounceTime) > debounceDelay) {
// whatever the reading is at, it's been there for longer
// than the debounce delay, so take it as the actual current state:
buttonEscState = reading;
lastEscDebounceTime=millis();
}

// save the reading. Next time through the loop,
// it'll be the lastButtonState:
lastButtonEscState = reading; 


//Down button 
// read the state of the switch into a local variable:
reading = digitalRead(buttonPinRight);

// check to see if you just pressed the Down button 
// (i.e. the input went from LOW to HIGH), and you've waited 
// long enough since the last press to ignore any noise: 

// If the switch changed, due to noise or pressing:
if (reading != lastButtonRightState) {
// reset the debouncing timer
lastRightDebounceTime = millis();
} 

if ((millis() - lastRightDebounceTime) > debounceDelay) {
// whatever the reading is at, it's been there for longer
// than the debounce delay, so take it as the actual current state:
buttonRightState = reading;
lastRightDebounceTime =millis();
}

// save the reading. Next time through the loop,
// it'll be the lastButtonState:
lastButtonRightState = reading; 


//Up button 
// read the state of the switch into a local variable:
reading = digitalRead(buttonPinLeft);

// check to see if you just pressed the Down button 
// (i.e. the input went from LOW to HIGH), and you've waited 
// long enough since the last press to ignore any noise: 

// If the switch changed, due to noise or pressing:
if (reading != lastButtonLeftState) {
// reset the debouncing timer
lastLeftDebounceTime = millis();
} 

if ((millis() - lastLeftDebounceTime) > debounceDelay) {
// whatever the reading is at, it's been there for longer
// than the debounce delay, so take it as the actual current state:
buttonLeftState = reading;
lastLeftDebounceTime=millis();;
}

// save the reading. Next time through the loop,
// it'll be the lastButtonState:
lastButtonLeftState = reading; 

//records which button has been pressed
if (buttonEnterState==HIGH){
lastButtonPushed=buttonPinEnter;
enter = true;
right = false;
left = false;

}else if(buttonEscState==HIGH){
lastButtonPushed=buttonPinEsc;
//escape = true;
enter = false;
right = false;
left = false;

}else if(buttonRightState==HIGH){
lastButtonPushed=buttonPinRight;
right = true;
left = false;
enter = false;

}else if(buttonLeftState==HIGH){
lastButtonPushed=buttonPinLeft;
left = true;
right = false;
enter = false;

}else{
lastButtonPushed=0;
enter = false;
right = false;
left = false;
} 
}

void navigateMenus() {
MenuItem currentMenu=menu.getCurrent();

switch (lastButtonPushed){
case buttonPinEnter:
if(!(currentMenu.moveDown())){ //if the current menu has a child and has been pressed enter then menu navigate to item below
menu.use();
}else{ //otherwise, if menu has no child and has been pressed enter the current menu is used
menu.moveDown();
} 
break;
case buttonPinEsc:
menu.toRoot(); //back to main
break;
case buttonPinRight:
menu.moveRight();
break; 
case buttonPinLeft:
menu.moveLeft();
break; 
}

lastButtonPushed=0; //reset the lastButtonPushed variable
}

/*
void printingSV() { //for printing the SetValue as the menu is being traversed
  lcd.clear();
  lcd.print("S:"); 
  //intSetpoint = Setpoint * 1;
  lcd.print( Setpoint );
  lcd.print((char)223);
}
*/

void changeValues()  {      // this function allows for the changing of values such as Brew Temp, Steam Temp,
   if ((setBrewTemp == true)  && (right == true))  {    // boiler to group offset temp, etc
    //Serial.print("Incrementing setBrewTemp");
    brewTempSetting = brewTempSetting + 1;
    lcd.setCursor(0,0);
    lcd.print("      ");
    lcd.setCursor(0,0);
    lcd.print(brewTempSetting);
    //Serial.print("brewTempSetting = ");
    //Serial.println(brewTempSetting); 
             
   }else if ((setBrewTemp == true)  && (left == true))  {
      //Serial.print("Decrementing SetBrewTemp");
      brewTempSetting = brewTempSetting - 1;
      //Serial.println(brewTempSetting);
      lcd.print("      ");
      lcd.setCursor(0,0);
      lcd.setCursor(0,0);
      lcd.print(brewTempSetting);
      
   }else if ((setSteamTemp == true) && (left == true))  {
      //Serial.print("Decrementing STEAM temp");
      steamTempSetting = steamTempSetting - 1;
      //Serial.println(steamTempSetting);
      lcd.setCursor(0,0);
      lcd.print("      ");
      lcd.setCursor(0,0);
      lcd.print(steamTempSetting);
     
   }else if ((setSteamTemp == true) && (right == true))  {
      //Serial.print("Incrementing STEAM temp");
      steamTempSetting = steamTempSetting + 1;
      //Serial.println(steamTempSetting);
      lcd.setCursor(0,0);
      lcd.print("      ");
      lcd.setCursor(0,0);
      lcd.print(steamTempSetting);
     
   }else if ((setOffsetBool == true) && (right == true))  {
      //Serial.print("offset increasing ");
      offsetSetting = offsetSetting + 0.25;
      //Serial.println(offsetSetting);
      lcd.setCursor(0,0);
      lcd.print(offsetSetting);
      lcd.print("     ");
     
   }else if ((setOffsetBool == true) && (left == true))  {
      //Serial.print("offset dropping ");
     offsetSetting = offsetSetting - 0.25;
      //Serial.print(offsetSetting);
      lcd.setCursor(0,0);
      lcd.print(offsetSetting);
      lcd.print("     ");
  
  }else if ((setTempScale == true) && (left == true))  {
      //Serial.println("Changing temp scale LEFT");
    
    
  }else if ((setTempScale == true) && (right == true))  {
      //Serial.println("Changing Temp Scale RIGHT");
    }
  }


void setMenuState(int brewTempSet, int steamTempSet, int pValSet, int iValSet, int dValSet, int offsetSet)  {
  setBrewTemp = brewTempSet;
  setSteamTemp = steamTempSet;
  setOffsetBool = offsetSet;

  }


