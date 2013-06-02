/* 
 * Arduino PID AutoTune Code
 * by Brett Beauregard
 * April 2011
 
 
 * February 2013
 * Edited by Konstantine @Palanski to incorporate Autotune and PID Front End
 * Edits and additions marked with //KP
 
 *HOW TO USE:
 * Upload this code to Arduino with the appropriate pin settings and sensor settings.
 * Download Processing with the ControlP5 library and run the PID_FrontEnd_v03 (set appropriate COM)
 * myPort = new Serial(this, Serial.list()[0], 9600);  //Edit the [0] value to select appropriate COM
 * Allow the system to come to equilibrium after setting an initial setpoint
 * When system is at equilibrium, toggle Tuning to 'On'.
 * When the tuning is finished, the Tuning toggle will go to 'Off', and the new PID values will be on the screen.
 * SEARCH ALL CODE FOR //KP to see where edits MUST be made by YOU for this to be effective in the tuning
*/


//DS18B20 OneWire and DallasTemperature Library Initialization
//#include <OneWire.h>
//#include <DallasTemperature.h>
//#define oneWireBus 9
//OneWire oneWire(oneWireBus);
//DallasTemperature sensors(&oneWire);

//PID and PID Autotune Library Initialization

#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <LiquidCrystal.h>
#include "Adafruit_MAX31855.h"

int thermoDO = 3;
int thermoCS = 4;
int thermoCLK = 5;
Adafruit_MAX31855 thermocouple(thermoCLK, thermoCS, thermoDO);

const int numReadings = 10;

double readings[numReadings];      // the readings from the analog input
int index = 0;                  // the index of the current reading
double total = 0.0;                  // the running total
double average = 0.0;                // the average      
double tempReadF = 0.0;

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(14, 15, 16, 17, 18, 19);



//
byte ATuneModeRemember=2;
double Input, Output; 
double Setpoint = 212; //EDIT Set initial Setpoint
double kp=16,ki=0.5,kd=4; //KP Set intial PID parameters, if you have any.

double kpmodel=1.5, taup=100, theta[50];
double OutputStart=5;

double aTuneStep=750, aTuneNoise=1, aTuneStartValue=750; //KP Set aTuneStep to ensure noticeable heating and cooling from aTuneStartValue
unsigned int aTuneLookBack=60000; //KP How far back to look back to find maxima and minima. 
// For Slow processes this will be large, for fast processes, this will be smaller.

unsigned long  modelTime, serialTime;

PID myPID(&Input, &Output, &Setpoint,kp,ki,kd, DIRECT);
PID_ATune aTune(&Input, &Output);

//set to false to connect to the real world
boolean useSimulation = false; //EDIT Keep false unless simulating

//Tuning?
boolean tuning = false; //EDIT Keep False


const int relayPin = 7;    //EDIT Relay settings. (I'm running my code with a relay).
unsigned int WindowSize = 1000;
unsigned long windowStartTime;

void setup()
{ 
  windowStartTime = millis();
  pinMode(relayPin, OUTPUT); // Set relayPin as Output
  
  if(useSimulation)
  {
    for(byte i=0;i<50;i++)
    {
      theta[i]=OutputStart;
    }
    modelTime = 0;
  }
  
  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);
  
  //Setup the pid 
  myPID.SetMode(AUTOMATIC);

  if(tuning)
  {
    tuning=false;
    changeAutoTune();
    tuning=true;
  }
  
  aTune.SetControlType(1); //EDIT Set 0 for PI control and 1 for PID control autotuning parameters
  
  serialTime = 0;
  Serial.begin(9600);

}

void loop()
{
 unsigned long now = millis(); 
 if(!useSimulation)
  { //pull the Input in from the thermocouple
   //Pass to Input of PID
   tempRead();

   Input = average;
  }
  
  if(tuning)
  {
    byte val = (aTune.Runtime());
    if (val!=0)
    {
      tuning = false;
    }
    if(!tuning)
    { //we're done, set the tuning parameters
      kp = aTune.GetKp();
      ki = aTune.GetKi();
      kd = aTune.GetKd();
      myPID.SetTunings(kp,ki,kd);
      Serial.print("P="); Serial.print(kp);
      Serial.print("I="); Serial.print(ki);
      Serial.print("D="); Serial.print(kd);
      AutoTuneHelper(false);
    }
  }
  else myPID.Compute();
  
  if(useSimulation)
  {
    theta[30]=Output;
    if(now>=modelTime)
    {
      modelTime +=100; 
      DoModel();
    }
  }
  else // This is optimized for Relay output.
  { 
  if(now - windowStartTime>WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if(Output > now - windowStartTime) digitalWrite(relayPin,HIGH);
  else digitalWrite(relayPin,LOW);  
}
  
  //send-receive with processing if it's time
  if(millis()>serialTime)
  {
    SerialReceive();
    SerialSend();
    serialTime+=500;
  }
}


/********************************************
 * Serial Communication functions / helpers
 ********************************************/


union {                // This Data structure lets
  byte asBytes[24];    // us take the byte array
  float asFloat[6];    // sent from processing and
}                      // easily convert it to a
foo;                   // float array


void changeAutoTune()
{
 if(!tuning)
  {
    //Set the Output to the desired starting frequency.
    aTuneStartValue = Output; //KP Initial aTuneStartValue will be = Output at Toggle
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
  }
  else
  { //cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}

void AutoTuneHelper(boolean start)
{
  if(start)
    ATuneModeRemember = myPID.GetMode();
  else
    myPID.SetMode(ATuneModeRemember);
}


void SerialSend()
{
Serial.print("PID ");
  Serial.print(Setpoint);   
  Serial.print(" ");
  Serial.print(Input);   
  Serial.print(" ");
  Serial.print(Output);   
  Serial.print(" ");
  Serial.print(myPID.GetKp());   
  Serial.print(" ");
  Serial.print(myPID.GetKi());   
  Serial.print(" ");
  Serial.print(myPID.GetKd());   
  Serial.print(" ");
  if(myPID.GetMode()==AUTOMATIC) Serial.print("Automatic");
  else Serial.print("Manual");  
  Serial.print(" ");
  if(myPID.GetDirection()==DIRECT) Serial.print("Direct");
  else Serial.print("Reverse");
  Serial.print(" ");
  if(tuning==false) Serial.println("Off"); //KP Added the On/Off for Tuning Toggle
  else Serial.println("On");
}

void SerialReceive()
{
 
  // read the bytes sent from Processing
  int index=0;
  
  byte Auto_Man = -1;
  byte Direct_Reverse = -1;
  byte Tuning_Mode = -1; //KP Tuning Mode?
  
  while(Serial.available()&&index<26)
  {
    if(index==0) Auto_Man = Serial.read();
    else if(index==1) Direct_Reverse = Serial.read();
    else if(index==2) Tuning_Mode = Serial.read(); //KP Tuning Mode?
    else foo.asBytes[index-3] = Serial.read();
    index++;
  } 
  
  // if the information we got was in the correct format, 
  // read it into the system
  if(index==26  && (Auto_Man==0 || Auto_Man==1) && (Direct_Reverse==0 || Direct_Reverse==1) && (Tuning_Mode==0 || Tuning_Mode==1))
  {
    Setpoint=double(foo.asFloat[0]);
    //Input=double(foo.asFloat[1]);       // * the user has the ability to send the 
                                          //   value of "Input"  in most cases (as 
                                          //   in this one) this is not needed.
    if(Auto_Man==0)                       // * only change the Output if we are in 
    {                                     //   manual mode.  otherwise we'll get an
      Output=double(foo.asFloat[2]);      //   Output blip, then the controller will 
    }                                     //   overwrite.
    
    double p, i, d;                       // * read in and set the controller tunings
    p = double(foo.asFloat[3]);           //
    i = double(foo.asFloat[4]);           //
    d = double(foo.asFloat[5]);           //
    myPID.SetTunings(p, i, d);            //
    
    if(Auto_Man==0) myPID.SetMode(MANUAL);// * set the controller mode
    else myPID.SetMode(AUTOMATIC);             //
    
    if(Direct_Reverse==0) myPID.SetControllerDirection(DIRECT);// * set the controller Direction
    else myPID.SetControllerDirection(REVERSE);          //
    
    if(Tuning_Mode == 0) tuning=false; // Set Tuning mode on/off
    else tuning=true;
  }
  Serial.flush();                         // * clear any random data from the serial buffer
}

void DoModel()
{
  //cycle the dead time
  for(byte i=0;i<49;i++)
  {
    theta[i] = theta[i+1];
  }
  //compute the Input
  Input = (kpmodel / taup) *(theta[0]-OutputStart) + Input*(1-1/taup) + ((float)random(-10,10))/100;

}

void tempRead()  {
   double c = thermocouple.readCelsius();
   if (isnan(c)) {
     Serial.println("Something wrong with thermocouple!"); 
     lcd.setCursor(0,0);
     lcd.print("Thermocouple Err");
   } else {
    // Serial.print("C = "); 
    // Serial.println(c);
   
  //tempReadF = thermocouple.readInternal();
   tempReadF = thermocouple.readFarenheit();
    
   total= total - readings[index];         
    // read from the sensor:  
   readings[index] = tempReadF; 
    // add the reading to the total:
   total= total + readings[index];       
    // advance to the next position in the array:  
   index = index + 1;                    

    // if we're at the end of the array...
   if (index >= numReadings)              
    // ...wrap around to the beginning: 
   index = 0;                           

   // calculate the average:
   average = total / numReadings;         
   // send it to the computer as ASCII digits
    
   //Serial.print("Internal Temp = ");
   //Serial.println(thermocouple.readInternal());
   lcd.setCursor(0,1);
   lcd.print(average);
   lcd.print((char)223);
   //Serial.print("Temp ");
   Serial.print(average);
   Serial.print("\t");
   }
   
}
