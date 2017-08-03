/* Temperature Controller for Weber Smokey Mountiain using a ESP8266 and Thermocouple.
 *  
 */
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

#include <String.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>


//byte ATuneModeRemember=2;
double kp=35.21,ki=2,kd=30;

double kpmodel=1.5, taup=100, theta[50];
double outputStart=5;
double aTuneStep=100, aTuneNoise=1, aTuneStartValue=50;
unsigned int aTuneLookBack=20;

byte ATuneModeRemember;
unsigned long  modelTime, serialTime;
int modelStep, tuningStep;
boolean tuning = false;


//#define USE_SIMULATION
//#define SIMULATION_MODEL_BALANCED_LAG_AND_DELAY



extern Tuning tuningRule[];

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 4 
#define TEMPERATURE_PRECISION 9



// initialize the Thermocouple
const char *ssid = "webersmoker";
const char *password = "saintbbq";
int channel = 8;

ESP8266WebServer server(80);

double previousTemp = 0.0;
double pidTemp = 0.0; 
int tempC = 0;
int tempF = 0;
int tempSet = 250;
double tempSetPid = 250.0;
int probe2Temp = 0;
int probe3Temp = 0;
int tempTextColor = 3368448;
int controllerStateColor = 39168;
// Generally, you should use "unsigned long" for variables that hold time
unsigned long previousMillis = 0;        // will store last temp was read
const long interval = 100;              // interval at which to read sensor
const long recoveryTime = 180000;        // disable the fan for 3 minutes after we open the lid
long timeout = 0;
bool isOn = false;
bool lidOpen = false;
double fanOutput = 0.0;


PID myPID(&pidTemp, &fanOutput, &tempSetPid,kp,ki,kd , DIRECT);
PID_ATune aTune(&pidTemp, &fanOutput);

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device addresses
DeviceAddress probe1, probe2, probe3;


//root webapge
void handleRoot(){
  char temp[900];
  int sec = millis() / 1000;
  int min = sec / 60;
  int hr = min / 60;
  snprintf ( temp,900,

    "<html>\
      <head>\
        <meta http-equiv='refresh' content='5'/>\
        <title>ESP8266 Demo</title>\
        <style>\
          body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; text-align: center; font-size:200%; }\
        </style>\
      </head>\
      <body>\
        <h1>Weber Smoker Web Controller</h1>\
        <p>Uptime: %02d:%02d:%02d</p>\
        <p><h2 style=\"color:#%x;\"> Temperature Celsius: %d C </h2></p>\
        <p><h2 style=\"color:#%x;\"> Temperature Farenheit: %d F </h2></p>\
        <p><h2>Temperature Set : %d F </h2></p>\
        <p><h2 style =\"color:#%x;\">Fan Output : %d  % </h2></p>\
        <p>Probe 2 Temp : %d F</p>\
        <p>Probe 3 Temp : %d F</p>\
        <form action=/settings  method=POST >\
        <p>Set Temperature F\
        <input type=text name=setTemp >\
        <input type=submit value=Set></form>\
      </body>\
    </html>",

    hr, min % 60, sec % 60 , tempTextColor , int(tempC), tempTextColor, int(tempF) ,(int) tempSet , controllerStateColor , (int) ((float) fanOutput / 1024.0 * 100.0), probe2Temp , probe3Temp 
  );
  server.send ( 200, "text/html", temp );
}


//called when we set the temperature level
void handleSetting(){
  if (server.args() > 0 ) {
    for ( uint8_t i = 0; i < server.args(); i++ ) {
      if (server.argName(i) == "setTemp") {
        Serial.println("Arg "+ String(i)+"="+ server.arg(i));
        tempSet = server.arg(i).toInt();
        tempSetPid = tempSet;
        Serial.println("probe2Temp" + probe2Temp);
      }
   }
  }
  server.send(200, "text/plain", "Temperature set "  );
}

double getTemperatureC(DeviceAddress deviceAddress ) {
  float c = sensors.getTempC(deviceAddress);
  if (isnan(c)) {
     return 9999;
   } else {
     return c;
   }
}

double getTemperatureF(DeviceAddress deviceAddress ) {
  float c = sensors.getTempC(deviceAddress);
  if (isnan(c)) {
     return 9999;
   } else {
     return DallasTemperature::toFahrenheit(c);
   }
}

int selectTempTextColor(){
  if( tempF <= tempSet - 20 || tempF >= tempSet + 20)
  //return red
    return 13369344;
  else
  //return green
    return 39168;
}

int selectControllerStateColor(){
  if((int)fanOutput == 0)
    return 13369344;
  else
    return 39168;
}

void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}


void setup(void)
{


  pinMode(15,OUTPUT);
  Serial.begin(9600);
  Serial.println();
  sensors.begin();
  // locate devices on the bus
  Serial.print("Locating devices...");
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  myPID.SetMode(AUTOMATIC);
  aTune.SetControlType(PID_ATune::TYREUS_LUYBEN_PI);
  myPID.SetOutputLimits(0,1024);
  myPID.SetSampleTime(100);

   if(tuning)
  {
    tuning=false;
    changeAutoTune();
    tuning=true;
  }
  
  serialTime = 0;
  
  Serial.print("Parasite power is: "); 
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");

  //probe1 = { 0x3B, 0x1A, 0x3E, 0x88, 0x03, 0xDC , 0x2C ,0xA4};
  //probe2 = { 0x3B, 0xFD, 0x3C ,0x88, 0x03, 0xDC , 0x4C , 0xEE};

  oneWire.reset_search();
  // assigns the first address found to insideThermometer
  if (!oneWire.search(probe1)) Serial.println("Unable to find address for insideThermometer");
  // assigns the seconds address found to outsideThermometer
  if (!oneWire.search(probe2)) Serial.println("Unable to find address for outsideThermometer");


  //set the resolution to 9 bit
  sensors.setResolution(probe1, TEMPERATURE_PRECISION);
  sensors.setResolution(probe2, TEMPERATURE_PRECISION);
  //sensors.setResolution(probe2, TEMPERATURE_PRECISION);
  printAddress(probe1);
  Serial.println("");
  printAddress(probe2);
  Serial.println("");

  Serial.print("Device 0 Resolution: ");
  Serial.print(sensors.getResolution(probe1), DEC); 
  Serial.println();

  Serial.print("Device 1 Resolution: ");
  Serial.print(sensors.getResolution(probe2), DEC); 
  Serial.println();
  
  Serial.print("Configuring access point...");
  WiFi.mode(WIFI_AP);
  IPAddress apIP(192,168,1,1);
  WiFi.softAPConfig(apIP,apIP, IPAddress(255,255,255,0));
  //IPAddress myIP = WiFi.softAPIP();
  //Serial.print("AP IP address: ");
  //Serial.println(myIP);
  WiFi.softAP("Weber Smoker Controller");

  //dnsServer.start(DNS_PORT, "*" , apIP);
  
  server.begin();

  server.on ( "/", handleRoot );
  server.on ( "/settings", handleSetting);
  server.begin();

  Serial.println("HTTP server started");
}
 
void loop(void)
{
  unsigned long currentMillis = millis();

 if(currentMillis - previousMillis >= interval) {
    // save the last time you read the sensor 
    previousMillis = currentMillis;
    sensors.requestTemperatures();
    tempC = (int) getTemperatureC(probe1);
    previousTemp = pidTemp;
    pidTemp =  getTemperatureF(probe1);
    tempF = (int) pidTemp;
    Serial.println(tempF);
    probe2Temp = (int) getTemperatureF(probe2);
    //probe3Temp = getTemperatureF(probe3);
    Serial.println("");
    Serial.println(probe2Temp);

    if(previousTemp - pidTemp > 15 && previousTemp < 9998){
      lidOpen = true;
      timeout = millis();
      fanOutput = 0;
    }
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
      AutoTuneHelper(false);
    }
  }
  else {
    //tuningStep = 0;
    myPID.Compute();
  }
  if(fanOutput <= 0){
    fanOutput = 0;
    analogWrite(15,0);
  }
  else if(pidTemp >= tempSet){
    fanOutput = 0;
    analogWrite(15,0);
  }
  else{
    analogWrite(15, (int) fanOutput );
  }
  
  tempTextColor = selectTempTextColor();
  controllerStateColor = selectControllerStateColor();
  server.handleClient();
  
  //updateThingSpeak("1=" + String((int)humidity) + "&2=" + String((int)tempC) + "&3=" + String(humidifierStatus) + "&4=" + String(controllerStatus) + "&5=" + String(probe2Temp));

    //send-receive with processing if it's time
  if(millis()>serialTime)
  {
    SerialReceive();
    SerialSend();
    serialTime+=500;
  }
} 

void changeAutoTune()
{
 if(!tuning)
  {
    //Set the output to the desired starting frequency.
    fanOutput=aTuneStartValue;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
  }
  else
  { //cancel autotune
    Serial.println("Canceling Autotune");
    aTune.Cancel();
    tuning = false;
    kp = aTune.GetKp();
    ki = aTune.GetKi();
    kd = aTune.GetKd();
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
  Serial.print("setpoint: ");Serial.print(tempSetPid); Serial.print(" ");
  Serial.print("input: ");Serial.print(pidTemp); Serial.print(" ");
  Serial.print("output: ");Serial.print(fanOutput); Serial.print(" ");
  if(tuning){
    Serial.println("tuning mode");
  } else {
    Serial.print("kp: ");Serial.print(myPID.GetKp());Serial.print(" ");
    Serial.print("ki: ");Serial.print(myPID.GetKi());Serial.print(" ");
    Serial.print("kd: ");Serial.print(myPID.GetKd());Serial.println();
  }
}

void SerialReceive()
{
  if(Serial.available())
  {
   char b = Serial.read(); 
   Serial.flush(); 
   if((b=='1' && !tuning) || (b=='0' && tuning))changeAutoTune();
  }
}


/********************************************
 * Serial Communication functions / helpers
 ********************************************/


//union {                // This Data structure lets
//  byte asBytes[24];    // us take the byte array
//  float asFloat[6];    // sent from processing and
//}                      // easily convert it to a
//foo;                   // float array
//
//
//
//// getting float values from processing into the arduino
//// was no small task.  the way this program does it is
//// as follows:
////  * a float takes up 4 bytes.  in processing, convert
////    the array of floats we want to send, into an array
////    of bytes.
////  * send the bytes to the arduino
////  * use a data structure known as a union to convert
////    the array of bytes back into an array of floats
//
////  the bytes coming from the arduino follow the following
////  format:
////  0: 0=Manual, 1=Auto, else = ? error ?
////  1: 0=Direct, 1=Reverse, else = ? error ?
////  2-5: float setpoint
////  6-9: float input
////  10-13: float output  
////  14-17: float P_Param
////  18-21: float I_Param
////  22-245: float D_Param
//void SerialReceive()
//{
//
//  // read the bytes sent from Processing
//  int index=0;
//  byte Auto_Man = -1;
//  byte Direct_Reverse = -1;
//  while(Serial.available()&&index<26)
//  {
//    if(index==0) Auto_Man = Serial.read();
//    else if(index==1) Direct_Reverse = Serial.read();
//    else foo.asBytes[index-2] = Serial.read();
//    index++;
//  } 
//  
//  // if the information we got was in the correct format, 
//  // read it into the system
//  if(index==26  && (Auto_Man==0 || Auto_Man==1)&& (Direct_Reverse==0 || Direct_Reverse==1))
//  {
//    tempSetPid=double(foo.asFloat[0]);
//    //Input=double(foo.asFloat[1]);       // * the user has the ability to send the 
//                                          //   value of "Input"  in most cases (as 
//                                          //   in this one) this is not needed.
//    if(Auto_Man==0)                       // * only change the output if we are in 
//    {                                     //   manual mode.  otherwise we'll get an
//     fanOutput=double(foo.asFloat[2]);      //   output blip, then the controller will 
//    }                                     //   overwrite.
//    
//    double p, i, d;                       // * read in and set the controller tunings
//    p = double(foo.asFloat[3]);           //
//    i = double(foo.asFloat[4]);           //
//    d = double(foo.asFloat[5]);           //
//    myPID.SetTunings(p, i, d);            //
//    
//    if(Auto_Man==0) myPID.SetMode(MANUAL);// * set the controller mode
//    else myPID.SetMode(AUTOMATIC);             //
//    
//    if(Direct_Reverse==0) myPID.SetControllerDirection(DIRECT);// * set the controller Direction
//    else myPID.SetControllerDirection(REVERSE);          //
//  }
//  Serial.flush();                         // * clear any random data from the serial buffer
//}
//
//// unlike our tiny microprocessor, the processing ap
//// has no problem converting strings into floats, so
//// we can just send strings.  much easier than getting
//// floats from processing to here no?
//void SerialSend()
//{
//  Serial.print("PID ");
//  Serial.print(tempSetPid);   
//  Serial.print(" ");
//  Serial.print(pidTemp);   
//  Serial.print(" ");
//  Serial.print(fanOutput);   
//  Serial.print(" ");
//  Serial.print(myPID.GetKp());   
//  Serial.print(" ");
//  Serial.print(myPID.GetKi());   
//  Serial.print(" ");
//  Serial.print(myPID.GetKd());   
//  Serial.print(" ");
//  if(myPID.GetMode()==AUTOMATIC) Serial.print("Automatic");
//  else Serial.print("Manual");  
//  Serial.print(" ");
//  if(myPID.GetDirection()==DIRECT) Serial.println("Direct");
//  else Serial.println("Reverse");
//}


