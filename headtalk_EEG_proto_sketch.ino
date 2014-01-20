/*
 * HEADTALK PROTO EEG Sketch for Fio V3 + BLEBee w/ Mindflex
 * v0.0.1 2013-11-19
 * Copyright (c) 2013, Ears Team
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *^love, alex & jacob
*/


#include <SoftwareSerial.h>
#include "Brain.h"
#include "types.h"
/*
1: setup
+ setup pins
+ setup coms
+ check for faults
+ sessionStatus = idle
2: Control loop

*/


String sysVersion = "0.1";

//sensor values
unsigned int wristbandSensorValue = 0;
int shakeDetected;

//actions 
int shakeForTime = 0;

//interval between actions
unsigned long articulationIntervalMillis = 5;//happening 200x a second 
unsigned long measurementIntervalMillis = 50; //100x a second
unsigned long dataAcqIntervalMillis = 200;

//software serial pins
int softwareSerialInPin = D9;
int softwareSerialOutPin = D2;
int analogWristbandPin = A5;

//other pins

//software brain
SoftwareSerial brainEEGSerial(softwareSerialInPin, softwareSerialOutPin);
Brain brain(brainEEGSerial);


//system state
SystemStatus systemStatus = unknownStatus;
SessionState sessionState = pendingState;

//the loop
unsigned long lastArticulationMillis = 0;
unsigned long lastMeasurementMillis = 0;
unsigned long lastDataAcqMillis = 0;
void loop(){
  unsigned long currentMillis = millis();
  
  if (currentMillis - lastMeasurementMillis > measurementIntervalMillis){
    takeMeasurements();
    processData();   
    lastMeasurementMillis = currentMillis;
  }
  
  if (currentMillis - lastArticulationMillis > articulationIntervalMillis){
    articulateActuators();
    lastArticulationMillis = currentMillis;
  }
  
  if (currentMillis - lastDataAcqMillis > dataAcqIntervalMillis){
    updateDataAcquisition(); 
    lastDataAcqMillis = currentMillis;
  }
  
  respondToRequests();
  transitionStates();
}
  

//the loop's helpers
void transitionStates(){
  if (sessionState == beginState){
    sessionState = activeState;
    debugLog("began session");
  }else if (sessionState == endState){
    sessionState = idleState;
    debugLog("ended session");
  }else if (sessionState == estopState){
    //perhaps not the best place to put this...
    systemStatus =  faultStatus; 
    debugLog("system went fault via session");
  } 
}


//see whether pending requests exist on bluetooth control channel
//(control commnads<ESTOP, controlActive, endSession, newSession>)
String message = "";
void respondToRequests(){

  //for now, just debug incoming commands
  if (Serial1.available()) {

    // #potential problem of hang here?
    while(Serial1.available()){
      message += char(Serial1.read());
      // debugLog("BT read: " +  message);
    }

    int commandIndex = message.indexOf(";");
    if (commandIndex == -1){
      return;
    }

    String command = message.substring(0,commandIndex);
    message = message.substring(commandIndex +1);
    debugLog("BT Control: " +  command);
    
    //we'll compare command to available commands
    if (command == "startSession"){
      sessionState = beginState; 
    }else if (command == "endSession"){
      sessionState = endState; 
    }else if (command == "estop"){
      sessionState = estopState;
    }else if (command == "version"){
      sendControlJSON(&String("version"), &String(sysVersion));
    }else if (command == "flash"){
      pinMode(analogWristbandPin, OUTPUT);  
      shakeForTime = 200;
    }
    
  }
}

//save the data somewhere
void updateDataAcquisition(){
  String nextCSVEntry = String(sessionState) + "," + String(lastMeasurementMillis) + "," + String(wristbandSensorValue) + "," + String(shakeDetected)+ "," + String(shakeForTime) + "\n"; //+ String(brain.readSignalQuality()) + "," + String(brain.readMeditation()) + "," + String(brain.readAttention()) + "," + String(brain.readLowBeta()) + "," + String(brain.readLowGamma()) + "\n";
  // String csvLabels = "sessionState , dataMillis , brain.readSignalQuality(), brain.readMeditation(), brain.readAttention(),  brain.readLowBeta(), brain.readLowGamma(),  \n"

  debugLog("data: " + nextCSVEntry);

  //bluetooth!
  Serial1.print(nextCSVEntry); 
}

void sendControlJSON(String* property, String* value){
   String json = "{\""  + *property + "\":\""  + *value + "\"}\n";

   debugLog (json);
   //bluetooth!
   Serial1.print(json);
}


//actuate
void articulateActuators(){
  if (sessionState == activeState && systemStatus == goodStatus){
    //motor runs pump coninuously
    
    //now do your fancy solenoid control thang 

    if (shakeForTime > 0){

      analogWrite(analogWristbandPin, 255);

      shakeForTime = shakeForTime - ( millis() - lastArticulationMillis);
      
      if (shakeForTime <= 0){
          pinMode(analogWristbandPin, INPUT);  
          shakeForTime = 0;
      }

    }

  }else{
    //stopMotor
    // digitalWrite(motorRelayPin, LOW);
    //close solenoid
    // analogWrite(solenoidD2APin, 0);      
    if (sessionState == estopState || systemStatus == faultStatus){
        debugLog("waiting-- estopped or faulted"); 
    }
  
  }

};

//read sesnor data
void takeMeasurements() {

  wristbandSensorValue = analogRead(analogWristbandPin);

  if (brain.update()){ 
    debugLog("updated brain data.\n");
  }
  

};


//process data 
void processData() {
  //process data from takeMeasurements
  //detemrine whether an ESTOP is needed, or fault is occuring
  //make it easy for articulateActuators function, and *separate* the processing logic from the control logic as much as possible

  if (wristbandSensorValue > 1000){
    shakeDetected = true;
  }else{
    shakeDetected = false;
  }

}
  
//setup and startup
void setup()   {
  //pins 
  // analogWrite(analogWristbandPin, 255);//this works when OUTPUT

  pinMode(analogWristbandPin, INPUT);  
  //the symptom now is that 

  // Set the baudrate of the Arduino
  Serial.begin(9600);
  delay(3000);
  debugLog("Beginning setup.");

  Serial1.begin(9600);

  brainEEGSerial.begin(9600);
  // *brainEEG = Brain(*brainEEGSerial);

  delay(500);
  debugLog("..done setup.");
  
  if (checkSystemStatus() == goodStatus){
     debugLog("Began control loop.");
     sessionState = idleState;
  }else{
      debugLog("Faulted and aborted before entering control loop.");
      while(1){
      };
  }
}

SystemStatus checkSystemStatus(){
  
  boolean neverAFault = true;
  if (neverAFault == false){
    systemStatus = faultStatus;
  }
  
  //making sure status wasn't previously modified
  //if fault status, this ensures a reset is needed
  if (systemStatus == unknownStatus){
    //we checked to make sure stop button isn't on
    //we checked to make sure whatever was whatever
    systemStatus = goodStatus;
  }
  
  return systemStatus;
}

void debugLog(String logPiece){\
  //milisecondsSinceProgramStart: "log piece"
  //nice logging, printing to com port attached to computer
  const String seperator = String(": ");
  String toSend = String(millis(), DEC) + seperator + logPiece;
  
  
  //debug in the way most preferred 
  Serial.println(toSend);
} 




