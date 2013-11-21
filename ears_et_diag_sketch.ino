/*
 * ET Diag Sketch for Fio V3 + BLEBee
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
#include "types.h"
/*
1: setup
+ setup pins
+ setup coms
+ check for faults
+ sessionStatus = idle
2: Control loop

*/

//sensor values
unsigned int pressureSensorValue = 0;



//interval between actions
unsigned long articulationIntervalMillis = 5;//happening 200x a second 
unsigned long measurementIntervalMillis = 10;
unsigned long dataAcqIntervalMillis = 100;

//need to speicify these
int motorRelayPin = 28;
int solenoidD2APin = 29;
int pressureSensorA2DPin = 30;

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
void respondToRequests(){
  //for now, just debug incoming commands
  if (Serial1.available()) {
    String command = String(Serial1.read());
    debugLog("BT Control: " +  command);
    
    //we'll compare command to available commands
    if (command == "startSession"){
      sessionState = beginState; 
    }else if (command == "endSession"){
      sessionState = endState; 
    }else if (command == "estop"){
      sessionState = estopState;
    }
    
  }
}

//save the data somewhere
void updateDataAcquisition(){
 String nextEntry = "{ \"sessionState\": "  + String(sessionState) + ", \"dataMillis\":" + String(lastMeasurementMillis) + ", \"pressure\":" + String(pressureSensorValue) + "}";
 //we save to an SD card if we have one, but we'll write to serial1(BT) & log for now!
 debugLog (nextEntry);
 //bluetooth!
 Serial1.print(nextEntry); 
}

//actuate
void articulateActuators(){
  if (sessionState == activeState && systemStatus == goodStatus){
    //motor runs pump coninuously
    //stopMotor
    digitalWrite(motorRelayPin, HIGH);
    
    //now do your fancy solenoid control thang 

  }else{
    //stopMotor
    digitalWrite(motorRelayPin, LOW);
    //close solenoid
    analogWrite(solenoidD2APin, 0);      
    if (sessionState == estopState || systemStatus == faultStatus){
        debugLog("waiting-- estopped or faulted"); 
    }
  
  }

};

/*
- Articulate Solinoid
+ if ESTOP or if controlActive == False,
++ Stop motor, stop solenoid
++ Articulate stop state
+ else if controlActive == True,
+ keep running motor, control solinoid
*/

//read sesnor data
void takeMeasurements() {
  lastMeasurementMillis = millis();
  
  pressureSensorValue = sin((1.0) * millis()/1000.0)*127 + 127; //or analogRead(A0) //a pin #
};

//process data 
void processData() {
  //process data from takeMeasurements
  //detemrine whether an ESTOP is needed, or fault is occuring
  //make it easy for articulateActuators function, and separate the processing logic
}
  
//setup and startup
void debugLog(String logPiece){
  //milisecondsSinceProgramStart: "log piece"
  const String seperator = String(": ");
  String toSend = String(millis(), DEC) + seperator + logPiece;
  
  
  //debug in the way most preferred 
  Serial.println(toSend);
} 

void setup()   {
  //pins 
  pinMode(motorRelayPin, OUTPUT);
  pinMode(solenoidD2APin, OUTPUT);
  pinMode(pressureSensorA2DPin, INPUT);
  
  // Set the baudrate of the Arduino
  Serial.begin(9600);
  delay(1000);
  debugLog("Beginning setup.");

  Serial1.begin(9600);
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




