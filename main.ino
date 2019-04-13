// Libraries used in BFS algorithm
#include <StackList.h>
#include <QueueArray.h>

/////////////////////////////////////////////////////////////////////////
// Wifi variables
////////////////////////////////////////////////////////////////////////
#ifndef __CC3200R1M1RGC__
#include <SPI.h>
#endif
#include <WiFi.h>

char ssid[] = "NETGEAR65";
char password[] = "littlecello367";
IPAddress server(192,168,1,2);
WiFiClient client;

////////////////////////////////////////////////////////////////////////
// Motor Control variables
////////////////////////////////////////////////////////////////////////
int rightMotorPWM = 40;
int rightMotorPhase = 39;
int leftMotorPWM = 38;
int leftMotorPhase = 34;
int motorOffset = 4;
int speedVal = 220; //180
int harderTurn = 220;
int medTurn = 130;
int softTurn = 70;
int turnOnSpotVal = 130; // 90

////////////////////////////////////////////////////////////////////////
// Sensor variables
////////////////////////////////////////////////////////////////////////
int AnalogValue[5] = {0,0,0,0,0};
int sensorOnOff[5] = {0,0,0,0,0};
int AnalogPin[5] = {A9, A11, A13, A8, A14};
int distanceSensorPin = 2;
int distanceThreshold = 680;

int threshold = 150;
bool reachedWhiteMark = false;
bool lost = false;
unsigned long currentTime;

////////////////////////////////////////////////////////////////////////////////////
// Breadth First Search Variables
////////////////////////////////////////////////////////////////////////////////////
bool marked[8];
int edgeTo[8]; // parent array
StackList<int> path;
int current = 0;
int cameFrom = 4;
int dest;
bool edge[8][8] = { // Track stored as a truth table
  //       0      1      2      3      4      5     6       7
  /*0*/ {false, false, false, false, true,  false, true,  false},
  /*1*/ {false, false, false, false, false, false, true,  true},
  /*2*/ {false, false, false, true,  false, false, true,  false},
  /*3*/ {false, false, true,  false, false, false, false, true},
  /*4*/ {true,  false, false, false, false, false, false, true},
  /*5*/ {false, false, false, false, false, false, false, false},
  /*6*/ {true,  true,  true,  false, false, false, false, false},
  /*7*/ {false, true,  false, true,  true,  false, false, false},
};

////////////////////////////////////////////////////////////////////////////////////
// Setup
////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(9600);
  Serial.println("Begin");

  pinMode(distanceSensorPin, INPUT);

  // Wifi setup
  connectToServer();

  // drive to 0 checkpoint
  drive();
  stopLine();
  reachedWhiteMark = false;
}

////////////////////////////////////////////////////////////////////////////
// Main Loop
////////////////////////////////////////////////////////////////////////////
void loop(){

  // send current location and receive next destination
  receiveDest(current);

  // find shortest routes to all checkpoints
  breadthFirstSearch(current);

  // find shortest route to the specified destination
  // but if dest == 5, first go to 7, then 5
  if(dest == 5){
    pathTo(7);
  }
  else {
    pathTo(dest);
  }

  // 'path' contains the checkpoints to get to the destination
  while(!path.isEmpty()) {

    // conditions for each checkpoint on track, based on the direction you are approaching them
    switch(path.pop()) {

      case 0:
        if(current == 0){
          drive();
        }
        else if(current == 4){
          if(cameFrom == 0){
            turn180();
            drive();
          }
          else if(cameFrom == 7){
            drive();
          }
          current = 0;
          cameFrom = 4;
        }
        else if(current == 6){
          if(cameFrom == 1){
            turn90RightSix();
            drive();
          }
          else if(cameFrom == 2){
            drive();
          }
          current = 0;
          cameFrom = 6;
        }
        break;

      case 1:
        if(current == 6){
          if(cameFrom == 0){
            turn90Left();
            drive();
          }
          else if(cameFrom == 2){
            turn90Right();
            drive();
          }
          current = 1;
          cameFrom = 6;
        }
        else if(current == 7){
          if(cameFrom == 4){
            turn90Right();
            drive();
          }
          else if(cameFrom == 3){
            turn90Left();
            drive();
          }
          current = 1;
          cameFrom = 7;
        }
        break;

      case 2:
        if(current == 3){
          if(cameFrom == 2){
            turn180();
            drive();
          }
          else if(cameFrom == 7){
            drive();
          }
          current = 2;
          cameFrom = 3;
        }
        else if(current == 6){
          if(cameFrom == 1){
            turn90LeftSix();
            drive();
          }
          else if(cameFrom == 0){
            drive();
          }
          current = 2;
          cameFrom = 6;
        }
        break;

      case 3:
        if(current == 2){
          if(cameFrom == 6){
            drive();
          }
          else if(cameFrom == 3){
            turn180();
            drive();
          }
          current = 3;
          cameFrom = 2;
        }
        else if(current == 7){
          if(cameFrom == 4){
            drive();
          }
          else if(cameFrom == 1){
            turn90Right();
            drive();
          }
          current = 3;
          cameFrom = 7;
        }
        break;

      case 4:
        if(current == 0){
          if(cameFrom == 6){
            drive();
          }
          else if(cameFrom == 4){
            turn180();
            drive();
          }
          current = 4;
          cameFrom = 0;
        }
        else if(current == 7){
          if(cameFrom == 3){
            drive();
          }
          else if(cameFrom == 1){
            turn90Left();
            drive();
          }
          current = 4;
          cameFrom = 7;
        }
        break;

      case 6:
        if(current == 0){
          if(cameFrom == 4){
            drive();
          }
          else if(cameFrom == 6){
            turn180();
            drive();
          }
          current = 6;
          cameFrom = 0;
        }
        else if(current == 1){
          if(cameFrom == 7){
            drive();
          }
          else if(cameFrom == 6){
            turn180();
            drive();
          }
          current = 6;
          cameFrom = 1;
        }
        else if(current == 2){
          if(cameFrom == 3){
            drive();
          }
          else if(cameFrom == 6){
            turn180();
            drive();
          }
          current = 6;
          cameFrom = 2;
        }
        break;

      case 7:
        if(current == 4){
          if(cameFrom == 0){
            drive();
          }
          else if(cameFrom == 7){
            turn180();
            drive();
          }
          current = 7;
          cameFrom = 4;
        }
        else if(current == 1){
          if(cameFrom == 6){
            drive();
          }
          else if(cameFrom == 7){
            turn180();
            drive();
          }
          current = 7;
          cameFrom = 1;
        }
        else if(current == 3){
          if(cameFrom == 2){
            drive();
          }
          else if(cameFrom == 7){
            turn180();
            drive();
          }
          current = 7;
          cameFrom = 3;
        }
        break;
    }
    reachedWhiteMark = false;
  }

  // when the above while loop exits, the destination is reached
  stopLine();

  // if 5 was the destination, drive to 5 using distance sensor
  if(dest == 5){
    if(cameFrom == 4){
      turn90Left();
    }
    else if(cameFrom == 3){
      turn90Right();
    }
    else if(cameFrom == 1){
      park();
    }
    park();
  }
}

// this function uses the distance sensor to park at the wall
void park(){

  // first drive up the remaining white line
  while(!isLost()){
    readValues();

    if(sensorOnOff[0] == 0 && sensorOnOff[1] == 0 && sensorOnOff[2] == 1 && sensorOnOff[3] == 0 && sensorOnOff[4] == 0){      // 0 0 1 0 0
      forward();
    }
    else if(sensorOnOff[0] == 0 && sensorOnOff[1] == 1 && sensorOnOff[2] == 0 && sensorOnOff[3] == 0 && sensorOnOff[4] == 0){ // 0 1 0 0 0
      turnLeft(medTurn);
    }
    else if(sensorOnOff[0] == 0 && sensorOnOff[1] == 0 && sensorOnOff[2] == 0 && sensorOnOff[3] == 1 && sensorOnOff[4] == 0){ // 0 0 0 1 0
      turnRight(medTurn);
    }
    else if(sensorOnOff[0] == 0 && sensorOnOff[1] == 1 && sensorOnOff[2] == 1 && sensorOnOff[3] == 0 && sensorOnOff[4] == 0){ // 0 1 1 0 0
      turnLeft(softTurn);
    }
    else if(sensorOnOff[0] == 0 && sensorOnOff[1] == 0 && sensorOnOff[2] == 1 && sensorOnOff[3] == 1 && sensorOnOff[4] == 0){ // 0 0 1 1 0
      turnRight(softTurn);
    }
  }

  // then keep driving forward into the wild
  while(readDistanceSensor() < distanceThreshold){
    forward();
  }

  // stop and update server
  stopLine();
  receiveDest(5);
}

////////////////////////////////////////////////////////////////////////////
// Path finding algorithm
////////////////////////////////////////////////////////////////////////////
void breadthFirstSearch(int node){

  // reset variables
  for(int i=0; i<8; i++){
    edgeTo[i] = 0;
    marked[i] = false;
  }

  // 'marked' means the node has been visited
  marked[node] = true;

  // 'q' contains the unexplored nodes
  QueueArray<int> q;
  q.push(node);

  while(!q.isEmpty()){

    // take a node...
    int v = q.pop();

    // and explore each neighbour of 'v'
    for(int i=0; i<8; i++){

      // first check if they are connected
      if(edge[v][i] || edge[i][v]){

        // then check if they have been visited before
        if(!marked[i]){
          marked[i] = true;
          q.push(i);

          // store the parent of current node 'i' as 'v'
          edgeTo[i] = v;
        }
      }
    }
  }
}

// traverse back up minimum spanning tree
void pathTo(int destination) {

  for(int i=destination; i != current; i = edgeTo[i]){
    path.push(i);
    if(path.peek() == current){
      break;
    }
  }
}

////////////////////////////////////////////////////////////////////////////
// Wifi functions
////////////////////////////////////////////////////////////////////////////
void connectToServer(){

  Serial.print("Attempting to connect to Network named: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  while ( WiFi.status() != WL_CONNECTED);

  Serial.println("\nYou're connected to the network");
  Serial.println("Waiting for an ip address");

  while (WiFi.localIP() == INADDR_NONE);

  Serial.println("\nIP Address obtained");
  printWifiStatus();
}

void receiveDest(int currentPos){

    Serial.println("\nStarting connection to server...");
    // if you get a connection, report back via serial:
    if (client.connect(server, 80)) {

      Serial.println("connected to server");
      // Make a HTTP request:
      String messageToSend = "POST /mobilerobotics/api/position/tag/?group=2&pos=";
      messageToSend += String(currentPos);
      messageToSend += " HTTP/1.1";

      Serial.println("Sending message...");
      client.println(messageToSend);
      client.println("Host: energia.nu");
      client.println("Connection: close");
      client.println();
    }

    // Read reply from server
    char buffer[255] = {0};
    while (!client.available()) {}; // Wait for connection to be available...
    if (client.available()) {
     client.read((uint8_t*)buffer, client.available());
    }
    String finalMessage = buffer;

    Serial.println("Full message received:");
    Serial.println(finalMessage);
    Serial.println("End of received");

    // Extract next destination by splitting up the string
    String jsonMessage = finalMessage.substring(finalMessage.indexOf("{"), finalMessage.length()-1);
    String nextPosition = jsonMessage.substring(16, 17);

    // end of trip, do nothing
    if(nextPosition.equals("T")){
      while(true);
    }

    // otherwise convert to integer and store in dest
    dest = nextPosition.toInt();

    // if the server's disconnected, stop the client:
    if (!client.connected()) {
      Serial.println();
      Serial.println("disconnecting from server.");
      client.stop();
    }
}

void printWifiStatus() {
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
}

////////////////////////////////////////////////////////////////////////////
// Line following functions
////////////////////////////////////////////////////////////////////////////

// this function just follows the white line and returns when a checkpoint is reached
void drive() {
  while(reachedWhiteMark == false){

    readValues();

    //normal operation
    if(sensorOnOff[0] == 0 && sensorOnOff[1] == 0 && sensorOnOff[2] == 1 && sensorOnOff[3] == 0 && sensorOnOff[4] == 0){      // 0 0 1 0 0
      lost = false;
      forward();
    }
    else if(sensorOnOff[0] == 0 && sensorOnOff[1] == 1 && sensorOnOff[2] == 0 && sensorOnOff[3] == 0 && sensorOnOff[4] == 0){ // 0 1 0 0 0
      lost = false;
      turnLeft(medTurn);
    }
    else if(sensorOnOff[0] == 0 && sensorOnOff[1] == 0 && sensorOnOff[2] == 0 && sensorOnOff[3] == 1 && sensorOnOff[4] == 0){ // 0 0 0 1 0
      lost = false;
      turnRight(medTurn);
    }
    else if(sensorOnOff[0] == 0 && sensorOnOff[1] == 1 && sensorOnOff[2] == 1 && sensorOnOff[3] == 0 && sensorOnOff[4] == 0){ // 0 1 1 0 0
      lost = false;
      turnLeft(softTurn);
    }
    else if(sensorOnOff[0] == 0 && sensorOnOff[1] == 0 && sensorOnOff[2] == 1 && sensorOnOff[3] == 1 && sensorOnOff[4] == 0){ // 0 0 1 1 0
      lost = false;
      turnRight(softTurn);
    }
    else if(sensorOnOff[0] == 1 && sensorOnOff[1] == 1 && sensorOnOff[2] == 0 && sensorOnOff[3] == 0 && sensorOnOff[4] == 0){ // 1 1 0 0 0
      lost = false;
      turnLeftHard();
    }
    else if(sensorOnOff[0] == 0 && sensorOnOff[1] == 0 && sensorOnOff[2] == 0 && sensorOnOff[3] == 1 && sensorOnOff[4] == 1){ // 0 0 0 1 1
      lost = false;
      turnRightHard();
    }
    else if(sensorOnOff[0] == 1 && sensorOnOff[1] == 0 && sensorOnOff[2] == 0 && sensorOnOff[3] == 0 && sensorOnOff[4] == 0){ // 1 0 0 0 0
      lost = false;
      turnLeftHard();
    }
    else if(sensorOnOff[0] == 0 && sensorOnOff[1] == 0 && sensorOnOff[2] == 0 && sensorOnOff[3] == 0 && sensorOnOff[4] == 1){ // 0 0 0 0 1
      lost = false;
      turnRightHard();
    }

    // checkpoint reached, therefore stop just after white line
    else if(isAtStopLine()){
      lost = false;

      // wait until we have drove past the white mark
      while(isAtStopLine());

      Serial.println("Reached white mark");
      reachedWhiteMark = true;
    }

    // if car has lost the white line, wait for 1000ms before driving backwards
    else if(isLost()){

      // first time we are lost, record the time, break
      if(lost == false){
        currentTime = millis();
        lost = true;
      }

      // only if we are lost for more than 1000 milliseconds, reverse
      else if(millis() - currentTime >= 1000){
        stopLine();
        while(isLost()){
          reverse();
        }
      }
    }
  }
}

// returns true if the sensors detect a checkpoint
bool isAtStopLine(){
    readValues();

    if(sensorOnOff[0] == 1 && sensorOnOff[1] == 1 && sensorOnOff[2] == 1 && sensorOnOff[3] == 1 && sensorOnOff[4] == 1){ // 1 1 1 1 1
      return true;
    }
    else if(sensorOnOff[0] == 1 && sensorOnOff[1] == 1 && sensorOnOff[2] == 1 && sensorOnOff[3] == 0 && sensorOnOff[4] == 0){ // 1 1 1 0 0
      return true;
    }
    else if(sensorOnOff[0] == 0 && sensorOnOff[1] == 0 && sensorOnOff[2] == 1 && sensorOnOff[3] == 1 && sensorOnOff[4] == 1){ // 0 0 1 1 1
      return true;
    }
    else if(sensorOnOff[0] == 0 && sensorOnOff[1] == 1 && sensorOnOff[2] == 1 && sensorOnOff[3] == 1 && sensorOnOff[4] == 1){ // 0 1 1 1 1
      return true;
    }
    else if(sensorOnOff[0] == 1 && sensorOnOff[1] == 1 && sensorOnOff[2] == 1 && sensorOnOff[3] == 1 && sensorOnOff[4] == 0){ // 1 1 1 1 0
      return true;
    }

    return false;
}

// returns true if the sensors detect the normal straight line
// the function is used in the 180 degree turning function
bool isAtStraightLine(){
  readValues();

  if(sensorOnOff[0] == 0 && sensorOnOff[1] == 0 && sensorOnOff[2] == 1 && sensorOnOff[3] == 0 && sensorOnOff[4] == 0){      // 0 0 1 0 0
    return true;
  }
  else if(sensorOnOff[0] == 0 && sensorOnOff[1] == 1 && sensorOnOff[2] == 0 && sensorOnOff[3] == 0 && sensorOnOff[4] == 0){ // 0 1 0 0 0
    return true;
  }
  else if(sensorOnOff[0] == 0 && sensorOnOff[1] == 0 && sensorOnOff[2] == 0 && sensorOnOff[3] == 1 && sensorOnOff[4] == 0){ // 0 0 0 1 0
    return true;
  }

  return false;
}

// returns true if lost
bool isLost(){
  readValues();

  if(sensorOnOff[0] == 0 && sensorOnOff[1] == 0 && sensorOnOff[2] == 0 && sensorOnOff[3] == 0 && sensorOnOff[4] == 0){
    return true;
  }

  return false;
}

// reads the line sensor values
void readValues(){

  //take sampleAmount sets of readings
  int AnalogAverage[5] = {0,0,0,0,0};
  int sampleAmount = 50;

  for(int i=0; i<sampleAmount; i++){
    for (int j=0; j<5; j++){
      AnalogAverage[j] += analogRead(AnalogPin[j]);
    }
  }

  //average them by dividing by sampleAmount
  for(int i=0; i<5; i++){
    AnalogValue[i] = AnalogAverage[i] / sampleAmount;

    if(AnalogValue[i] <= threshold)
      sensorOnOff[i] = 1;
    else
      sensorOnOff[i] = 0;
  }
}

// reads the sharp distance sensor
int readDistanceSensor(){
  return analogRead(distanceSensorPin);
}

//////////////////////////////////////////////////////////////////////////////
// Motor control functions
//////////////////////////////////////////////////////////////////////////////
void forward(){
  digitalWrite(rightMotorPhase, LOW); //forward
  digitalWrite(leftMotorPhase, LOW ); //forward
  analogWrite(rightMotorPWM, speedVal - motorOffset); // set speed of motor
  analogWrite(leftMotorPWM, speedVal); // set speed of motor
}

void turnLeft(int leftTurnOffset){
  digitalWrite(rightMotorPhase, LOW); //forward
  digitalWrite(leftMotorPhase, LOW ); //forward
  analogWrite(rightMotorPWM, speedVal); // set speed of motor
  analogWrite(leftMotorPWM, speedVal - leftTurnOffset); // set speed of motor
}

void turnRight(int rightTurnOffset){
  digitalWrite(rightMotorPhase, LOW); //forward
  digitalWrite(leftMotorPhase, LOW ); //forward
  analogWrite(rightMotorPWM, speedVal - rightTurnOffset); // set speed of motor
  analogWrite(leftMotorPWM, speedVal); // set speed of motor
}

void turnLeftHard(){
  digitalWrite(rightMotorPhase, LOW); //forward
  digitalWrite(leftMotorPhase, LOW ); //forward
  analogWrite(rightMotorPWM, harderTurn); // set speed of motor
  analogWrite(leftMotorPWM, 0); // set speed of motor
}

void turnRightHard(){
  digitalWrite(rightMotorPhase, LOW); //forward
  digitalWrite(leftMotorPhase, LOW ); //forward
  analogWrite(rightMotorPWM, 0); // set speed of motor
  analogWrite(leftMotorPWM, harderTurn); // set speed of motor
}

void stopLine(){
  analogWrite(rightMotorPWM, 0); // set speed of motor
  analogWrite(leftMotorPWM, 0); // set speed of motor
  Serial.println("Stop"); // Display motor direction
}

void turn180(){

  //clockwise turn
  digitalWrite(rightMotorPhase, HIGH); //backward
  digitalWrite(leftMotorPhase, LOW ); //forward

  while(sensorOnOff[4] == 0){
    analogWrite(rightMotorPWM, turnOnSpotVal); // set speed of motor
    analogWrite(leftMotorPWM, turnOnSpotVal); // set speed of motor
    Serial.println("Turning 180"); // Display motor direction
    readValues();
  }

  while(sensorOnOff[3] == 0){
    analogWrite(rightMotorPWM, turnOnSpotVal); // set speed of motor
    analogWrite(leftMotorPWM, turnOnSpotVal); // set speed of motor
    Serial.println("Turning 180"); // Display motor direction
    readValues();
  }

  while(sensorOnOff[2] == 0){
    analogWrite(rightMotorPWM, turnOnSpotVal); // set speed of motor
    analogWrite(leftMotorPWM, turnOnSpotVal); // set speed of motor
    Serial.println("Turning 180"); // Display motor direction
    readValues();
  }

  while(sensorOnOff[1] == 0){
    analogWrite(rightMotorPWM, turnOnSpotVal); // set speed of motor
    analogWrite(leftMotorPWM, turnOnSpotVal); // set speed of motor
    Serial.println("Turning 180"); // Display motor direction
    readValues();
  }

  while(sensorOnOff[0] == 0){
    analogWrite(rightMotorPWM, turnOnSpotVal); // set speed of motor
    analogWrite(leftMotorPWM, turnOnSpotVal); // set speed of motor
    Serial.println("Turning 180"); // Display motor direction
    readValues();
  }

  while(!isAtStraightLine()){
    analogWrite(rightMotorPWM, turnOnSpotVal); // set speed of motor
    analogWrite(leftMotorPWM, turnOnSpotVal); // set speed of motor
    Serial.println("Turning 180"); // Display motor direction
    readValues();
  }
}

void turn90Right(){
  digitalWrite(rightMotorPhase, HIGH); //backward
  digitalWrite(leftMotorPhase, LOW ); //forward

  while(sensorOnOff[1] == 0){
    analogWrite(rightMotorPWM, turnOnSpotVal); // set speed of motor
    analogWrite(leftMotorPWM, turnOnSpotVal); // set speed of motor
    Serial.println("Turning Right"); // Display motor direction
    readValues();
  }

  while(sensorOnOff[0] == 0){
    analogWrite(rightMotorPWM, turnOnSpotVal); // set speed of motor
    analogWrite(leftMotorPWM, turnOnSpotVal); // set speed of motor
    Serial.println("Turning Right"); // Display motor direction
    readValues();
  }

  while(sensorOnOff[0] == 1){
    analogWrite(rightMotorPWM, turnOnSpotVal); // set speed of motor
    analogWrite(leftMotorPWM, turnOnSpotVal); // set speed of motor
    Serial.println("Turning Right"); // Display motor direction
    readValues();
  }

  while(sensorOnOff[2] == 0){
    analogWrite(rightMotorPWM, turnOnSpotVal); // set speed of motor
    analogWrite(leftMotorPWM, turnOnSpotVal); // set speed of motor
    Serial.println("Turning Right"); // Display motor direction
    readValues();
  }
}

void turn90Left(){
  digitalWrite(rightMotorPhase, LOW); //forward
  digitalWrite(leftMotorPhase, HIGH ); //backward

  while(sensorOnOff[0] == 0){
    analogWrite(rightMotorPWM, turnOnSpotVal); // set speed of motor
    analogWrite(leftMotorPWM, turnOnSpotVal); // set speed of motor
    Serial.println("Turning Left"); // Display motor direction
    readValues();
  }

  while(sensorOnOff[1] == 0){
    analogWrite(rightMotorPWM, turnOnSpotVal); // set speed of motor
    analogWrite(leftMotorPWM, turnOnSpotVal); // set speed of motor
    Serial.println("Turning Left"); // Display motor direction
    readValues();
  }
}

// special case for checkpoint 6
void turn90RightSix(){
  digitalWrite(rightMotorPhase, HIGH); //backward
  digitalWrite(leftMotorPhase, LOW ); //forward

  while(sensorOnOff[3] == 0){
    analogWrite(rightMotorPWM, turnOnSpotVal); // set speed of motor
    analogWrite(leftMotorPWM, turnOnSpotVal); // set speed of motor
    Serial.println("Turning Right"); // Display motor direction
    readValues();
  }
}

// special case for checkpoint 6
void turn90LeftSix(){
  digitalWrite(rightMotorPhase, LOW); //forward
  digitalWrite(leftMotorPhase, HIGH ); //backward

  while(sensorOnOff[1] == 0){
    analogWrite(rightMotorPWM, turnOnSpotVal); // set speed of motor
    analogWrite(leftMotorPWM, turnOnSpotVal); // set speed of motor
    Serial.println("Turning Right"); // Display motor direction
    readValues();
  }
}

void reverse(){
  digitalWrite(rightMotorPhase, HIGH); //reverse
  digitalWrite(leftMotorPhase, HIGH ); //reverse
  analogWrite(rightMotorPWM, speedVal - motorOffset); // set speed of motor
  analogWrite(leftMotorPWM, speedVal); // set speed of motor
  Serial.println("Reverse"); // Display motor direction
}
