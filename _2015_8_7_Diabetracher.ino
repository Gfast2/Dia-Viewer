/*  Dia Betrachter test Programme.
 *  This Programm control a step motor (later perhaps a BLDC) according to the
 *  Signal of a push button and a position sensor (IR sensor).
 *  The user push the button to see the next dia.
 *  There would be about 10 Dias on each Dia plate.
 *
 *  TODO: 
 *    don't use time based logic, in order to give end user more flexiblity to tune the speed of the maschine.
 *
 *  written by : Su Gao
 *  last edite : 7.Aug 2015
 */

#define ST  2
#define DIR 3
#define BUTTON 9

void setup() {
  Serial.begin(115200);
  Serial.println("I started");

  pinMode(ST, OUTPUT);
  pinMode(DIR, OUTPUT);
  digitalWrite(DIR, HIGH); //turn on the pull up resistor in Arduino.
  pinMode(13,OUTPUT); //The on board LED will be used as indicator to show if the sensor is triggered or not.
  digitalWrite(13,HIGH);
  pinMode(A0,INPUT); // The acutal pin being used as sensor pin, because the IR sensor I used is some how not "digitalRead() able".
  digitalWrite(A0,HIGH);
  pinMode(BUTTON,INPUT); // the only user interface to trigger the dia betrachter.
  digitalWrite(BUTTON,HIGH);
}

boolean bState = false; //button State
boolean bState_old = false; //button old state
boolean pushFlag = false; //button pushed flag
unsigned long timeStamp = 0; //time now
unsigned long timeStamp_old = 0; //time stamp old

void loop() {
  timeStamp = millis();

  bState = digitalRead(BUTTON); //pushed = LOW
  Serial.println(bState);
  if(bState==false && bState_old ==true){ // button pushed
    pushFlag = true;
    timeStamp_old = timeStamp; // save the time when the button is pushed
  }

  int i = analogRead(A0);
  if(i < 500)    digitalWrite(13,LOW);
  else           digitalWrite(13,HIGH);

  if(pushFlag == true){ // If the button is pushed
    if(i > 500){ 
      if(timeStamp - timeStamp_old < 1000) // If the sensor still "see" the holes.
        moveMotor(); //move motor one step.
      else
        pushFlag == false; // Sensor see the hole again (the hole after the old holes)
    } else { //sensor "see" the hole no more.
      if(i < 500) //when the sensor can't see the hole during the rotation time.
        moveMotor();
    }
  }
  bState_old = digitalRead(BUTTON);
  delay(2); //slow down loop for my old computer. It can be comment out.
}

void moveMotor(){
  digitalWrite(ST, HIGH);
  delayMicroseconds(800);
  digitalWrite(ST, LOW);
  delayMicroseconds(800);

}


