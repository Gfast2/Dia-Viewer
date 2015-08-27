/* This is the firmware of a Diabetrachter (Dia Viewer), which can mount 10 piece of Dia on his rolling wheel. 
 * User push the mechanical push button to see the next Dia. There is a position sensor build besides the 
 * rolling wheel. The sensor will be triggered one time on each round, that is used to calibrate the  pos-
 * ition of each Dia that are mounted on the wheel. In the controller modul of this machine there is a codier 
 * which can be set to 10 different values, that is used to set how many Dias mounted / should be displayed. 
 * At last there is a timer system to help reset the system (That means go back to the first Dia). If the mac-
 * hine is not used more than 40 seconds and the showned dia is not the first one, it will roll back to the 
 * first Dia automatically.
 *
 *
 * TODO  :  If the first Dia is always be exposured under light, it will change its color much faster than other.
 *          And I think it happens even under the light of low brightness LED lights.
 *
 *
 * Design References:
 * Button Debouncing      : http://playground.arduino.cc/Learning/SoftwareDebounce
 * Gabellichtschranke Pnp : http://www.voelkner.de/products/37880/Gabellichtschranke-Pnp-Pm-F24p-5mm.html
 * Accelstepper library   : http://www.airspayce.com/mikem/arduino/AccelStepper
 *
 *
 * written by  :  Su Gao
 * last edite  :  27.AUG.2015
 *
 */
#include <AccelStepper.h>

AccelStepper motor(1, 2, 3); //(mode, st pin, dir pin) Tipp: https://www.pjrc.com/teensy/td_libs_AccelStepper.html

int st = 2;     // step motor driver step pin
int button = 8; // push button for user
int sensor = 6; // infarot sensor (for positioning) pin, using the white wire, 1-position hole, 0-normal.
int readPin[4] = {3,4,7,12};

boolean sensorState;     // default not see the position hole. false - no hole, true - hole (sensor triggered)
boolean buttonState;     // when button pushed down return LOW, else return HIGH
boolean buttonTriggered; // return true when debounced button read value changed & seen as "triggered",else return false.
int debounce_count = 50; // sample number for debouncing.
int dias = 0;            // read how many dias should be displayed.
int positionNow = 0;     // new targeted position in unit step
int diaNow = 1;          // position in unit dia number
unsigned long timeInterval = 40000; // When machine is not used more than 40 seconds, it will go back to begin & calibrate it self.
long caliSpeed = 2500;   //Unit: Microsecond. set the rotation speed when motor do calibration.

void setup(){  
  pinMode(st,    OUTPUT); //STEP pin of step motor
  pinMode(button, INPUT);
  digitalWrite(button, HIGH); // activate pull-up resistor in AVR  
  pinMode(sensor, INPUT);
  
  dias = diaN(); // check how many dias should be displayed. Note: This check only happen once at very beginning.
  cali();        //do startup calibration.
}

void loop(){
  buttonState = digitalRead(button);
  sensorState = digitalRead(sensor); //Lichtschrankgabe read value.
  
  //read the position of the wheel. If it is not the zero point, let it calibration, if yes, do nothing.
  timeControler(buttonState, sensorState, timeInterval); 
  
  buttonTriggered = buttonDebouncer(buttonState); // check button state without bouncing problem.
  
  if(buttonTriggered == true && motor.distanceToGo() == 0){ //only when motor stopped, reactive for button.
     long nextPosi = nextPosition(dias);                    //number of dias mounted on the machine.
     motor.moveTo(nextPosi); 
     buttonTriggered = false;
  }
  
  if(motor.currentPosition() >= 3200){ //if motor has finished to rotate around.
      motor.setCurrentPosition(0);     //Reset current position back to 0.
      positionNow = 0;
  }
  
  motor.run();
}

// If machine is not used more than timeInterval milliseconds and it is not showing the first dia.
// this method rotate the wheel back to the first dia. (This will block the programm flow)
void timeControler(boolean buttonState, boolean sensorState, unsigned long timeInterval){  
  static boolean bsOld = true;
  static unsigned long timeOld = 0;
  unsigned long timeNow = millis();

  if(sensorState == true)   // When sensor is triggered.
    timeOld = timeNow;      // Basically said like "deactivate timer system".
    
  if(buttonState != bsOld){ // If button is pressed. (meaningless to check debouncing here)
    timeOld = timeNow;
    bsOld = buttonState;
  }

  if(abs(timeNow - timeOld) > timeInterval){
      cali();               //at last execute one time calibration if needed.
      timeOld = timeNow;
  }
}


void cali(){
  while(!sensorDebouncer(digitalRead(sensor))){
    digitalWrite(st,HIGH); //As work around for the limit of accelstepper library
    delayMicroseconds(1);
    digitalWrite(st,LOW);
    delayMicroseconds(caliSpeed);
  }
  motor.setCurrentPosition(0); //hard stop motor + reset the position of motor.
  motor.setMaxSpeed(300);
  motor.setAcceleration(1600);
  diaNow = 1;
  positionNow = 0;
}

// Return true when sensor is triggered with debounced result, return false when sensor is not triggered
boolean sensorDebouncer(int sensorState){
  static int counter = 0;           // How many times we have see the new value
  static int sensorStateOld = true; // The old state of sensor read value.
  if(sensorState == sensorStateOld && counter > 0)
    counter--;
  if(sensorState != sensorStateOld)
    counter++;
  if(counter >= 1){ // It's actually meaningless to do sensor debouncing. So here set to 1 as work around
    counter = 0;
    sensorStateOld = sensorState;
    if(sensorState == true)
      return true;  // return true when sensor is triggered
  }
  return false;
}

// Check if button really pushed down without bouncing problems
// When button pressed down without bouncing issue return true, else return false.
int buttonDebouncer(int buttonState){
  static int counter = 0; // how many times we have seen new value.
  static boolean buttonStateOld = HIGH; 
  if(buttonState == buttonStateOld && counter > 0)
    counter--;
  if(buttonState != buttonStateOld)
    counter++;
  if(counter >= debounce_count){
    counter = 0;
    buttonStateOld = buttonState;
    if(buttonState == LOW) //only when the button is pushed down trigger motor to move.
      return true;
  }
  return false;
}

//return the nextPositon in absolute value to move to. 
//Argument diaNumber is used to set how many dias are mounted on the machine
long nextPosition(int diaNumber){
  if(diaNow < diaNumber){
    if(positionNow >= 3200) // When very fast constentively pushing button, positionNow will over flow.
      return positionNow;
    positionNow += 320;     // 200steps per round, 16th driver mode -> 3200 steps/round
    diaNow++;
  } else{
    positionNow = 3200;     // go to the beginning.
    diaNow = 1;             // reset to the beginning.
  }
  return positionNow;
}

//Read "Codier-Drehschalter" 4-bit raw values
int codierReader(){
  int rawValue = 0; // raw read value built up by four raw read value from codier connected pins.
  for(int i=0; i<4; i++){
    pinMode(readPin[i],INPUT);
    digitalWrite(readPin[i],HIGH);
    int tmp = ~digitalRead(readPin[i]); //Invert the output.
    rawValue += (tmp << (4-i));         //the first read value is the MSB (Most Significant Bit)
  }
  return rawValue;
}

//return the number of Dias are mounted on machine.
int diaN(){
  //I read the number from hardware directlly.
  int realCode[10] = {-60,-44,-52,-36,-56,-40,-48,-32,-58,-42};
  int codierNumber = codierReader(); // Read value from codier right now.
  
  for(int i =0; i<10; i++){
    if(realCode[i] == codierNumber){
      if(i == 0) 
        return 10; // When codier point to 0, this set as default for all dias mounted on it.
      return i;    // return the decoded number of dias should be shown on going mashines.
    }
  }
  return 0;        // Only when read wrong value, return 0.
}
