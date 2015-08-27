/* This is the firmware of a Diabetrachter (Dia Viewer), that can mount 10 piece of Dia on his rolling wheel. 
 * User push the mechanical push button to see the next Dia. There is a position sensor build besides the 
 * rolling wheel. The sensor will be triggered one time on each round, in order to calibrate the precise pos-
 * ition of each Dia that are mounted on the wheel. In the controller modul of this machine there is a poten-
 * tion meter to control the brightness of the backlight for Dias and a codier which have 10 different posit-
 * ions to set how many Dias mounted / should be displayed. At last there is a timer system to help reset the
 * system. If the machine is not used more than two minute and the showned dia is not the first one, it will 
 * roll back to the first Dia automatically.
 *
 *
 * TODO  :  If the first Dia is always be exposured under light, it will change its color much faster than other.
 *
 *
 * Design References:
 * Button Debouncing      : http://playground.arduino.cc/Learning/SoftwareDebounce
 * Gabellichtschranke Pnp : http://www.voelkner.de/products/37880/Gabellichtschranke-Pnp-Pm-F24p-5mm.html
 *
 *
 *
 * written by  :  Su Gao
 * last edite  :  27.AUG.2015
 *
 */
#include <AccelStepper.h>

int button = 8; // push button for user
int st = 2; // step motor driver step pin
int sensor = 6; // infarot sensor (for positioning) pin, using the white wire, 1-position hole, 0-normal.

boolean sensorState; // default not see the position h ole. false - no hole, true - hole (sensor triggered)
boolean buttonState; // button pushed down - LOW, else return HIGH
boolean buttonTriggered; //return true when triggered debounced triggered, return false when not triggered.
int debounce_count = 50; // number of millis/samples to consider before declaring a debounced input
int dias = 0; // read how many dias should be displayed.
int positionNow = 0; //new targeted position in unit step
int diaNow = 1; // position in unit dia number
unsigned long timeInterval = 40000; // When machine is not used more than 40 seconds, it will go back to start+calibrate it self,too.
long caliSpeed = 2500; //Unit: Microsecond. set the rotation speed when motor do calibration.
AccelStepper motor(1, 2, 3); //(mode, st pin, dir pin) Tipp: https://www.pjrc.com/teensy/td_libs_AccelStepper.html

void setup(){
  
  pinMode(st,OUTPUT); //STEP pin of step motor
  pinMode(button ,INPUT);
  digitalWrite(button, HIGH); // activate pull-up resistor in AVR  

  pinMode(sensor,INPUT);
  
  dias = diaN(); // check how many dias should be displayed. Note: This check only happen once at very beginning.
  cali(); //do startup calibration.
}


int stepToCorrect = 0; // buffer for steps to be correct.
void loop(){
  buttonState = digitalRead(button);
  sensorState = digitalRead(sensor); //Lichtschrankgabe read value.
  
  //read the position of the wheel. If it is not the zero point, let it calibration, if yes, do nothing.
  timeControler(buttonState, sensorState, timeInterval); 
  int stepToMove = stepCorrecter(sensorState); // Check if sensor is triggered and where it's triggered.
  buttonTriggered = buttonDebouncer(buttonState); // check button state without bouncing problem.
  
  if(stepToMove != 0){
    stepToCorrect = stepToMove-45; // 3200 stands 1/10 for a 200 steps/round step motor at 16th drive mode.
//    Serial.print("stepToCorrect: ");
//    Serial.println(stepToCorrect);
  }
  
  if(buttonTriggered == true && motor.distanceToGo() == 0){ //only when motor stopped, do reaction.
     long nextPosi = nextPosition(dias/*10*/); //dias is the number of dias mounted on the wheel.
     /*
     if(stepToCorrect != 0){
       nextPosi -= stepToCorrect; // Compensate the missing steps on next movement.
       stepToCorrect = 0;
     }
     */
     motor.moveTo(nextPosi); 
     buttonTriggered = false;
  }
  
  if(motor.currentPosition() >= 3200){ //if motor has finished to rotate around.
//    if(motor.targetPosition() == motor.currentPosition()){ //if it's finished to rotating.
      motor.setCurrentPosition(0); //Reset current position back to 0.
      positionNow = 0;
  //  }
  }
  
  motor.run();
  //Serial.println(motor.currentPosition());

  /*
  int ledBrightness = brightnessRead();
  setLed(ledBrightness);
  */
  //setLed(brightnessSetting()); // take care of the brightness of LEDs
  //Serial.println(digitalRead(sensor));
  //delay(3);
}

// If machine is not used more than timeInterval minute and it is not showing the first dia.
// this method rotate the wheel back to the front. (it will block the programm flow)
void timeControler(boolean buttonState, boolean sensorState, unsigned long timeInterval){  
  static boolean bsOld = true;
  static unsigned long timeOld = 0;
  unsigned long timeNow = millis();

  if(sensorState == true) // When sensor is triggered.
    timeOld = timeNow; // Basically said like "deactivate timer system".
    
  if(buttonState != bsOld){ // If button is pressed. (In this case, we really don't need to think about debouncing)
    timeOld = timeNow;
    bsOld = buttonState;
  }

  if(abs(timeNow - timeOld) > timeInterval){
      cali(); //at last execute one time calibration if needed.
      timeOld = timeNow;
  }
  //Serial.println(abs(timeNow - timeOld));
}


void cali(){
//  Serial.println("start cali()");
//  motor.setSpeed(1500); //set the speed.
//  motor.setMaxSpeed(3000);
  while(!sensorDebouncer(digitalRead(sensor))){
    //motor.runSpeed(); //run constant speed, slowly
    digitalWrite(st,HIGH); //As work around for the problem on the line above.
    delayMicroseconds(1);
    digitalWrite(st,LOW);
    delayMicroseconds(caliSpeed);
  }
  //Serial.println("jump out of while loop in cali()");
  motor.setCurrentPosition(0); //hard stop motor + reset the position of motor.
  motor.setMaxSpeed(300);
  motor.setAcceleration(1600);
  diaNow = 1;
  positionNow = 0;
//  Serial.println("jump out cali()");
}

// Return true when sensor is triggered with debounced result, return false when sensor is not triggered
boolean sensorDebouncer(int sensorState){
//  Serial.println("in sensorDebouncer()");
  static int counter = 0; // How many times we have see the new value
  static int sensorStateOld = true; // The old state of sensor read value.
  if(sensorState == sensorStateOld && counter > 0)
    counter--;
  if(sensorState != sensorStateOld)
    counter++;
  if(counter >= /*debounce_count*/1){ //TODO  :  is that meaningful to do 50 times counting and let motor do about 50 steps more than it needs?
    counter = 0;
    sensorStateOld = sensorState;
    if(sensorState == true)
      return true;// return true when sensor is triggered
  }
  return false;
}

//read the sensor state and return how many steps are still not finished.
// P.S. : I suspect there is only tiny "step lost" during time going 
// caused by unprecision of all physical parts.
int stepCorrecter(boolean sensorVal){
   if(sensorDebouncer(sensorVal) == true)
   //  if(sensorState == false)
      // motor.setCurrentPosition(0); //reset the current position to zero. Calibration.
     return motor.distanceToGo();
   return 0; // if during the process sensor not triggered.
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
//Argument diaNumber is setter to see how many dia moount on this Diabetrachter.
long nextPosition(int diaNumber){
  if(diaNow < diaNumber){
    if(positionNow >= 3200) //When very fast constentively pushing button, positionNow will over flow.
      return positionNow;
    positionNow += 320; // 200steps per round, 16th drive mode.
    diaNow++;
//    Serial.println("if state here");
  } else{
    positionNow = 3200; // go to the beginning.
    diaNow = 1; //reset to the beginning.
//    Serial.println("else state here");
  }
  /*
  Serial.print("diaNow = ");
  Serial.print(diaNow);
  Serial.print(" positionNow: ");
  Serial.println(positionNow);
  */
  return positionNow;
}
/*
// Get the target brigtness of all LEDs.
int brightnessRead(){
  return (int) (analogRead(brightSetting) / 4);
}

//set the brightness of all leds.
void setLed(int brightness){
  for(int i; i<5; i++)
    analogWrite(led[i], brightness);
}
*/
int readPin[4] = {3,4,7,12};
//int readVal[4] = {0,0,0,0}; //raw read values from pins connected to codier
//Read "Codier-Drehschalter" 4-bit raw values
int codierReader(){
  int rawValue = 0; // raw read value built up by four raw read value from codier connected pins.
  for(int i=0; i<4; i++){
    pinMode(readPin[i],INPUT);
    digitalWrite(readPin[i],HIGH);
    int tmp = ~digitalRead(readPin[i]); //Invert the output.
    rawValue += (tmp << (4-i)); //the first read value is the MSB (Most Significant Bit)
  }
  return rawValue;
}

//return the number of Dias are mounted on maschine.
int diaN(){
  int realCode[10] = {-60,-44,-52,-36,-56,-40,-48,-32,-58,-42
    /* 
    // The Binary code not works as I expected. 
    // but I read the Code above directlly.
    0b0000,
    0b1000,
    0b0100,
    0b1100,
    0b0010,
    0b1010,
    0b0110,
    0b1110,
    0b0001,
    0b1001
    */
  };
  int codierNumber = codierReader(); // Read value from codier right now.
  for(int i =0; i<10; i++){
    if(realCode[i] == codierNumber){
      if(i == 0) 
        return 10; // When codier point to 0, this set as default for all dias mounted on it.
      return i; // return the decoded number of dias should be shown on going mashines.
    }
  }
//  Serial.println("Codier doesn't give a correct number"); // TODO : Give out a more clearly exception.
  return 0; 
}
