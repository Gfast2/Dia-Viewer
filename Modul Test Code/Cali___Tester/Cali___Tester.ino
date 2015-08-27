#include <AccelStepper.h>
AccelStepper stepper(1,2,3);

int sensor = 8;
int debounce_count = 50;
void setup()
{  
  Serial.begin(115200);
  Serial.println("I started");

  pinMode(sensor,INPUT);

  stepper.setMaxSpeed(3000);
  stepper.setSpeed(1000);  
   /*
  while(!sensorDebouncer(digitalRead(sensor))){
    stepper.runSpeed(); //run constant speed, slowly
  }
  */
  stepper.setSpeed(1500);
  stepper.setMaxSpeed(3000);
  stepper.setAcceleration(1200);
  stepper.setCurrentPosition(0);
  stepper.moveTo(3600);
  Serial.println("jump out of while loop");
}
void loop()
{  
  stepper.run();
}
/*
// Return true when sensor is triggered with debounced result, return false when sensor is not triggered
boolean sensorDebouncer(int sensorState){
//  Serial.println("in sensorDebouncer()");
  static int counter = 0; // How many times we have see the new value
  static int sensorStateOld = true; // The old state of sensor read value.
  if(sensorState == sensorStateOld && counter > 0)
    counter--;
  if(sensorState != sensorStateOld)
    counter++;
  if(counter >= debounce_count){ //TODO  :  is that meaningful to do 50 times counting and let motor do about 50 steps more than it needs?
    counter = 0;
    sensorStateOld = sensorState;
    if(sensorState == true)
      return true;// return true when sensor is triggered
  }
  return false;
}
*/
