/* Tester of the functionality of position sensor.
 * Color Code  :
 * Brow  :  +5V
 * Blue  :  GND
 * black :  return 1 when triggered, else return 0
 * white :  return 0 when triggered, else return 1
 *
 * written   : Su Gao
 * last edite: 23-AUG-2015
 */

int sensor = 8; // Gabellichtschrank.

void setup()
{
  Serial.begin(115200);
  Serial.println("I started");
  pinMode(sensor,INPUT);  
}

void loop()
{
  boolean s = digitalRead(sensor);
  Serial.println(s);
  delay(50);

}
