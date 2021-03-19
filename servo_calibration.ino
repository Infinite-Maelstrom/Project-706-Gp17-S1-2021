#include <Servo.h>

//Initialise variables
Servo fan_rotator;
int pos = 0;

void setup()
{
  pinMode(13, OUTPUT);
  fan_rotator.attach(9);
}


//Rotate servo to three positions
void loop()
{
  fan_rotator.write(pos);
  pos = (pos + 90) % 270;

  //This bit left over from the default arduino code
  digitalWrite(13, HIGH);
  delay(1000); // Wait for 1000 millisecond(s)
  digitalWrite(13, LOW);
  delay(1000); // Wait for 1000 millisecond(s)
}
