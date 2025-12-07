#include    <Servo.h>

#define servo_PIN    9

Servo myservo;

void setup()
{
    myservo.attach(servo_PIN);
    myservo.write(90);
}

void loop()
{
        myservo.write(45);    
        delay(15);

        myservo.write(135);
        delay(15);
}
