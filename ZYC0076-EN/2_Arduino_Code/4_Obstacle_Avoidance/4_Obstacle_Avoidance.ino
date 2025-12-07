#include <Servo.h>  //servo library
Servo myservo;      // create servo object to control servo
// Ultrasonic control pin
const int Trig = 12;
const int Echo = 13;
// PWM control pin
#define PWM1_PIN            5
#define PWM2_PIN            6      
// 74HCT595N Chip pins
#define SHCP_PIN            2                               // The displacement of the clock
#define EN_PIN              7                               // Can make control
#define DATA_PIN            8                               // Serial data
#define STCP_PIN            4                               // Memory register clock                  

const int Forward       = 92;                               // forward
const int Backward      = 163;                              // back
const int Turn_Left     = 149;                              // left translation
const int Turn_Right    = 106;                              // Right translation 
const int Top_Left      = 20;                               // Upper left mobile
const int Bottom_Left   = 129;                              // Lower left mobile
const int Top_Right     = 72;                               // Upper right mobile
const int Bottom_Right  = 34;                               // The lower right move
const int Stop          = 0;                                // stop
const int Contrarotate  = 172;                              // Counterclockwise rotation
const int Clockwise     = 83;                               // Rotate clockwise

int rightDistance = 0, leftDistance = 0, middleDistance = 0;

float SR04(int Trig, int Echo)
{
    digitalWrite(Trig, LOW);
    delayMicroseconds(2);
    digitalWrite(Trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(Trig, LOW);
    float distance = pulseIn(Echo, HIGH) / 58.00;
    delay(10);
    return distance;
}

void setup()
{
    myservo.attach(9,700,2400);//attach servo on pin 9 to servo object
    pinMode(Trig, OUTPUT);
    pinMode(Echo, INPUT);

    pinMode(SHCP_PIN, OUTPUT);
    pinMode(EN_PIN, OUTPUT);
    pinMode(DATA_PIN, OUTPUT);
    pinMode(STCP_PIN, OUTPUT);
    pinMode(PWM1_PIN, OUTPUT);
    pinMode(PWM2_PIN, OUTPUT);
    Serial.begin(9600);
}

void loop()
{
    middleDistance = SR04(Trig, Echo);
    Serial.print("distance:");
    Serial.println(middleDistance);
    if (middleDistance <= 25) 
    {
        Motor(Stop, 0);
        delay(500);
        myservo.write(10);
        delay(500);
        rightDistance = SR04(Trig, Echo);//SR04();

        delay(500);
        myservo.write(90);
        delay(500);
        myservo.write(170);
        delay(500);
        leftDistance = SR04(Trig, Echo);//SR04();

        delay(500);
        myservo.write(90);
        delay(500);
     
        if(rightDistance > leftDistance){
            Motor(Stop, 0);
            delay(100);
            Motor(Backward, 180);
            delay(1000);
            Motor(Clockwise, 250); 
            delay(600);
        }
        else if(rightDistance < leftDistance) {
            Motor(Stop, 0);
            delay(100);
            Motor(Backward, 180);
            delay(1000);
            Motor(Contrarotate, 250);
            delay(600);
        }
        else if((rightDistance < 20) || (leftDistance < 20)){
            Motor(Backward, 180);
            delay(1000);
            Motor(Contrarotate, 250); 
            delay(600);
        }
        else{
            Motor(Backward, 180);
            delay(1000);
            Motor(Clockwise, 250); 
            delay(600);
        }
    }
    else 
    {
        Motor(Forward, 250);
    }
}

void Motor(int Dir, int Speed)
{
    analogWrite(PWM1_PIN, Speed);
    analogWrite(PWM2_PIN, Speed);

    digitalWrite(STCP_PIN, LOW);
    shiftOut(DATA_PIN, SHCP_PIN, MSBFIRST, Dir);
    digitalWrite(STCP_PIN, HIGH);
}
