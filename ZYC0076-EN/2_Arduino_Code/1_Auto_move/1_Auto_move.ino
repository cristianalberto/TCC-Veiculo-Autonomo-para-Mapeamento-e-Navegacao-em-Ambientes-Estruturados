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

void setup()
{
    pinMode(SHCP_PIN, OUTPUT);
    pinMode(EN_PIN, OUTPUT);
    pinMode(DATA_PIN, OUTPUT);
    pinMode(STCP_PIN, OUTPUT);
    pinMode(PWM1_PIN, OUTPUT);
    pinMode(PWM2_PIN, OUTPUT);
}

void loop()
{
    /* Forward */
    Motor(Forward, 250);     
    delay(2000);
    /* Backward */
    Motor(Backward, 250);
    delay(2000);
    /* Turn_Left */
    Motor(Turn_Left, 250);
    delay(2000);
    /* Turn_Right */
    Motor(Turn_Right, 250);
    delay(2000);
    /* Top_Left */
    Motor(Top_Left, 250);
    delay(2000);
    /* Bottom_Right */
    Motor(Bottom_Right, 250);
    delay(2000);
    /* Bottom_Left */
    Motor(Bottom_Left, 250);
    delay(2000);
    /* Top_Right */
    Motor(Top_Right, 250);
    delay(2000);
    /* Clockwise */
    Motor(Clockwise, 250);
    delay(2000);
    /* Contrarotate */
    Motor(Contrarotate, 250);
    delay(2000);
    /* Stop */
    Motor(Stop, 250);
    delay(2000);
}

void Motor(int Dir, int Speed)
{
    digitalWrite(EN_PIN, LOW);
    analogWrite(PWM1_PIN, Speed);
    analogWrite(PWM2_PIN, Speed);

    digitalWrite(STCP_PIN, LOW);
    shiftOut(DATA_PIN, SHCP_PIN, MSBFIRST, Dir);
    digitalWrite(STCP_PIN, HIGH);
}