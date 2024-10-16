#define triggerPin 15
#define echoPin 17
float cm = 0;
float acumulado = 0;

void setUltra(){
    pinMode(triggerPin, OUTPUT); // Clear the trigger
    pinMode(echoPin, INPUT);
}

long readUltrasonicDistance()
{
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);
    return pulseIn(echoPin, HIGH);
}

cm = 0.01723 * readUltrasonicDistance();