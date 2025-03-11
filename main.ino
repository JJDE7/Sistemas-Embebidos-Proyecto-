void setup ()
{
    Serial.begin(9600);
    pinMode(11, OUTPUT);

}

void loop ()
{
    digitalWrite(11, HIGH);
    delay(1000);11, LOW);
    delay(1000);
}  