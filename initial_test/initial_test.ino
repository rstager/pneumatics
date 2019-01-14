#define DRIVE0_PIN 3
#define DRIVE1_PIN A1
#define DRIVE2_PIN A2
#define DRIVE3_PIN A3

void setup() {
  // put your setup code here, to run once:
//  pinMode(DRIVE0_PIN, OUTPUT);
  pinMode(DRIVE1_PIN, OUTPUT);
  pinMode(DRIVE2_PIN, OUTPUT);
  pinMode(DRIVE3_PIN, OUTPUT);
  digitalWrite(DRIVE1_PIN, LOW);
  digitalWrite(DRIVE2_PIN, LOW);
  digitalWrite(DRIVE3_PIN, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  for (int fadeValue = 0 ; fadeValue <= 255; fadeValue += 5) {
    // sets the value (range from 0 to 255):
    analogWrite(DRIVE0_PIN, fadeValue);
    // wait for 30 milliseconds to see the dimming effect
    delay(400);
//  digitalWrite(DRIVE0_PIN, HIGH);
//  delay(20);
//  digitalWrite(DRIVE0_PIN, LOW);
//  delay(20);
  }
}
