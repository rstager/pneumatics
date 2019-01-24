#define DRIVE0_PIN 3
#define DRIVE1_PIN 2
#define DRIVE2_PIN 5
#define DRIVE3_PIN 4

void setup() {
  // put your setup code here, to run once:
  pinMode(DRIVE0_PIN, OUTPUT);
  pinMode(DRIVE1_PIN, OUTPUT);
  pinMode(DRIVE2_PIN, OUTPUT);
  pinMode(DRIVE3_PIN, OUTPUT);
  digitalWrite(DRIVE1_PIN, LOW);
  digitalWrite(DRIVE3_PIN, LOW);
}

void loop() {
  //analogWrite(DRIVE0_PIN, 125);
  // put your main code here, to run repeatedly:
  for (int fadeValue = 0 ; fadeValue <= 255; fadeValue += 5) {
    // sets the value (range from 0 to 255):
    analogWrite(DRIVE0_PIN, fadeValue);
    analogWrite(DRIVE2_PIN, fadeValue);
    // wait for 30 milliseconds to see the dimming effect
    delay(400);
  }
}
