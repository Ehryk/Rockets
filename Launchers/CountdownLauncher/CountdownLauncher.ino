
int RELAYPIN = 5;
int LEDPIN = 0;

void setup() {
  // Initialize the relay pin as output
  pinMode(RELAYPIN, OUTPUT);
  // Initialize the LED pin as output
  pinMode(LEDPIN, OUTPUT);
  delay(1000);
  countdown();
}

void loop() {
}

void countdown() {
  blink();
  delay(1000);
  blink();
  delay(1000);
  blink();
  delay(1000);
  launch();
}

void blink() {
    digitalWrite(LEDPIN, HIGH);   // Turn the relay on
    delay(200);                   // Wait for a blink
    digitalWrite(LEDPIN, LOW);    // Turn the relay off
}

void launch() {
    digitalWrite(RELAYPIN, HIGH);   // Turn the relay on
    delay(1000);                    // Wait for a second
    digitalWrite(RELAYPIN, LOW);    // Turn the relay off

}
