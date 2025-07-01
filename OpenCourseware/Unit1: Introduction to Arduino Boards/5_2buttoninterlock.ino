// Two-button logic:
// - LED ON  when both buttons are pressed (1 && 1)
// - LED BLINKS when only one is pressed (1 && 0 or 0 && 1)
// - LED OFF when none are pressed (0 && 0)

// === GLOBALS ===
const int buttonLeftPin  = 2;
const int buttonRightPin = 3;
const int ledPin = 13;

int buttonLeft  = 0;
int buttonRight = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);  // Wait for USB serial connection

  pinMode(buttonLeftPin, INPUT);    // Assume external pull-downs or pull-ups wired
  pinMode(buttonRightPin, INPUT);
  pinMode(ledPin, OUTPUT);

  Serial.println("Setup Complete --- Binary Button Logic");
}

void loop() {
  // === Read buttons as int (0 or 1) ===
  buttonLeft  = digitalRead(buttonLeftPin);   // returns 0 or 1
  buttonRight = digitalRead(buttonRightPin);

  // === Logic based on binary input ===
  if (buttonLeft == 1 && buttonRight == 1) {
    digitalWrite(ledPin, 1);  // ON
    Serial.println("LED ON — Both buttons pressed");
  }
  else if ((buttonLeft == 1 && buttonRight == 0) || (buttonLeft == 0 && buttonRight == 1)) {
    // Only one button pressed
    digitalWrite(ledPin, 1);
    delay(100);
    digitalWrite(ledPin, 0);
    delay(100);
    Serial.println("LED BLINK — Only one button pressed");
  }
  else {
    digitalWrite(ledPin, 0);  // OFF
    Serial.println("LED OFF — No button pressed");
  }

  delay(100); // Short delay to control Serial spam
}
