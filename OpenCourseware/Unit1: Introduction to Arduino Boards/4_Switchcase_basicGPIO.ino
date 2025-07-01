// Simple Switch-Case Example: Mode cycles through 1, 2, 3 in each loop

// === GLOBALS ===
int mode = 1;

void setup() {
  Serial.begin(9600);
  while (!Serial);  // Wait for USB serial (native USB boards)

  Serial.println("Setup Complete --- Switch Case Cycling 1-2-3");
  delay(1000);
}

void loop() {
  // === SWITCH-CASE BASED ON MODE ===
  switch (mode) {
    case 1:
      Serial.println("Mode 1: LED OFF");
      // digitalWrite(13, LOW); // optional hardware control
      break;

    case 2:
      Serial.println("Mode 2: LED ON");
      // digitalWrite(13, HIGH);
      break;

    case 3:
      Serial.println("Mode 3: LED BLINK");
      // digitalWrite(13, HIGH);
      // delay(100);
      // digitalWrite(13, LOW);
      // delay(100);
      break;
  }

  // === CYCLE MODE FROM 1 TO 3 ===
  mode = mode + 1;
  if (mode > 3) 
  { 
    mode = 1;
  }
  delay(1000); // Slow down the loop
}
