// Simple output over Serial: Print a loop counter continuously.

// === GLOBALS ===
int loop_count = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);  // Wait for USB Serial to be ready (on boards like Leonardo or RP2040)

  Serial.println("Setup Complete --- Hello World!");
  delay(2000);  // Blocking delay to let you read the setup message
}

void loop() {
  loop_count++;  // Same as: loop_count = loop_count + 1;

  Serial.print("Loop repeated: ");
  Serial.print(loop_count);
  Serial.println(" times");

  delay(1000);  // Blocking delay: slows loop for human-readable output
}