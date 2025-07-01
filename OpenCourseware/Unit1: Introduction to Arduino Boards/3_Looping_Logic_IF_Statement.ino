// Simple output over Serial: Print a loop counter continuously until count = 5

// === GLOBALS ===
int loop_count = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);  // Wait for USB Serial (important for native USB boards)

  Serial.println("Setup Complete --- Hello World!");
  delay(2000);  // Allow time to read startup message
}

void loop() {
  loop_count++;  // Increment the counter

  Serial.print("Loop repeated: ");
  Serial.print(loop_count);
  Serial.println(" times");

  // === LOGIC: Stop at 5 ===
  if (loop_count >= 5) {
    Serial.println("Reached 5 loops. Stopping now.");
    while (1);  // Infinite loop to halt execution
  }

  delay(1000);  // Delay for readability
}