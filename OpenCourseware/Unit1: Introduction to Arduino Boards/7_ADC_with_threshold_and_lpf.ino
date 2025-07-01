// ADC Demo + Safety Limit + LPF (Complementary Filter) — No Functions Version

// === GLOBALS ===
const int adcPin = A0;
int adcValue = 0;
float rawVoltage = 0.0;
float filteredVoltage = 0.0;
float threshold = 3.0;

const float alpha = 0.1;  // Filter responsiveness

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("ADC Demo + LPF (No Functions)");
}

void loop() {
  // === Read ADC ===
  adcValue = analogRead(adcPin);
  rawVoltage = (adcValue / 1023.0) * 3.3;  // USE * 5.0 IF ARDUINO UNO

  // === Apply complementary filter (LPF) ===
  filteredVoltage = alpha * rawVoltage + (1.0f - alpha) * filteredVoltage;

  // === Output ===
  Serial.print("ADC: ");
  Serial.print(adcValue);
  Serial.print(" | Raw V: ");
  Serial.print(rawVoltage, 3);
  Serial.print(" | Filtered V: ");
  Serial.println(filteredVoltage, 3);

  // === Threshold Check ===
  if (filteredVoltage >= threshold) {
    Serial.println("-> Threshold Exceeded");
  }

  delay(100);
}
