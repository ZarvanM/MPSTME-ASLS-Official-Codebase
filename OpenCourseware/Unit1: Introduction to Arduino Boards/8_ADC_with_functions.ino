// ADC Demo + Safety Limit + LPF (Complementary Filter) with good C++ structure

// === GLOBALS ===
const int adcPin = A0;
int adcValue = 0;
float rawVoltage = 0.0;
float filteredVoltage = 0.0;
float threshold = 3.0;

const float alpha = 0.1;  // Filter responsiveness

// === Function Declarations ===
float lowPassFilter(float input, float previous, float alpha);
void checkThreshold(float value, float limit);

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("ADC Demo + LPF Function");
}

void loop() {
  adcValue = analogRead(adcPin);
  rawVoltage = (adcValue / 1023.0) * 3.3;  // USE * 5.0 IF ARDUINO UNO

  // Apply filter
  filteredVoltage = lowPassFilter(rawVoltage, filteredVoltage, alpha);

  // Output measurements
  Serial.print("ADC: ");
  Serial.print(adcValue);
  Serial.print(" | Raw V: ");
  Serial.print(rawVoltage, 3);
  Serial.print(" | Filtered V: ");
  Serial.println(filteredVoltage, 3);

  // Call threshold checker
  checkThreshold(filteredVoltage, threshold);

  delay(100);
}

// === Function Definitions ===
float lowPassFilter(float input, float previous, float alpha) {
  return alpha * input + (1.0f - alpha) * previous;
}

void checkThreshold(float value, float limit) {
  if (value >= limit) {
    Serial.println("-> Threshold Exceeded");
  }
}
