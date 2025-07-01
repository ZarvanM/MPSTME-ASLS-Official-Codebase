// ADC Demo + Safety Limit
// Reads analog voltage and converts to actual voltage

const int adcPin = A0;  // ADC input pin
int adcValue = 0;
float voltage = 0.0;
float threshold = 3.0;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("ADC Demo Started");
}

void loop() {
  adcValue = analogRead(adcPin);  // 0–1023 for 10-bit ADC on UNO & RP2040 DEFAULT (2^10 unique levels)
  voltage = (adcValue / 1023.0) * 3.3;  // Convert to voltage (V) for 5V=Arduino UNO ; 3.3V for RP2040 boards

  Serial.print("ADC Raw: ");
  Serial.print(adcValue);
  Serial.print(" | Voltage: ");
  Serial.print(voltage, 3);  // Show 3 decimal places
  Serial.println(" V |");

if(voltage>=threshold){
  Serial.println("Pre-Set Threshold Reached");  // warn
}

  delay(500);
}