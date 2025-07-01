//Simple output over Serial, Print Once only.

void setup() {
Serial.begin(9600);
while (!Serial);

Serial.println("Setup Complete --- Hello World!");
}

void loop() {
  // put your main code here, to run repeatedly:

}
