// Schedule 3 separate tasks at different frequencies using micros()

// === CONFIG: Task Frequencies (in microseconds) ===
const unsigned long intervalA = 200000;   // 0.2s = 5Hz
const unsigned long intervalB = 500000;   // 0.5s = 2Hz
const unsigned long intervalC = 1000000;  // 1s   = 1Hz

// === STATE: Last Execution Timestamps ===
unsigned long lastA = 0;
unsigned long lastB = 0;
unsigned long lastC = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("micros() Task Scheduler Demo");
}

void loop() {
  unsigned long now = micros();

  // Task A: 5Hz
  if ((unsigned long)(now - lastA) >= intervalA) {
    lastA = now;
    taskA();
  }

  // Task B: 2Hz
  if ((unsigned long)(now - lastB) >= intervalB) {
    lastB = now;
    taskB();
  }

  // Task C: 1Hz
  if ((unsigned long)(now - lastC) >= intervalC) {
    lastC = now;
    taskC();
  }

}

// === TASK FUNCTIONS ===
void taskA() {
  Serial.println("Task A called (5Hz)");
}

void taskB() {
  Serial.println("Task B called (2Hz)");
}

void taskC() {
  Serial.println("Task C called (1Hz)");
}
