#define NUM_VALUES     4               // Exactly 4 data integers
#define CHECKSUM_INDEX 4               // 5th value is checksum
#define TOTAL_VALUES   5               // 4 data + 1 checksum
#define MSG_BUF_SIZE   64              // More than enough for 5 values
#define NUM_BUF_SIZE   8               // For each individual number

char messageBuffer[MSG_BUF_SIZE];     // Holds entire CSV message (no $ or \n)
char numberBuffer[NUM_BUF_SIZE];      // Builds each number
int parsedValues[TOTAL_VALUES];       // 4 data + 1 checksum

uint8_t msgIndex = 0;
bool receiving = false;

void setup() {
  Serial.begin(9600);
  Serial1.setRX(13);
  Serial1.setPollingMode(true);
  Serial1.begin(921600);
}

void loop() {
  while (Serial1.available() > 0) {
    char c = Serial1.read();

    if (c == '$') {
      receiving = true;
      msgIndex = 0;
      return;  // Don't store '$'
    }

    if (receiving) {
      if (c == '\n') {
        messageBuffer[msgIndex] = '\0';
        receiving = false;
        parseMessage();
      }
      else if (msgIndex < MSG_BUF_SIZE - 1) {
        messageBuffer[msgIndex++] = c;
      }
      else {
        // Overflow, discard
        receiving = false;
        msgIndex = 0;
      }
    }
  }
}

void parseMessage() {
  int valueCount = 0;
  int numIndex = 0;

  for (int i = 0; i <= msgIndex; i++) {
    char c = messageBuffer[i];

    if (c == ',' || c == '\0') {
      numberBuffer[numIndex] = '\0';
      parsedValues[valueCount++] = atoi(numberBuffer);
      numIndex = 0;
    }
    else if (numIndex < NUM_BUF_SIZE - 1) {
      numberBuffer[numIndex++] = c;
    }
  }

  // Require exactly 5 values (4 + checksum)
  if (valueCount != TOTAL_VALUES) {
    Serial.println("Invalid frame: wrong number of values");
    return;
  }

  // Compute checksum
  int sum = 0;
  for (int i = 0; i < NUM_VALUES; i++) {
    sum += parsedValues[i];
  }

  if (sum != parsedValues[CHECKSUM_INDEX]) {
    Serial.print("Checksum Fail! Your UART Debugger ran into a problem. FREEZING.");
    delay(5000000);
  }

  // If valid, print parsed data
  Serial.print("Parsed: ");
  for (int i = 0; i < NUM_VALUES; i++) {
    Serial.print(parsedValues[i]);
    if (i < NUM_VALUES - 1) {
      Serial.print(", ");
    } else {
      Serial.println();
    }
  }
}
