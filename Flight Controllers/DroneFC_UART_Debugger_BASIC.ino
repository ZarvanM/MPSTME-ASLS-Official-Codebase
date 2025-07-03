#define BUF_SIZE     128        // Size of UART input buffer
#define MAX_VALUES   4          // Max number of integers to parse
#define NUMBUF_SIZE  8          // Max characters in one number (e.g. "-1502")

char messageBuffer[BUF_SIZE];           // Holds the incoming message (excluding $ and \n)
char numberBuffer[NUMBUF_SIZE];         // Holds characters for one number at a time
int parsedValues[MAX_VALUES];           // Final array of parsed integers

uint8_t messageIndex = 0;               // Current write position in messageBuffer
bool receivingMessage = false;          // Are we currently inside a $...<newline> frame?

void setup() {
  Serial.begin(9600);
  Serial1.setRX(13);                    // Set RX pin for UART
  Serial1.setPollingMode(true);        // Use polling instead of interrupt
  Serial1.begin(921600);               // High-speed UART for input
}

void loop() {
  while (Serial1.available() > 0) {
    char incoming = Serial1.read();

    // Start of message
    if (!receivingMessage && incoming == '$') {
      receivingMessage = true;
      messageIndex = 0;                // Reset buffer
    }

    // If receiving, build up the message
    if (receivingMessage) {
      if (incoming == '\n') {
        // End of message
        messageBuffer[messageIndex] = '\0';  // Null-terminate for safety
        receivingMessage = false;
        parseMessage();                      // Extract values from the message
      } 
      else if (messageIndex < BUF_SIZE - 1) {
        messageBuffer[messageIndex++] = incoming;
      } 
      else {
        // Buffer overflow — discard message
        receivingMessage = false;
        messageIndex = 0;
      }
    }
  }
}

void parseMessage() {
  int numberIndex = 0;      // Index in numberBuffer
  int valueCount = 0;       // Number of values stored in parsedValues[]

  for (int i = 0; i <= messageIndex; i++) {
    char c = messageBuffer[i];

    // When we hit a comma or end of string, convert numberBuffer
    if (c == ',' || c == '\0') {
      numberBuffer[numberIndex] = '\0';  // Terminate current number string

      if (valueCount < MAX_VALUES) {
        parsedValues[valueCount] = atoi(numberBuffer);  // Convert to int
        valueCount++;
      }

      numberIndex = 0;  // Reset for next number
    }
    else if (numberIndex < NUMBUF_SIZE - 1) {
      numberBuffer[numberIndex] = c;
      numberIndex++;
    }
  }

  // Print parsed values
  Serial.print("Parsed: ");
  for (int i = 0; i < valueCount; i++) {
    Serial.print(parsedValues[i]);

    if (i < valueCount - 1) {
      Serial.print(", ");
    } else {
      Serial.println();
    }
  }
}
