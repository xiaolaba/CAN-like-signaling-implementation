# CAN-like-signaling-implementation
Here's Arduino C code that implements CAN-like signaling to send "hello" at 19200 baud N81  

### goal  
arduino c code  
CANH and CANL are D7 D8 pin mapping,  
init, both IO line as input, disable pullup, it is idle state or logic "1"  
CANH set output to 0v, and CANL set output to 5v at the same time, this is active state or logic "0"  
imitate serial transimission to send string "hello" via such IO lines. 19200 baud N81.  


```
// CAN-like signaling implementation, xiaolaba, 2025-09-01
#define CANH_PIN 7
#define CANL_PIN 8

void setup() {
  // Initialize both pins as inputs (idle state - logic "1")
  pinMode(CANH_PIN, INPUT);
  pinMode(CANL_PIN, INPUT);
  digitalWrite(CANH_PIN, LOW);  // Disable pullup
  digitalWrite(CANL_PIN, LOW);  // Disable pullup
  
  // Initialize serial for debugging
  Serial.begin(115200);
}

void setCANState(bool state) {
  if (state) {
    // Logic "1" - Idle state (both inputs)
    pinMode(CANH_PIN, INPUT);
    pinMode(CANL_PIN, INPUT);
  } else {
    // Logic "0" - Active state (CANH=0V, CANL=5V)
    pinMode(CANH_PIN, OUTPUT);
    pinMode(CANL_PIN, OUTPUT);
    digitalWrite(CANH_PIN, LOW);
    digitalWrite(CANL_PIN, HIGH);
  }
}

void sendCANByte(uint8_t data) {
  // Calculate bit time in microseconds for 19200 baud
  const unsigned long bitTime = 1000000UL / 19200; // ~52.08 microseconds
  
  // Start bit (logic "0")
  setCANState(false);
  delayMicroseconds(bitTime);
  
  // Send 8 data bits (LSB first)
  for (int i = 0; i < 8; i++) {
    bool bitState = (data >> i) & 0x01;
    setCANState(bitState);
    delayMicroseconds(bitTime);
  }
  
  // Stop bit (logic "1")
  setCANState(true);
  delayMicroseconds(bitTime);
}

void sendCANString(const char* str) {
  while (*str) {
    sendCANByte(*str++);
  }
}

void loop() {
  // Send "hello" every 2 seconds
  sendCANString("hello");
  Serial.println("Sent: hello");
  delay(2000);
}
```

### Key features of this implementation:

    Initialization: Both CANH (D7) and CANL (D8) are set as inputs with pullups disabled, representing the idle state (logic "1")

    CAN Signaling:

        Logic "1": Both pins as inputs (idle)

        Logic "0": CANH=OUTPUT LOW (0V), CANL=OUTPUT HIGH (5V)

    19200 baud N81:  
        No parity (N)  
        8 data bits  
        1 stop bit  
        Bit timing: ~52.08μs per bit  
    Transmission Protocol:  
        Start bit: logic "0"  
        8 data bits (LSB first)  
        Stop bit: logic "1"  
    String Transmission: The sendCANString() function sends each character of "hello" sequentially  

To use this code:  
    Connect CANH to pin D7 and CANL to pin D8  
    Upload the code to your Arduino  
    The message "hello" will be sent every 2 seconds  
    Serial monitor will show confirmation messages  

Note: This implements the electrical signaling you described, but it's not actual CAN protocol (which includes arbitration, error checking, etc.). It's a simple serial-like transmission using CAN-like voltage levels.  




### complete Arduino C code with both transmission and reception routines  
```
// CAN-like signaling implementation with TX and RX
#define CANH_PIN 7
#define CANL_PIN 8

// Buffer for received data
#define RX_BUFFER_SIZE 32
char rxBuffer[RX_BUFFER_SIZE];
uint8_t rxIndex = 0;
bool receiving = false;
unsigned long lastBitTime = 0;

void setup() {
  // Initialize both pins as inputs (idle state - logic "1")
  pinMode(CANH_PIN, INPUT);
  pinMode(CANL_PIN, INPUT);
  digitalWrite(CANH_PIN, LOW);  // Disable pullup
  digitalWrite(CANL_PIN, LOW);  // Disable pullup
  
  // Initialize serial for debugging
  Serial.begin(115200);
  Serial.println("CAN-like communication ready");
  
  // Attach interrupt for reception (on CANH pin change)
  attachInterrupt(digitalPinToInterrupt(CANH_PIN), canPinChange, CHANGE);
}

void setCANState(bool state) {
  if (state) {
    // Logic "1" - Idle state (both inputs)
    pinMode(CANH_PIN, INPUT);
    pinMode(CANL_PIN, INPUT);
  } else {
    // Logic "0" - Active state (CANH=0V, CANL=5V)
    pinMode(CANH_PIN, OUTPUT);
    pinMode(CANL_PIN, OUTPUT);
    digitalWrite(CANH_PIN, LOW);
    digitalWrite(CANL_PIN, HIGH);
  }
}

bool readCANState() {
  // Read the current state of CAN lines
  // Returns true for logic "1" (idle), false for logic "0" (active)
  int canhState = digitalRead(CANH_PIN);
  int canlState = digitalRead(CANL_PIN);
  
  // If both are inputs (high impedance), it's logic "1"
  // If CANH is LOW and CANL is HIGH, it's logic "0"
  return (canhState == LOW && canlState == LOW); // Both inputs = logic "1"
}

void sendCANByte(uint8_t data) {
  noInterrupts(); // Disable interrupts during transmission
  // Calculate bit time in microseconds for 19200 baud
  const unsigned long bitTime = 1000000UL / 19200; // ~52.08 microseconds
  
  // Start bit (logic "0")
  setCANState(false);
  delayMicroseconds(bitTime);
  
  // Send 8 data bits (LSB first)
  for (int i = 0; i < 8; i++) {
    bool bitState = (data >> i) & 0x01;
    setCANState(bitState);
    delayMicroseconds(bitTime);
  }
  
  // Stop bit (logic "1")
  setCANState(true);
  delayMicroseconds(bitTime);
  interrupts(); // Re-enable interrupts
}

void sendCANString(const char* str) {
  while (*str) {
    sendCANByte(*str++);
  }
}

void canPinChange() {
  unsigned long currentTime = micros();
  bool currentState = readCANState();
  
  if (!receiving) {
    // Look for start bit (transition to logic "0")
    if (!currentState) {
      receiving = true;
      rxIndex = 0;
      rxBuffer[0] = 0;
      lastBitTime = currentTime + (1000000UL / 19200) * 1.5; // Sample in middle of bit
    }
  } else {
    // Check if it's time to sample a bit
    if (currentTime >= lastBitTime) {
      sampleBit(currentState);
      lastBitTime += (1000000UL / 19200); // Next bit time
    }
  }
}

void sampleBit(bool bitState) {
  static uint8_t bitCount = 0;
  static uint8_t receivedByte = 0;
  
  if (bitCount == 0) {
    // This should be the start bit (logic "0")
    if (bitState) {
      // Error - expected start bit, got logic "1"
      receiving = false;
      return;
    }
    bitCount++;
  } else if (bitCount <= 8) {
    // Data bits (LSB first)
    receivedByte |= (bitState ? 1 : 0) << (bitCount - 1);
    bitCount++;
  } else {
    // Stop bit (should be logic "1")
    if (bitState) {
      // Valid stop bit received
      if (rxIndex < RX_BUFFER_SIZE - 1) {
        rxBuffer[rxIndex++] = receivedByte;
        rxBuffer[rxIndex] = 0; // Null terminate
      }
    }
    // Reset for next byte
    receiving = false;
    bitCount = 0;
    receivedByte = 0;
  }
}

void checkReceivedData() {
  if (rxIndex > 0) {
    Serial.print("Received: ");
    Serial.println(rxBuffer);
    
    // Echo back received data
    Serial.print("Echoing: ");
    Serial.println(rxBuffer);
    sendCANString(rxBuffer);
    
    // Clear buffer
    rxIndex = 0;
    rxBuffer[0] = 0;
  }
}

void loop() {
  // Check for received data
  checkReceivedData();
  
  // Send "hello" every 5 seconds if not receiving
  static unsigned long lastSendTime = 0;
  if (millis() - lastSendTime >= 5000 && !receiving) {
    sendCANString("hello");
    Serial.println("Sent: hello");
    lastSendTime = millis();
  }
  
  // Handle timeout for reception
  if (receiving && micros() - lastBitTime > 2000000UL / 19200) {
    // Timeout - reset reception state
    receiving = false;
    Serial.println("Reception timeout");
  }
}



```


### send every 100ms, after sending, listen to until 100ms times up

```

// CAN-like signaling implementation with timed TX/RX
#define CANH_PIN 7
#define CANL_PIN 8

// Buffer for received data
#define RX_BUFFER_SIZE 32
char rxBuffer[RX_BUFFER_SIZE];
uint8_t rxIndex = 0;
bool receiving = false;
unsigned long lastBitTime = 0;
unsigned long transmitStartTime = 0;

void setup() {
  // Initialize both pins as inputs (idle state - logic "1")
  pinMode(CANH_PIN, INPUT);
  pinMode(CANL_PIN, INPUT);
  digitalWrite(CANH_PIN, LOW);  // Disable pullup
  digitalWrite(CANL_PIN, LOW);  // Disable pullup
  
  // Initialize serial for debugging
  Serial.begin(115200);
  Serial.println("CAN-like communication ready - 100ms cycle");
  
  // Attach interrupt for reception (on CANH pin change)
  attachInterrupt(digitalPinToInterrupt(CANH_PIN), canPinChange, CHANGE);
}

void setCANState(bool state) {
  if (state) {
    // Logic "1" - Idle state (both inputs)
    pinMode(CANH_PIN, INPUT);
    pinMode(CANL_PIN, INPUT);
  } else {
    // Logic "0" - Active state (CANH=0V, CANL=5V)
    pinMode(CANH_PIN, OUTPUT);
    pinMode(CANL_PIN, OUTPUT);
    digitalWrite(CANH_PIN, LOW);
    digitalWrite(CANL_PIN, HIGH);
  }
}

bool readCANState() {
  // Read the current state of CAN lines
  // Returns true for logic "1" (idle), false for logic "0" (active)
  int canhState = digitalRead(CANH_PIN);
  int canlState = digitalRead(CANL_PIN);
  
  // If both are inputs (high impedance), it's logic "1"
  // If CANH is LOW and CANL is HIGH, it's logic "0"
  return (canhState == LOW && canlState == LOW); // Both inputs = logic "1"
}

void sendCANByte(uint8_t data) {
  // Calculate bit time in microseconds for 19200 baud
  const unsigned long bitTime = 1000000UL / 19200; // ~52.08 microseconds
  
  // Start bit (logic "0")
  setCANState(false);
  delayMicroseconds(bitTime);
  
  // Send 8 data bits (LSB first)
  for (int i = 0; i < 8; i++) {
    bool bitState = (data >> i) & 0x01;
    setCANState(bitState);
    delayMicroseconds(bitTime);
  }
  
  // Stop bit (logic "1")
  setCANState(true);
  delayMicroseconds(bitTime);
}

void sendCANString(const char* str) {
  noInterrupts(); // Disable interrupts during transmission
  while (*str) {
    sendCANByte(*str++);
  }
  interrupts(); // Re-enable interrupts
}

void canPinChange() {
  if (receiving) { // Only process interrupts during reception period
    unsigned long currentTime = micros();
    bool currentState = readCANState();
    
    if (!receiving) {
      // Look for start bit (transition to logic "0")
      if (!currentState) {
        receiving = true;
        rxIndex = 0;
        rxBuffer[0] = 0;
        lastBitTime = currentTime + (1000000UL / 19200) * 1.5; // Sample in middle of bit
      }
    } else {
      // Check if it's time to sample a bit
      if (currentTime >= lastBitTime) {
        sampleBit(currentState);
        lastBitTime += (1000000UL / 19200); // Next bit time
      }
    }
  }
}

void sampleBit(bool bitState) {
  static uint8_t bitCount = 0;
  static uint8_t receivedByte = 0;
  
  if (bitCount == 0) {
    // This should be the start bit (logic "0")
    if (bitState) {
      // Error - expected start bit, got logic "1"
      receiving = false;
      return;
    }
    bitCount++;
  } else if (bitCount <= 8) {
    // Data bits (LSB first)
    receivedByte |= (bitState ? 1 : 0) << (bitCount - 1);
    bitCount++;
  } else {
    // Stop bit (should be logic "1")
    if (bitState) {
      // Valid stop bit received
      if (rxIndex < RX_BUFFER_SIZE - 1) {
        rxBuffer[rxIndex++] = receivedByte;
        rxBuffer[rxIndex] = 0; // Null terminate
        Serial.print("Received char: ");
        Serial.println((char)receivedByte);
      }
    }
    // Reset for next byte
    receiving = false;
    bitCount = 0;
    receivedByte = 0;
  }
}

void processReceivedData() {
  if (rxIndex > 0) {
    Serial.print("Complete message: ");
    Serial.println(rxBuffer);
    
    // Clear buffer
    rxIndex = 0;
    rxBuffer[0] = 0;
  }
}

void loop() {
  static unsigned long lastCycleTime = 0;
  unsigned long currentTime = millis();
  
  // 100ms cycle: transmit then receive
  if (currentTime - lastCycleTime >= 100) {
    lastCycleTime = currentTime;
    
    // Phase 1: Transmit "hello" (takes ~5.2ms)
    Serial.println("--- TRANSMIT PHASE ---");
    transmitStartTime = micros();
    sendCANString("hello");
    unsigned long transmitDuration = micros() - transmitStartTime;
    Serial.print("Transmit took: ");
    Serial.print(transmitDuration / 1000.0, 2);
    Serial.println("ms");
    
    // Phase 2: Receive for remaining time (until 100ms elapses)
    Serial.println("--- RECEIVE PHASE ---");
    receiving = true; // Enable reception
    rxIndex = 0;      // Clear receive buffer
    rxBuffer[0] = 0;
    
    // Listen until 100ms cycle completes
    unsigned long receiveStartTime = millis();
    while (millis() - lastCycleTime < 100) {
      // Process any completed messages immediately
      if (rxIndex > 0) {
        processReceivedData();
      }
      
      // Handle reception timeout
      if (receiving && micros() - lastBitTime > 2000000UL / 19200) {
        receiving = false;
        Serial.println("Bit timeout, waiting for new start bit");
      }
      
      // Small delay to prevent busyloop
      delayMicroseconds(100);
    }
    
    receiving = false; // Disable reception after cycle
    Serial.println("--- CYCLE COMPLETE ---");
    
    // Process any remaining received data
    if (rxIndex > 0) {
      processReceivedData();
    }
  }
}
```
