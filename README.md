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
