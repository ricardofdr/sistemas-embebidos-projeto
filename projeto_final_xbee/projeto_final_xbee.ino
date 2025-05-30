#include <SPI.h>
#include <MFRC522.h>

#define BAUD_RATE 9600
#define BAUD_RATE_DIVISOR (F_CPU / 16 / BAUD_RATE - 1)

MFRC522 mfrc522(10, 9);

#define GATE_TRIG_DDR DDRD
#define GATE_TRIG_PORT PORTD
#define GATE_TRIG_BIT PORTD7
#define GATE_ECHO_PIN PINB
#define GATE_ECHO_BIT PINB0

#define PARK1_TRIG_PIN A0
#define PARK1_ECHO_PIN A1
#define PARK2_TRIG_PIN A2
#define PARK2_ECHO_PIN A3
#define PARK3_TRIG_PIN A4
#define PARK3_ECHO_PIN A5

#define GREEN_LED PD6
#define YELLOW1_LED PD4
#define YELLOW2_LED PD5
#define YELLOW3_LED PD2

const byte frameStartByte = 0x7E;
const byte frameTypeTXrequest = 0x10;
const byte frameTypeRXpacket = 0x90;
//const byte frameTypeATcommand = 0x08;
const byte frameTypeATresponse = 0x88;
const long destAddressHigh = 0x13A200;
// const long destAddressLow = 0x4098229C;
const long destAddressLow = 0x40A099E8;
char DBcommand[] = "DB";

int sensorPin = 0;   //the analog pin the TMP36's Vout (sense) pin is connected to
byte ATcounter = 0;  // for identifying current AT command frame
byte rssi = 0;       // RSSI value of last received packet

bool gateOpen = false;
bool carPresent = false;
bool waitingForCarEntry = false;
unsigned long lastRFIDRead = 0;
unsigned long lastGateClose = 0;
unsigned long lastParkingCheck = 0;
const long rfidCooldown = 2000;
const long gateCloseDelay = 3000;
const long parkingCheckInterval = 500;
const int gateDistanceThreshold = 10;
const int parkDistanceThreshold = 20;
volatile bool openingGate = false;
volatile bool closingGate = false;

unsigned long lastStatusSendTime = 0;
const long statusSendInterval = 500; // Send status every 500ms

volatile bool park1_occupied = false;
volatile bool park2_occupied = false;
volatile bool park3_occupied = false;
volatile int total = 0;

// For defining bit positions in the status byte (optional but good practice)
const byte SLOT1_BIT = 0;
const byte SLOT2_BIT = 1;
const byte SLOT3_BIT = 2;
const byte GATE_OPEN_BIT = 3;
const byte GATE_CLOSED_BIT = 4;
const byte GATE_OPENING_BIT = 5;
const byte GATE_CLOSING_BIT = 6;

void USART_init(void) {
  UCSR0A = 0;
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
  UBRR0 = BAUD_RATE_DIVISOR;
}

void USART_transmit(char data) {
  while (!(UCSR0A & (1 << UDRE0)))
    ;
  UDR0 = data;
}

void USART_transmit_string(const char* str) {
  while (*str) {
    USART_transmit(*str++);
  }
}

void setGateState(bool open) {
  if (open && !gateOpen) {
    USART_transmit_string("Opening gate: OCR2B = 62\n");
    openingGate = true;
    OCR2B = 62;
    gateOpen = true;
    closingGate = false;
  } else if (!open && gateOpen) {
    USART_transmit_string("Closing gate: OCR2B = 12\n");
    closingGate = true;
    OCR2B = 12;
    gateOpen = false;
    openingGate = false;
  }
}

void updateLEDs() {
  if (total < 3) {
    PORTD |= (1 << GREEN_LED);
    USART_transmit_string("Parking available.\n");
  } else {
    PORTD &= ~(1 << GREEN_LED);
    USART_transmit_string("Parking full.\n");
  }
  PORTD = (PORTD & ~((1 << YELLOW1_LED) | (1 << YELLOW2_LED) | (1 << YELLOW3_LED))) | (park1_occupied << YELLOW1_LED) | (park2_occupied << YELLOW2_LED) | (park3_occupied << YELLOW3_LED);
}

void setup() {
  SPI.begin();
  mfrc522.PCD_Init();

  USART_init();
  USART_transmit_string("System initialized.\n");

  // Adicionar pin output para XBee

  GATE_TRIG_DDR |= (1 << GATE_TRIG_BIT);
  GATE_TRIG_PORT &= ~(1 << GATE_TRIG_BIT);
  DDRB &= ~(1 << DDB0);

  DDRC |= (1 << DDC0) | (1 << DDC2) | (1 << DDC4);
  DDRC &= ~((1 << DDC1) | (1 << DDC3) | (1 << DDC5));
  PORTC &= ~((1 << PC0) | (1 << PC2) | (1 << PC4));

  DDRD |= (1 << DDD3);
  DDRD |= (1 << GREEN_LED) | (1 << YELLOW1_LED) | (1 << YELLOW2_LED) | (1 << YELLOW3_LED);
  PORTD &= ~((1 << YELLOW1_LED) | (1 << YELLOW2_LED) | (1 << YELLOW3_LED));
  PORTD |= (1 << GREEN_LED);

  TCCR2A = (1 << COM2B1) | (0 << COM2B0) | (1 << WGM20) | (0 << WGM21);
  TCCR2B = (1 << WGM22) | (1 << CS22) | (0 << CS21) | (0 << CS20);
  OCR2A = 249;
  OCR2B = 12;
  USART_transmit_string("Servo initialized: OCR2B = 12 (closed)\n");

  updateLEDs();
}

void loop() {
  unsigned long currentMillis = millis();
  
  check_if_remote_control();

  if (currentMillis - lastParkingCheck >= parkingCheckInterval) {
    checkParkingSlots();
    lastParkingCheck = currentMillis;
  }

  // Send status at its own interval
  if (currentMillis - lastStatusSendTime >= statusSendInterval) {
    create_and_send_park_status();
    lastStatusSendTime = currentMillis;
  }
  
  if (currentMillis - lastRFIDRead >= rfidCooldown) {
    create_and_send_park_status();
    if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
      String uid = "";
      for (byte i = 0; i < mfrc522.uid.size; i++) {
        char buf[3];
        sprintf(buf, "%02X", mfrc522.uid.uidByte[i]);
        uid += buf;
      }
      uid.toUpperCase();

      USART_transmit_string("UID: ");
      USART_transmit_string(uid.c_str());
      USART_transmit_string("\n");

      if (uid == "9D227789" && total < 3) {
        USART_transmit_string("Access granted (entry). Opening gate.\n");
        setGateState(true);
        waitingForCarEntry = true;
        carPresent = false;
      } else {
        USART_transmit_string(total >= 3 ? "Access denied: Parking full.\n" : "Access denied: Invalid card.\n");
        if (!waitingForCarEntry && !carPresent) {
          setGateState(false);
        }
      }


      lastRFIDRead = currentMillis;
    }
  }

  if (currentMillis - lastParkingCheck >= parkingCheckInterval) {
    checkParkingSlots();
    lastParkingCheck = currentMillis;
    updateLEDs();
  }

  unsigned long dist = readUltrasonicDistance();

  char distStr[32];
  sprintf(distStr, "Gate distance: %lu cm\n", dist);
  USART_transmit_string(distStr);

  if (dist < gateDistanceThreshold && dist != 0) {
    if (!carPresent) {
      USART_transmit_string("Car detected (entry/leaving). Opening gate.\n");
      carPresent = true;
      if (waitingForCarEntry) {
        total = min(total + 1, 3);
        char totalStr[32];
        sprintf(totalStr, "Car entering. Total cars: %d\n", total);
        USART_transmit_string(totalStr);
        updateLEDs();
      }
    }
    setGateState(true);
    lastGateClose = currentMillis;
  } else if (carPresent && currentMillis - lastGateClose >= gateCloseDelay) {
    USART_transmit_string("Car left. Closing gate.\n");
    setGateState(false);
    if (!waitingForCarEntry) {
      total = max(total - 1, 0);
      char totalStr[32];
      sprintf(totalStr, "Car exited. Total cars: %d\n", total);
      USART_transmit_string(totalStr);
      updateLEDs();
    }
    waitingForCarEntry = false;
    carPresent = false;
    lastGateClose = currentMillis;
  }
}

void checkParkingSlots() {
  int dist1 = measureDistance(PARK1_TRIG_PIN, PARK1_ECHO_PIN);
  park1_occupied = (dist1 < parkDistanceThreshold && dist1 > 0);
  char distStr1[32];
  sprintf(distStr1, "Park 1: %s (%d cm)\n", park1_occupied ? "occupied" : "free", dist1);
  USART_transmit_string(distStr1);

  int dist2 = measureDistance(PARK2_TRIG_PIN, PARK2_ECHO_PIN);
  park2_occupied = (dist2 < parkDistanceThreshold && dist2 > 0);
  char distStr2[32];
  sprintf(distStr2, "Park 2: %s (%d cm)\n", park2_occupied ? "occupied" : "free", dist2);
  USART_transmit_string(distStr2);

  int dist3 = measureDistance(PARK3_TRIG_PIN, PARK3_ECHO_PIN);
  park3_occupied = (dist3 < parkDistanceThreshold && dist3 > 0);
  char distStr3[32];
  sprintf(distStr3, "Park 3: %s (%d cm)\n", park3_occupied ? "occupied" : "free", dist3);
  USART_transmit_string(distStr3);
}

unsigned long readUltrasonicDistance() {
  GATE_TRIG_PORT &= ~(1 << GATE_TRIG_BIT);
  delayMicroseconds(2);
  GATE_TRIG_PORT |= (1 << GATE_TRIG_BIT);
  delayMicroseconds(10);
  GATE_TRIG_PORT &= ~(1 << GATE_TRIG_BIT);

  unsigned long start = micros();
  while (!(GATE_ECHO_PIN & (1 << GATE_ECHO_BIT))) {
    if (micros() - start > 30000) return 0;
  }

  unsigned long echoStart = micros();
  while (GATE_ECHO_PIN & (1 << GATE_ECHO_BIT)) {
    if (micros() - echoStart > 30000) return 0;
  }

  unsigned long duration = micros() - echoStart;
  return duration / 58;
}

unsigned long measureDistance(int trigPin, int echoPin) {
  if (trigPin == PARK1_TRIG_PIN) PORTC &= ~(1 << PC0);
  else if (trigPin == PARK2_TRIG_PIN) PORTC &= ~(1 << PC2);
  else if (trigPin == PARK3_TRIG_PIN) PORTC &= ~(1 << PC4);
  delayMicroseconds(2);

  if (trigPin == PARK1_TRIG_PIN) PORTC |= (1 << PC0);
  else if (trigPin == PARK2_TRIG_PIN) PORTC |= (1 << PC2);
  else if (trigPin == PARK3_TRIG_PIN) PORTC |= (1 << PC4);
  delayMicroseconds(10);
  if (trigPin == PARK1_TRIG_PIN) PORTC &= ~(1 << PC0);
  else if (trigPin == PARK2_TRIG_PIN) PORTC &= ~(1 << PC2);
  else if (trigPin == PARK3_TRIG_PIN) PORTC &= ~(1 << PC4);

  uint8_t echoBit;
  if (echoPin == PARK1_ECHO_PIN) echoBit = PC1;
  else if (echoPin == PARK2_ECHO_PIN) echoBit = PC3;
  else echoBit = PC5;

  unsigned long start = micros();
  while (!(PINC & (1 << echoBit))) {
    if (micros() - start > 30000) return 0;
  }

  unsigned long echoStart = micros();
  while (PINC & (1 << echoBit)) {
    if (micros() - echoStart > 30000) return 0;
  }

  unsigned long duration = micros() - echoStart;
  return duration / 58;
}

void create_and_send_park_status() {
  byte status_payload = 0;

  if (park1_occupied) status_payload |= (1 << SLOT1_BIT);
  if (park2_occupied) status_payload |= (1 << SLOT2_BIT);
  if (park3_occupied) status_payload |= (1 << SLOT3_BIT);
  if (gateOpen) status_payload |= (1 << GATE_OPEN_BIT);
  if (!gateOpen) status_payload |= (1 << GATE_CLOSED_BIT);
  if (openingGate) status_payload |= (1 << GATE_OPENING_BIT);
  if (closingGate) status_payload |= (1 << GATE_CLOSING_BIT);

  formatTXAPIpacket(status_payload);
}

void formatTXAPIpacket(byte status_value) {
  // Format and transmit a Transmit Request API frame with status data

  long sum = 0; // Accumulate the checksum

  Serial.write(frameStartByte); // API frame Start Delimiter

  // Length - High and low parts of the frame length
  // (Number of bytes between the length and the checksum)
  // Original was 0x11 for 2 bytes temp + 1 byte RSSI.
  // New is 1 byte status + 1 byte RSSI. So length decreases by 1.
  Serial.write(0x00);
  Serial.write(0x10); // New length: 16 bytes

  Serial.write(frameTypeTXrequest); // Frame Type
  sum += frameTypeTXrequest;

  Serial.write(0x00); // Frame ID - set to zero for no reply
  sum += 0x00;

  // 64-bit Destination Address
  Serial.write((destAddressHigh >> 24) & 0xFF);
  sum += ((destAddressHigh >> 24) & 0xFF);
  Serial.write((destAddressHigh >> 16) & 0xFF);
  sum += ((destAddressHigh >> 16) & 0xFF);
  Serial.write((destAddressHigh >> 8) & 0xFF);
  sum += ((destAddressHigh >> 8) & 0xFF);
  Serial.write(destAddressHigh & 0xFF);
  sum += (destAddressHigh & 0xFF);

  Serial.write((destAddressLow >> 24) & 0xFF);
  sum += ((destAddressLow >> 24) & 0xFF);
  Serial.write((destAddressLow >> 16) & 0xFF);
  sum += ((destAddressLow >> 16) & 0xFF);
  Serial.write((destAddressLow >> 8) & 0xFF);
  sum += ((destAddressLow >> 8) & 0xFF);
  Serial.write(destAddressLow & 0xFF);
  sum += (destAddressLow & 0xFF);

  // 16-bit Destination Network Address
  Serial.write(0xFF);
  Serial.write(0xFE);
  sum += 0xFF + 0xFE;

  Serial.write(0x00); // Broadcast Radius
  sum += 0x00;

  Serial.write(0x00); // Options
  sum += 0x00;

  // RF Data
  Serial.write(status_value); // Our packed status byte
  sum += status_value;
  Serial.write(rssi); // RSSI reading of the last received packet (if any)
  sum += rssi;

  // Checksum
  Serial.write(0xFF - (sum & 0xFF));

  delay(10);
}

void check_if_remote_control() {
  if (Serial.available() >= 10) {
    char key = decodeAPIpacket();
    // Remote control sent close gate command
    if (key == 'C') {
      USART_transmit_string("Remote control command received: Close gate.\n");
      setGateState(false);
    }
    else if (key == 'O') {
      USART_transmit_string("Remote control command received: Open gate.\n");
      setGateState(true);
    }
  }
}

// Add this function to your code
char decodeAPIpacket(void) {
  // Function for decoding the received API frame from XBEE
  char rxbyte = ' '; // Initialize to a non-command character
  byte frametype;
  unsigned long startTime = millis(); // For a simple timeout

  // Wait for start byte, but not indefinitely
  while (Serial.read() != frameStartByte) {
    if (Serial.available() == 0) {
      return rxbyte; // No API frame present.
    }
    if (millis() - startTime > 100) { // Timeout after 100ms of waiting for start byte
        // USART_transmit_string("Timeout waiting for 0x7E\n");
        return rxbyte;
    }
  }

  // Wait for length bytes (MSB, LSB) and frame type
  startTime = millis();
  while (Serial.available() < 3) { // Need 2 for length, 1 for type
    if (millis() - startTime > 100) { // Timeout
        // USART_transmit_string("Timeout waiting for length/type\n");
        return rxbyte;
    }
  }

  Serial.read();  // MSB of length (we are not using it directly in this simplified version)
  Serial.read();  // LSB of length (we are not using it directly)
  frametype = Serial.read();

  if (frametype == frameTypeRXpacket) { // 0x90
    // Zigbee Receive Packet API Frame
    // Expecting 13 more bytes: 8 (Addr64) + 2 (Addr16) + 1 (RecvOpt) + 1 (RF Data) + 1 (Checksum)
    startTime = millis();
    while (Serial.available() < 13) {
      if (millis() - startTime > 200) { // Timeout
          // USART_transmit_string("Timeout RX packet data\n");
          return rxbyte;
      }
    }

    // Skip over the bytes in the API frame we don't care about (addresses, receive options)
    for (int i = 0; i < 11; i++) {
      Serial.read();
    }
    // The next byte is the key pressed and sent from the remote XBEE
    rxbyte = Serial.read(); // This is your 'C' or 'O'

    Serial.read();             // Read the last byte (Checksum) but don't store it
    
    // Only query RSSI if we actually got a valid command character,
    // otherwise, we might spam DB commands on noise.
    if (rxbyte == 'C' || rxbyte == 'O') { // Or any other valid command chars
        formatATcommandAPI(DBcommand);  // query the RSSI of the last received packet
    }

  } else if (frametype == frameTypeATresponse) { // 0x88
    // AT Command Response API frame
    // Expecting 6 more bytes for DB response: 1 (FrameID) + 2 (AT Cmd) + 1 (Status) + 1 (RSSI) + 1 (Checksum)
    startTime = millis();
    while (Serial.available() < 6) {
        if (millis() - startTime > 100) { // Timeout
            // USART_transmit_string("Timeout AT response data\n");
            return rxbyte; // rxbyte will still be ' ' or its previous value
        }
    }

    // Skip Frame ID, AT Command (2 bytes), Command Status
    for (int i = 0; i < 4; i++) {
      Serial.read();
    }
    rssi = Serial.read(); // Read the RSSI value
    Serial.read();        // Read the last byte (Checksum) but don't store it
    // rxbyte remains unchanged here, which is fine.
  } else {
    // Unknown frame type, try to clear buffer to next potential start byte or until empty
    // USART_transmit_string("Unknown frame type: ");
    // char typeStr[4]; sprintf(typeStr, "%02X\n", frametype); USART_transmit_string(typeStr);
    while(Serial.available() > 0 && Serial.peek() != frameStartByte) {
        Serial.read();
    }
  }
  return rxbyte;
}

void formatATcommandAPI(char* command) {
  long sum = 0;
  ATcounter += 1;

  Serial.write(frameStartByte);
  Serial.write(0x00); // Length MSB
  Serial.write(0x04); // Length LSB (type(1) + id(1) + ATcmd(2) = 4)
  
  Serial.write(0x08); // AT Command Frame Type
  sum += 0x08;

  Serial.write(ATcounter); 
  sum += ATcounter;

  Serial.write(command[0]); 
  sum += command[0];
  Serial.write(command[1]); 
  sum += command[1];

  Serial.write(0xFF - (sum & 0xFF)); 
  delay(10); 
}
