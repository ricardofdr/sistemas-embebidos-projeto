#include <SPI.h>
#include <MFRC522.h>

#define BAUD_RATE 9600
#define BAUD_RATE_DIVISOR (F_CPU / 16 / BAUD_RATE - 1)

MFRC522 mfrc522(10, 9); // SS=10, RST=9

#define TRIG_PIN_DDR DDRD
#define TRIG_PIN_PORT PORTD
#define TRIG_PIN_BIT PORTD7   // Pin 7 (PD7)

#define ECHO_PIN_PIN PINB
#define ECHO_PIN_BIT PINB0    // Pin 8 (PB0)

bool gateOpen = false;
bool waitingForCarToLeave = false;
unsigned long lastRFIDRead = 0;  // Last time RFID was read
unsigned long lastGateClose = 0; // Last time gate was closed
const long rfidCooldown = 2000;  // 2s cooldown for RFID re-read
const long gateCloseDelay = 1000; // 1s delay after car leaves before closing

void USART_init(void) {
  UCSR0A = 0;
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
  UBRR0 = BAUD_RATE_DIVISOR;
}

void USART_transmit(char data) {
  while (!(UCSR0A & (1 << UDRE0)));
  UDR0 = data;
}

void USART_transmit_string(const char* str) {
  while (*str) {
    USART_transmit(*str++);
  }
}

void setup() {
  // Initialize SPI and RFID
  SPI.begin();
  mfrc522.PCD_Init();

  // Initialize USART for serial output
  USART_init();
  USART_transmit_string("System initialized.\n");

  // Configure ultrasonic pins
  TRIG_PIN_DDR |= (1 << TRIG_PIN_BIT);    // Trig (PD7) as output
  TRIG_PIN_PORT &= ~(1 << TRIG_PIN_BIT);  // Trig low initially
  DDRB &= ~(1 << DDB0);                   // Echo (PB0) as input

  // Configure servo pin (Pin 3 = PD3 / OC2B) as output
  DDRD |= (1 << DDD3);

  // Configure Timer 2 for phase-correct PWM, TOP = OCR2A
  TCCR2A = (1 << COM2B1) | (0 << COM2B0) | (1 << WGM20) | (0 << WGM21);  
  // COM2B1: Clear OC2B on compare match (up-counting), set on down-counting
  // WGM20: Phase-correct PWM, WGM21=0, WGM22 (in TCCR2B) sets TOP = OCR2A

  TCCR2B = (1 << WGM22) | (1 << CS22) | (0 << CS21) | (0 << CS20);  
  // WGM22: Phase-correct PWM with TOP = OCR2A
  // CS22: Prescaler = 64 (16MHz / 64 = 250kHz, 4µs per tick)

  OCR2A = 249;  // Set TOP for ~19.92ms period (2 * 249 * 4µs)
  OCR2B = 12;   // Initial pulse width ~500µs (gate closed)
}

void loop() {
  unsigned long currentMillis = millis();

  // RFID check (only if cooldown period has passed)
  if (currentMillis - lastRFIDRead >= rfidCooldown) {
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

      if (uid == "9D227789") {
        USART_transmit_string("Access granted. Opening gate.\n");
        gateOpen = true;
        waitingForCarToLeave = true;
        OCR2B = 62;  // ~2500µs, gate open (wide angle)
      } else {
        USART_transmit_string("Access denied.\n");
        gateOpen = false;
        waitingForCarToLeave = false;
        OCR2B = 12;  // ~500µs, gate closed
      }

      lastRFIDRead = currentMillis;  // Update last RFID read time
    }
  }

  // Check ultrasonic sensor if gate is open and waiting for car to leave
  if (waitingForCarToLeave) {
    unsigned long dist = readUltrasonicDistance();

    char distStr[32];
    sprintf(distStr, "Distance: %lu cm\n", dist);
    USART_transmit_string(distStr);

    if (dist > 30 || dist == 0) {
      if (currentMillis - lastGateClose >= gateCloseDelay) {
        USART_transmit_string("Car left. Closing gate.\n");
        OCR2B = 12;  // ~500µs, gate closed
        gateOpen = false;
        waitingForCarToLeave = false;
        lastGateClose = currentMillis;  // Update last gate close time
      }
    } else {
      lastGateClose = currentMillis;  // Reset close delay while car is detected
    }
  }
}

unsigned long readUltrasonicDistance() {
  // Trigger pulse
  TRIG_PIN_PORT &= ~(1 << TRIG_PIN_BIT); // Low
  delayMicroseconds(2);
  TRIG_PIN_PORT |= (1 << TRIG_PIN_BIT);  // High
  delayMicroseconds(10);
  TRIG_PIN_PORT &= ~(1 << TRIG_PIN_BIT); // Low

  // Measure echo
  unsigned long start = micros();
  while (!(ECHO_PIN_PIN & (1 << ECHO_PIN_BIT))) {
    if (micros() - start > 30000) return 0;  // Timeout
  }

  unsigned long echoStart = micros();
  while (ECHO_PIN_PIN & (1 << ECHO_PIN_BIT)) {
    if (micros() - echoStart > 30000) return 0;  // Timeout
  }

  unsigned long duration = micros() - echoStart;
  return duration / 58; // Convert to cm
}