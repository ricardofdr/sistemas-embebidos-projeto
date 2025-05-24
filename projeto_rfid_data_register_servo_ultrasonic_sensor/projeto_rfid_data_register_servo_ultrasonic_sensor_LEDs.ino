#include <SPI.h>
#include <MFRC522.h>

#define BAUD_RATE 9600
#define BAUD_RATE_DIVISOR (F_CPU / 16 / BAUD_RATE - 1)

MFRC522 mfrc522(10, 9);  // SS=10, RST=9

bool gateOpen = false;
bool waitingForCarToLeave = false;
unsigned long lastRFIDRead = 0;    // Last time RFID was read
unsigned long lastGateClose = 0;   // Last time gate was closed
const long rfidCooldown = 2000;    // 2s cooldown for RFID re-read
const long gateCloseDelay = 1000;  // 1s delay after car leaves before closing
volatile int total_estacionamentos_ocupados = 0;  // Total number of occupied parking spots

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

void updateLEDs() {
  if (total_estacionamentos_ocupados < 3) {
    PORTD |= (1 << PORTD6);   // Green LED on (PD6)
    PORTD &= ~(1 << PORTD2);  // Red LED off (PD2)
    USART_transmit_string("Parking available.\n");
  } else {
    PORTD &= ~(1 << PORTD6);  // Green LED off (PD6)
    PORTD |= (1 << PORTD2);   // Red LED on (PD2)
    USART_transmit_string("Parking full.\n");
  }
}

void setup() {
  // Initialize SPI and RFID
  SPI.begin();
  mfrc522.PCD_Init();

  // Initialize USART for serial output
  USART_init();
  USART_transmit_string("System initialized.\n");

  // Configure pins: PD7 (Trig), PD3 (Servo), PD2 (Red LED), PD6 (Green LED) as outputs, PB0 (Echo) as input
  DDRD |= (1 << DDD7) | (1 << DDD3) | (1 << DDD2) | (1 << DDD6);
  PORTD &= ~(1 << PORTD7);  // Trig low initially
  PORTD &= ~(1 << PORTD2);  // Red LED off initially
  PORTD |= (1 << PORTD6);   // Green LED on initially (parking empty)
  DDRB &= ~(1 << DDB0);     // Echo (PB0) as input

  // Configure Timer 2 for phase-correct PWM, TOP = OCR2A
  TCCR2A = (1 << COM2B1) | (0 << COM2B0) | (1 << WGM20) | (0 << WGM21);
  // COM2B1: Clear OC2B on compare match (up-counting), set on down-counting
  // WGM20: Phase-correct PWM, WGM21=0, WGM22 (in TCCR2B) sets TOP = OCR2A
  TCCR2B = (1 << WGM22) | (1 << CS22) | (0 << CS21) | (0 << CS20);
  // WGM22: Phase-correct PWM with TOP = OCR2A
  // CS22: Prescaler = 64 (16MHz / 64 = 250kHz, 4µs per tick)
  OCR2A = 249;  // ~19.92ms period (2 * 249 * 4µs)
  OCR2B = 12;   // Initial pulse width ~500µs (gate closed)

  updateLEDs(); // Initialize LED state
}

void loop() {
  unsigned long currentMillis = millis();

  // RFID check (only if cooldown period has passed and parking not full)
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

      if (uid == "9D227789" && total_estacionamentos_ocupados < 3) {
        USART_transmit_string("Access granted. Opening gate.\n");
        gateOpen = true;
        waitingForCarToLeave = true;
        OCR2B = 100;  // ~2500µs, gate open (wide angle)
      } else {
        if (total_estacionamentos_ocupados >= 3) {
          USART_transmit_string("Access denied: Parking full.\n");
        } else {
          USART_transmit_string("Access denied: Invalid card.\n");
        }
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
        if (total_estacionamentos_ocupados < 3) {
          total_estacionamentos_ocupados++;  // Increment parking count
          char countStr[32];
          sprintf(countStr, "Occupied spaces: %d\n", total_estacionamentos_ocupados);
          USART_transmit_string(countStr);
        }
        updateLEDs();  // Update LED state
        lastGateClose = currentMillis;  // Update last gate close time
      }
    } else {
      lastGateClose = currentMillis;  // Reset close delay while car is detected
    }
  }
}

unsigned long readUltrasonicDistance() {
  PORTD &= ~(1 << PORTD7);  // Trig low
  delayMicroseconds(2);
  PORTD |= (1 << PORTD7);   // Trig high
  delayMicroseconds(10);
  PORTD &= ~(1 << PORTD7);  // Trig low

  unsigned long start = micros();
  while (!(PINB & (1 << PINB0))) {
    if (micros() - start > 30000) return 0;  // Timeout
  }

  unsigned long echoStart = micros();
  while (PINB & (1 << PINB0)) {
    if (micros() - echoStart > 30000) return 0;  // Timeout
  }

  unsigned long duration = micros() - echoStart;
  return duration / 58;  // Convert to cm
}