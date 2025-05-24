#include <SPI.h>
#include <MFRC522.h>

// Define pins for RFID reader
#define SS_PIN 10 // Slave Select pin for RFID
#define RST_PIN 9 // Reset pin for RFID
MFRC522 mfrc522(SS_PIN, RST_PIN);

// Define pins for first ultrasonic sensor (using Timer1 Input Capture)
#define TRIGGER_PIN1 A0 // PC0/A0
#define ECHO_PIN1 A1    // PC1/A1

// Define pins for second ultrasonic sensor (using manual pulse timing)
#define TRIGGER_PIN2 A2 // PC2/A2
#define ECHO_PIN2 A3    // PC3/A3

// Define pin for servo motor (using manual pulse generation)
#define SERVO_PIN 3 // PD3

#define BAUD_RATE 9600
#define BAUD_RATE_DIVISOR (F_CPU / 16 / BAUD_RATE - 1)

// Variables for ultrasonic sensor
const unsigned int precision = (1000000 / (F_CPU / 1000)) * 8; // constant for conversion
const unsigned int ns2cm = 58000;
volatile unsigned int start = 0;
volatile unsigned long atraso = 0;
volatile byte direccao = 1; // direction of count (1 is up)

// Timing variables - read car positions and declared them every 2 seconds
unsigned long lastTriggerTime = 0;
const unsigned long triggerInterval = 1000; // 2 seconds between readings

bool gateOpen = false;
unsigned long lastRFIDRead = 0;                  // Last time RFID was read
unsigned long lastGateClose = 0;                 // Last time gate was closed
const long rfidCooldown = 2000;                  // 2s cooldown for RFID re-read
const long gateCloseDelay = 1000;                // 1s delay after car leaves before closing
volatile int total_estacionamentos_ocupados = 0; // Total number of occupied parking spots
volatile int total_carros_no_estacionamento = 0; // Total number of cars in parking
volatile bool carAtParkDoor = false;             // Flag to indicate if a car is entering the park
volatile bool carLeavingPark = false;            // Flag to indicate if a car is entering the park

volatile bool park1_occupied = false; // Track previous state of parking spot 1
volatile bool park2_occupied = false; // Track previous state of parking spot 2
const int distanceThreshold = 5;      // Distance threshold for parking spot detection (in cm)
volatile int distance1 = 0;           // Distance for parking spot 1
volatile int distance2 = 0;           // Distance for parking spot 2
volatile bool car1_leaving = false;
volatile bool car2_leaving = false;

// Add these constants after the other defines at the top
const unsigned long GATE_TIMEOUT = 10000;       // 10 seconds maximum gate open time
const unsigned long STATE_RESET_TIMEOUT = 5000; // 5 seconds to reset stuck states

// Add these variables after the other global variables
unsigned long gateOpenTime = 0;
unsigned long lastStateChange = 0;

void USART_init(void)
{
    UCSR0A = 0;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
    UBRR0 = BAUD_RATE_DIVISOR;
}

void USART_transmit(char data)
{
    while (!(UCSR0A & (1 << UDRE0)))
        ;
    UDR0 = data;
}

void USART_transmit_string(const char *str)
{
    while (*str)
    {
        USART_transmit(*str++);
    }
}

void updateLEDs()
{
    if (total_carros_no_estacionamento < 3)
    {
        PORTD |= (1 << PORTD6);  // Green LED on (PD6)
        PORTD &= ~(1 << PORTD2); // Red LED off (PD2)
        USART_transmit_string("Parking available.\n");
    }
    else
    {
        PORTD &= ~(1 << PORTD6); // Green LED off (PD6)
        PORTD |= (1 << PORTD2);  // Red LED on (PD2)
        USART_transmit_string("Parking full.\n");
    }
}

void setup()
{
    // Initialize SPI and RFID
    SPI.begin();
    mfrc522.PCD_Init();

    // Initialize USART for serial output
    USART_init();
    USART_transmit_string("System initialized.\n");

    // Configure pins: PD7 (Trig), PD3 (Servo), PD2 (Red LED), PD6 (Green LED) as outputs, PB0 (Echo) as input
    DDRD |= (1 << DDD7) | (1 << DDD3) | (1 << DDD2) | (1 << DDD6);
    PORTD &= ~(1 << PORTD7); // Trig low initially
    PORTD &= ~(1 << PORTD2); // Red LED off initially
    PORTD |= (1 << PORTD6);  // Green LED on initially (parking empty)
    DDRB &= ~(1 << DDB0);    // Echo (PB0) as input

    // Configure first ultrasonic sensor pins (PORTC)
    DDRC |= (1 << DDC0);  // Set A0 (pin 14) as output for trigger
    DDRC &= ~(1 << DDC1); // Set A1 (pin 15) as input for echo

    // Configure second ultrasonic sensor pins (PORTC)
    DDRC |= (1 << DDC2);  // Set A2 (pin 16) as output for trigger
    DDRC &= ~(1 << DDC3); // Set A3 (pin 17) as input for echo

    // Configure Timer 2 for phase-correct PWM, TOP = OCR2A
    TCCR2A = (1 << COM2B1) | (0 << COM2B0) | (1 << WGM20) | (0 << WGM21);
    // COM2B1: Clear OC2B on compare match (up-counting), set on down-counting
    // WGM20: Phase-correct PWM, WGM21=0, WGM22 (in TCCR2B) sets TOP = OCR2A
    TCCR2B = (1 << WGM22) | (1 << CS22) | (0 << CS21) | (0 << CS20);
    // WGM22: Phase-correct PWM with TOP = OCR2A
    // CS22: Prescaler = 64 (16MHz / 64 = 250kHz, 4µs per tick)
    OCR2A = 249; // ~19.92ms period (2 * 249 * 4µs)
    OCR2B = 12;  // Initial pulse width ~500µs (gate closed)

    updateLEDs(); // Initialize LED state
}

void loop()
{
    unsigned long currentMillis = millis();

    // // Add timeout check for gate operations
    // if (carAtParkDoor && gateOpen)
    // {
    //     if (gateOpenTime == 0)
    //     {
    //         gateOpenTime = currentMillis;  // Record when gate opened
    //     }
    //     else if (currentMillis - gateOpenTime > GATE_TIMEOUT)
    //     {
    //         USART_transmit_string("Gate timeout - forcing close\n");
    //         OCR2B = 12;  // Force gate closed
    //         gateOpen = false;
    //         carAtParkDoor = false;
    //         carLeavingPark = false;
    //         gateOpenTime = 0;
    //     }
    // }
    // else
    // {
    //     gateOpenTime = 0;  // Reset timer when gate is not in operation
    // }

    // // Reset stuck car leaving states
    // if ((car1_leaving || car2_leaving) && (currentMillis - lastStateChange > STATE_RESET_TIMEOUT))
    // {
    //     USART_transmit_string("Resetting stuck car leaving states\n");
    //     car1_leaving = false;
    //     car2_leaving = false;
    //     carLeavingPark = false;
    // }

    // RFID check (only if cooldown period has passed and parking not full)
    if (currentMillis - lastRFIDRead >= rfidCooldown)
    {
        // Store previous state before updating
        bool prev_park1_occupied = park1_occupied;
        bool prev_park2_occupied = park2_occupied;

        // Check current parking slots status if there are cars in the parking
        if (total_carros_no_estacionamento > 0)
        {
            checkParkingSlots();
        }

        bool prev_car1_leaving = car1_leaving;
        bool prev_car2_leaving = car2_leaving;

        car1_leaving = prev_park1_occupied && !park1_occupied;
        car2_leaving = prev_park2_occupied && !park2_occupied;

        // Update state change time if car leaving status changed
        if (car1_leaving != prev_car1_leaving || car2_leaving != prev_car2_leaving)
        {
            lastStateChange = currentMillis;
        }

        rfidReading(currentMillis);
    }

    // Check ultrasonic sensor if gate is open and waiting for car to leave
    if (carAtParkDoor)
    {
        unsigned long dist = readUltrasonicDistance();

        char distStr[32];
        sprintf(distStr, "Distance: %lu cm\n", dist);
        USART_transmit_string(distStr);

        if (dist > 30 || dist == 0)
        {
            if (currentMillis - lastGateClose >= gateCloseDelay)
            {
                USART_transmit_string("Car left. Closing gate.\n");
                OCR2B = 12; // ~500µs, gate closed
                gateOpen = false;
                carAtParkDoor = false;

                // Flag to check if the car is leaving the parking slots
                if (carLeavingPark)
                {
                    total_carros_no_estacionamento--; // Decrement parking count when car leaves
                    carLeavingPark = false;           // Reset flag after processing
                    char countStr[32];
                    sprintf(countStr, "Occupied spaces: %d\n", total_carros_no_estacionamento);
                    USART_transmit_string(countStr);
                }
                else
                // Car is entering the parking slots
                {
                    if (total_carros_no_estacionamento < 3)
                    {
                        total_carros_no_estacionamento++; // Increment parking count
                        char countStr[32];
                        sprintf(countStr, "Occupied spaces: %d\n", total_carros_no_estacionamento);
                        USART_transmit_string(countStr);
                    }
                }
                updateLEDs();                  // Update LED state
                lastGateClose = currentMillis; // Update last gate close time
            }
        }
        else
        {
            lastGateClose = currentMillis; // Reset close delay while car is detected
        }
    }
    else // Check if no car wants to exit from inside the park
    {
        // Check if a car is leaving the parking slots
        if (car1_leaving || car2_leaving)
        {
            if (total_estacionamentos_ocupados > 0)
            {
                total_estacionamentos_ocupados--;
            }
            char carMsg[64];
            sprintf(carMsg, "Car is no longer in his parking slot: %d\n", car1_leaving ? 1 : 2);
            USART_transmit_string(carMsg);
            unsigned long dist = readUltrasonicDistance();
            if (dist < 20)
            {
                carLeavingPark = true; // Set flag to indicate car is leaving
                carAtParkDoor = true;
                char parkStr[32];
                sprintf(parkStr, "Park %d\n", car1_leaving ? 1 : 2);
                USART_transmit_string(parkStr);
                if (!gateOpen) // If gate is closed open
                {
                    gateOpen = true;
                    OCR2B = 100; // ~2500µs, gate open (wide angle)
                }
            }
        }
        else
        {
            carLeavingPark = false; // Reset flag if no car is leaving
            // No car at the park gate and no car leaving the parking slots
            gateOpen = false; // Ensure gate is closed
            OCR2B = 12;       // ~500µs, gate closed
        }
    }
}

void checkParkingSlots()
{
    total_estacionamentos_ocupados = 0; // Reset the count for each loop iteration
    distance1 = measureDistance(TRIGGER_PIN1, ECHO_PIN1);
    park1_occupied = (distance1 < distanceThreshold && distance1 > 0);
    if (park1_occupied)
    {
        USART_transmit_string("Park 1 is occupied");
        char distStr[32];
        sprintf(distStr, " %d cm\n", distance1);
        USART_transmit_string(distStr);
        total_estacionamentos_ocupados++;
    }
    else
    {
        USART_transmit_string("Park 1 is free");
        char distStr[32];
        sprintf(distStr, " %d cm\n", distance1);
        USART_transmit_string(distStr);
    }

    distance2 = measureDistance(TRIGGER_PIN2, ECHO_PIN2);
    park2_occupied = (distance2 < distanceThreshold && distance2 > 0);
    if (park2_occupied)
    {
        USART_transmit_string("Park 2 is occupied");
        char distStr[32];
        sprintf(distStr, " %d cm\n", distance2);
        USART_transmit_string(distStr);
        total_estacionamentos_ocupados++;
    }
    else
    {
        USART_transmit_string("Park 2 is free");
        char distStr[32];
        sprintf(distStr, " %d cm\n", distance2);
        USART_transmit_string(distStr);
    }
}

void rfidReading(long currentMilis)
{
    if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial())
    {
        String uid = "";
        for (byte i = 0; i < mfrc522.uid.size; i++)
        {
            char buf[3];
            sprintf(buf, "%02X", mfrc522.uid.uidByte[i]);
            uid += buf;
        }
        uid.toUpperCase();

        USART_transmit_string("UID: ");
        USART_transmit_string(uid.c_str());
        USART_transmit_string("\n");

        if (uid == "9D227789" && total_carros_no_estacionamento < 3)
        {
            USART_transmit_string("Access granted. Opening gate.\n");
            gateOpen = true;
            carAtParkDoor = true;
            OCR2B = 100; // ~2500µs, gate open (wide angle)
        }
        else
        {
            if (total_carros_no_estacionamento >= 3)
            {
                USART_transmit_string("Access denied: Parking full.\n");
            }
            else
            {
                USART_transmit_string("Access denied: Invalid card.\n");
            }
            gateOpen = false;
            carAtParkDoor = false;
            OCR2B = 12; // ~500µs, gate closed
        }

        lastRFIDRead = currentMillis; // Update last RFID read time
    }
}

unsigned long readUltrasonicDistance()
{
    PORTD &= ~(1 << PORTD7); // Trig low
    delayMicroseconds(2);
    PORTD |= (1 << PORTD7); // Trig high
    delayMicroseconds(10);
    PORTD &= ~(1 << PORTD7); // Trig low

    unsigned long start = micros();
    while (!(PINB & (1 << PINB0)))
    {
        if (micros() - start > 30000)
            return 0; // Timeout
    }

    unsigned long echoStart = micros();
    while (PINB & (1 << PINB0))
    {
        if (micros() - echoStart > 30000)
            return 0; // Timeout
    }

    unsigned long duration = micros() - echoStart;
    return duration / 58; // Convert to cm
}

long measureDistance(int triggerPin, int echoPin)
{
    // Use correct ports based on which pin we're working with
    if (triggerPin == TRIGGER_PIN1)
    {
        // First sensor uses Port C
        PORTC &= ~(1 << 0); // Clear A0 (pin 14)
        delayMicroseconds(2);
        PORTC |= (1 << 0); // Set A0 high
        delayMicroseconds(10);
        PORTC &= ~(1 << 0); // Set A0 low
    }
    else if (triggerPin == TRIGGER_PIN2)
    {
        // Second sensor uses Port C
        PORTC &= ~(1 << 2); // Clear A2 (pin 16)
        delayMicroseconds(2);
        PORTC |= (1 << 2); // Set A2 high
        delayMicroseconds(10);
        PORTC &= ~(1 << 2); // Set A2 low
    }

    // Measure the echo pulse duration manually
    unsigned long duration = pulseIn(echoPin, HIGH, 30000);

    // Convert to distance (cm)
    if (duration == 0)
    {
        return 0; // No echo received
    }
    else
    {
        return duration / 58; // Convert to cm (speed of sound)
    }
}