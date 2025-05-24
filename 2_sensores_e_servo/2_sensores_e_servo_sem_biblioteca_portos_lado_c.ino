// Define pins for first ultrasonic sensor (using Timer1 Input Capture)
#define TRIGGER_PIN1 A0 // PC0/A0
#define ECHO_PIN1 A1    // PC1/A1

// Define pins for second ultrasonic sensor (using manual pulse timing)
#define TRIGGER_PIN2 A2 // PC2/A2
#define ECHO_PIN2 A3    // PC3/A3

// Define pin for servo motor (using manual pulse generation)
#define SERVO_PIN 3 // PD3

const unsigned int precision = (1000000 / (F_CPU / 1000)) * 8; // constant for conversion
const unsigned int ns2cm = 58000;
volatile unsigned int start = 0;
volatile unsigned long atraso = 0;
volatile byte direccao = 1; // direction of count (1 is up)

// Timing variables - read car positions and declared them every 2 seconds
unsigned long lastTriggerTime = 0;
const unsigned long triggerInterval = 1000; // 2 seconds between readings

// Parking management variables
// volatile int total_carros_no_estacionamento = 0;
volatile int total_estacionamentos_ocupados = 0; // Total number of occupied parking spots
volatile bool park1_occupied = false;            // Track previous state of parking spot 1
volatile bool park2_occupied = false;            // Track previous state of parking spot 2
const int distanceThreshold = 5;                 // Distance threshold for parking spot detection (in cm)
volatile int distance1 = 0;                      // Distance for parking spot 1
volatile int distance2 = 0;                      // Distance for parking spot 2
volatile bool car1_leaving = false;
volatile bool car2_leaving = false;

// Servo control variables
volatile bool doorOpen = false;
int holdDoor = 3000; // variable to store the servo position

void setup()
{
    // put your setup code here, to run once:
    Serial.begin(9600);

    // Configure first sensor pins (PORTC)
    DDRC |= (1 << DDC0);  // Set A0 (pin 14) as output for trigger
    DDRC &= ~(1 << DDC1); // Set A1 (pin 15) as input for echo

    // Configure second sensor pins (PORTC)
    DDRC |= (1 << DDC2);  // Set A2 (pin 16) as output for trigger
    DDRC &= ~(1 << DDC3); // Set A3 (pin 17) as input for echo

    // Configure servo pin as output
    DDRD |= (1 << DDD3); // Set PD3 (pin 3) as output for servo

    // Ensure trigger pins start LOW
    PORTC &= ~(1 << PC0); // First sensor trigger LOW
    PORTC &= ~(1 << PC2); // Second sensor trigger LOW

    // Configure Timer2 for servo PWM on pin 3 (OC2B)
    TCCR2A = (1 << COM2B1) | (0 << COM2B0) | (1 << WGM21) | (1 << WGM20); // Fast PWM, non-inverting on OC2B
    TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);                     // Prescaler 1024 (slowest possible)

    setServoPosition(0); // Initialize servo position

    Serial.println("Dual ultrasonic sensor system initialized");
}

void loop()
{
    Serial.println("Reading both parking sensors");

    // Store previous state before updating
    bool prev_park1_occupied = park1_occupied;
    bool prev_park2_occupied = park2_occupied;

    checkParkingSlots();

    // Check for leaving cars using previous and current states
    car1_leaving = prev_park1_occupied && !park1_occupied;
    car2_leaving = prev_park2_occupied && !park2_occupied;

    // Handle car leaving
    if (car1_leaving)
    {
        if (!doorOpen)
        {
            int parkNumber = 1;
            Serial.print("Opening door for car leaving park ");
            Serial.println(parkNumber);

            setServoPosition(90);
            doorOpen = true;
            delay(holdDoor);
            setServoPosition(0);

            Serial.print("Door is closed for car leaving park ");
            Serial.println(parkNumber);
            doorOpen = false;
        }
        else
        {
            delay(holdDoor);
            setServoPosition(0);
            Serial.print("Door is closed for car leaving park ");
            Serial.println(1);
            doorOpen = false;
        }
    }

    if (car2_leaving)
    {
        if (!doorOpen)
        {
            int parkNumber = 2;
            Serial.print("Opening door for car leaving park ");
            Serial.println(parkNumber);

            setServoPosition(90);
            doorOpen = true;
            delay(holdDoor);
            setServoPosition(0);

            Serial.print("Door is closed for car leaving park ");
            Serial.println(parkNumber);
            doorOpen = false;
        }
        else
        {
            delay(holdDoor);
            setServoPosition(0);
            Serial.print("Door is closed for car leaving park ");
            Serial.println(2);
            doorOpen = false;
        }
    }

    Serial.print("Total occupied parking spots: ");
    Serial.println(total_estacionamentos_ocupados);

    Serial.println("Waiting for next measurement...");
    delay(2000);
}

void checkParkingSlots()
{
    total_estacionamentos_ocupados = 0; // Reset the count for each loop iteration
    distance1 = measureDistance(TRIGGER_PIN1, ECHO_PIN1);
    park1_occupied = (distance1 < distanceThreshold && distance1 > 0);
    if (park1_occupied)
    {
        Serial.println("Park 1 is occupied");
        Serial.print(distance1);
        Serial.println(" cm");
        total_estacionamentos_ocupados++;
    }
    else
    {
        Serial.println("Park 1 is free");
        Serial.print(distance1);
        Serial.println(" cm");
    }

    distance2 = measureDistance(TRIGGER_PIN2, ECHO_PIN2);
    park2_occupied = (distance2 < distanceThreshold && distance2 > 0);
    if (park2_occupied)
    {
        Serial.println("Park 2 is occupied");
        Serial.print(distance2);
        Serial.println(" cm");
        total_estacionamentos_ocupados++;
    }
    else
    {
        Serial.println("Park 2 is free");
        Serial.print(distance2);
        Serial.println(" cm");
    }
}

long measureDistance(int triggerPin, int echoPin)
{
    // Use correct ports based on which pin we're working with
    if (triggerPin == TRIGGER_PIN1)
    {
        // First sensor uses Port B
        PORTC &= ~(1 << 0); // Clear A0 (pin 14)
        delayMicroseconds(2);
        PORTC |= (1 << 0); // Set A0 high
        delayMicroseconds(10);
        PORTC &= ~(1 << 0); // Set A0 low
    }
    else if (triggerPin == TRIGGER_PIN2)
    {
        // Second sensor uses Port D
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

// Function to set servo position (0-180 degrees)
void setServoPosition(int degrees)
{
    Serial.print("Setting servo to ");
    Serial.print(degrees);
    Serial.println(" degrees");

    // Constrain the input
    if (degrees < 0)
        degrees = 0;
    if (degrees > 180)
        degrees = 180;

    //   uint8_t pulseWidth = 16 + ((uint16_t)degrees * 16 / 180);
    // uint8_t pulseWidth = 15 + ((uint16_t)degrees * 20 / 180);
    uint8_t pulseWidth = 12 + ((uint16_t)degrees * 12 / 180);

    // Debug output
    Serial.print("OCR2B value: ");
    Serial.println(pulseWidth);

    // Set OCR2B register for OC2B output (pin 3)
    OCR2B = pulseWidth; // CORRECTED: Using OCR2B
}

// void setServoPosition(int degrees)
// {
//     Serial.print("Setting servo to ");
//     Serial.print(degrees);
//     Serial.println(" degrees");

//     // Constrain the input
//     if (degrees < 0) degrees = 0;
//     if (degrees > 180) degrees = 180;

//     // Calculate pulse width in microseconds (1000-2000 µs range)
//     int pulseWidth = 1000 + (degrees * 1000 / 180);

//     Serial.print("Pulse width: ");
//     Serial.print(pulseWidth);
//     Serial.println(" µs");

//     // Generate PWM signal manually for 1 second to ensure servo moves
//     for (int i = 0; i < 50; i++) { // 50 pulses = 1 second at 50Hz
//         PORTD |= (1 << PD3);        // Set pin high
//         delayMicroseconds(pulseWidth);
//         PORTD &= ~(1 << PD3);       // Set pin low
//         delay(20 - (pulseWidth / 1000)); // Complete 20ms period
//     }
// }