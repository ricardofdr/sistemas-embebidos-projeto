#include <Servo.h>
Servo myservo; // create servo object to control a servo

// Variables for distance calculation
const unsigned int precision = (1000000 / (F_CPU / 1000)) * 8; // constant for conversion
const unsigned int ns2cm = 58000;
volatile unsigned int start = 0;
volatile unsigned long atraso = 0;
volatile byte direccao = 1; // direction of count (1 is up)

// Define pins for first ultrasonic sensor (using Timer1 Input Capture)
#define TRIGGER_PIN1 10 // PB2
#define ECHO_PIN1 8     // PB0

// Define pins for second ultrasonic sensor (using manual pulse timing)
#define TRIGGER_PIN2 7 // PD7
#define ECHO_PIN2 6    // PD6  // FIXED: This was missing in the pulseIn() call

// Define pin for servo motor (using manual pulse generation)
#define SERVO_PIN 2 // PD2

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

  myservo.attach(SERVO_PIN);
  Serial.println("Testing servo movement");
  myservo.write(0);
  delay(1000);
  myservo.write(90);
  delay(1000);
  myservo.write(0);
  delay(1000);
  Serial.println("Servo test complete");

  // Configure first sensor pins (PORTB)
  DDRB |= (1 << DDB2);  // Set PB2 (pin 10) as output for trigger
  DDRB &= ~(1 << DDB0); // Set PB0 (pin 8) as input for echo

  // Configure second sensor pins (PORTD)
  DDRD |= (1 << DDD7);  // Set PD7 (pin 7) as output for trigger
  DDRD &= ~(1 << DDD6); // Set PD6 (pin 6) as input for echo

  // Configure servo pin as output
  // DDRD |= (1 << DDD2);   // Set PD2 (pin 2) as output for servo

  // Ensure trigger pins start LOW
  PORTB &= ~(1 << PB2); // First sensor trigger LOW
  PORTD &= ~(1 << PD7); // Second sensor trigger LOW
  // PORTD &= ~(1 << PD2);  // Servo pin LOW

  // Set up Timer1 for input capture (for first sensor)
  // TCCR1A = 0; // Normal operation, no PWM
  // TCCR1B = 0; // Normal operation, no PWM

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

      myservo.write(90);
      doorOpen = true;
      delay(holdDoor);
      myservo.write(0);

      Serial.print("Door is closed for car leaving park ");
      Serial.println(parkNumber);
      doorOpen = false;
    }
    else
    {
      delay(holdDoor);
      myservo.write(0);
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

      myservo.write(90);
      doorOpen = true;
      delay(holdDoor);
      myservo.write(0);

      Serial.print("Door is closed for car leaving park ");
      Serial.println(parkNumber);
      doorOpen = false;
    }
    else
    {
      delay(holdDoor);
      myservo.write(0);
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
    PORTB &= ~(1 << 2); // Clear PB2 (pin 10)
    delayMicroseconds(2);
    PORTB |= (1 << 2); // Set PB2 high
    delayMicroseconds(10);
    PORTB &= ~(1 << 2); // Set PB2 low
  }
  else if (triggerPin == TRIGGER_PIN2)
  {
    // Second sensor uses Port D
    PORTD &= ~(1 << 7); // Clear PD7 (pin 7)
    delayMicroseconds(2);
    PORTD |= (1 << 7); // Set PD7 high
    delayMicroseconds(10);
    PORTD &= ~(1 << 7); // Set PD7 low
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
