#include <Servo.h>
Servo myservo;  // create servo object to control a servo

// Variables for distance calculation
const unsigned int precision = (1000000 / (F_CPU / 1000)) * 8;  // constant for conversion
const unsigned int ns2cm = 58000;
volatile unsigned int start = 0;
volatile unsigned long atraso = 0;
volatile byte direccao = 1;  // direction of count (1 is up)

// Define pins for first ultrasonic sensor (using Timer1 Input Capture)
#define TRIGGER_PIN1 10  // PB2
#define ECHO_PIN1 8      // PB0

// Define pins for second ultrasonic sensor (using manual pulse timing)
#define TRIGGER_PIN2 7  // PD7
#define ECHO_PIN2 6     // PD6  // FIXED: This was missing in the pulseIn() call

// Define pin for servo motor (using manual pulse generation)
#define SERVO_PIN 2      // PD2

// Timing variables - read car positions and declared them every 2 seconds
unsigned long lastTriggerTime = 0;
const unsigned long triggerInterval = 1000;  // 2 seconds between readings

// Parking management variables
volatile int total_carros_no_estacionamento = 0;
volatile int total_estacionamentos_ocupados = 0;
volatile bool park1_occupied = false;  // Track previous state of parking spot 1
volatile bool park2_occupied = false;  // Track previous state of parking spot 2
volatile bool car1_leaving = false;    // Flag to indicate a car is leaving
volatile bool car2_leaving = false;    // Flag to indicate a car is leaving

// Servo control variables
volatile bool doorOpen = false;
int timeon = 1500;  // variable to store the servo position


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  myservo.attach(SERVO_PIN);
  myservo.write(0);

    // Configure first sensor pins (PORTB)
  DDRB |= (1 << DDB2);   // Set PB2 (pin 10) as output for trigger
  DDRB &= ~(1 << DDB0);  // Set PB0 (pin 8) as input for echo

  // Configure second sensor pins (PORTD)
  DDRD |= (1 << DDD7);   // Set PD7 (pin 7) as output for trigger
  DDRD &= ~(1 << DDD6);  // Set PD6 (pin 6) as input for echo

  // Configure servo pin as output
  // DDRD |= (1 << DDD2);   // Set PD2 (pin 2) as output for servo

  // Ensure trigger pins start LOW
  PORTB &= ~(1 << PB2);  // First sensor trigger LOW
  PORTD &= ~(1 << PD7);  // Second sensor trigger LOW
  // PORTD &= ~(1 << PD2);  // Servo pin LOW

  // Set up Timer1 for input capture (for first sensor)
  TCCR1A = 0;  // Normal operation, no PWM
  TCCR1B = (0 << WGM13) | (0 << WGM12) | (0 << CS12) | (1 << CS11) | (0 << CS10) | (1 << ICES1);
  TIMSK1 = (1 << ICIE1);  // Enable input capture interrupt

  // Enable global interrupts
  sei();

  Serial.println("Dual ultrasonic sensor system initialized");
  Serial.println("Reading both sensors every 2 seconds...");

}

void loop() {
  // Check if it's time to take measurements
  unsigned long currentTime = millis();
  if (currentTime - lastTriggerTime >= triggerInterval) {
    Serial.println("\n----- New Measurement Cycle -----");

    // First sensor measurement (using Timer1 Input Capture)
    Serial.println("Measuring with Sensor 1 (PB2/PB0)...");
    sendTriggerPulse1();
    delay(50);  // Wait for measurement to complete
    unsigned int distance1 = (atraso * precision) / ns2cm;

    // Second sensor measurement (using manual pulse timing)
    Serial.println("Measuring with Sensor 2 (PD7/PD6)...");
    unsigned int distance2 = measureDistance2();

    manageNumberOfParkedCars(distance1, distance2);

    if (car1_leaving) {
      // openParkingDoor();
      if (!doorOpen) {
        doorOpen = true;
        // doorOpenTime = millis(); // Timestamp for when door is going to be open
        Serial.println("Opening parking door for exiting car 2");

        for (int angle = 0; angle <= 90; angle++) {  // Sweep from 0 to 180 degrees
          myservo.write(angle);
          delay(200);                // Wait for 20ms
        }
        for (int angle = 90; angle >= 0; angle--) {  // Sweep back from 180 to 0 degrees
          myservo.write(angle);                      // Move servo to the specified angle
          delay(200);                                 // Wait for 20ms
        }


        total_carros_no_estacionamento--;

      } else {
        Serial.println("Parking door is open, waiting 3 seconds...");
        for (int angle = 0; angle <= 90; angle++) {  // Sweep from 0 to 180 degrees
          myservo.write(angle);
          delay(200);                // Wait for 20ms
        }
        for (int angle = 90; angle >= 0; angle--) {  // Sweep back from 180 to 0 degrees
          myservo.write(angle);
          delay(200);                // Wait for 20ms
        }

        total_carros_no_estacionamento--;
      }
      doorOpen = false;
      car1_leaving = false;
    }

    if (car2_leaving) {
      // openParkingDoor();
      if (!doorOpen) {
        doorOpen = true;
        // doorOpenTime = millis(); // Timestamp for when door is going to be open
        Serial.println("Opening parking door for exiting car");

        for (int angle = 0; angle <= 90; angle++) {  // Sweep from 0 to 180 degrees
          myservo.write(angle);
          delay(200);                // Wait for 20ms
        }
        for (int angle = 90; angle >= 0; angle--) {  // Sweep back from 180 to 0 degrees
          myservo.write(angle);
          delay(200);                // Wait for 20ms
        }


        total_carros_no_estacionamento--;

      } else {
        Serial.println("Parking door is open, waiting 3 seconds...");
        for (int angle = 0; angle <= 90; angle++) {  // Sweep from 0 to 180 degrees
          myservo.write(angle);
          delay(200);                // Wait for 20ms
        }
        for (int angle = 90; angle >= 0; angle--) {  // Sweep back from 180 to 0 degrees
          myservo.write(angle);                      // Move servo to the specified angle
          delay(200);                                 // Wait for 20ms
        }

        total_carros_no_estacionamento--;
      }
      doorOpen = false;
      car2_leaving = false;
    }

    lastTriggerTime = currentTime;
  }
}


// Function to manually generate a 10µs trigger pulse for first sensor
void sendTriggerPulse1() {
  // Set trigger pin HIGH
  PORTB |= (1 << PB2);

  // Wait 10 microseconds
  delayMicroseconds(10);

  // Set trigger pin LOW
  PORTB &= ~(1 << PB2);
}

// Function to manage the number of parked cars
void manageNumberOfParkedCars(unsigned int distance1, unsigned int distance2) {
  // Define threshold for detecting a car (5 cm)
  const unsigned int threshold = 5;

  // Check parking spot 1
  bool current_park1_occupied = (distance1 < threshold && distance1 > 0);

  // Only update count if state has changed
  if (current_park1_occupied != park1_occupied) {
    if (current_park1_occupied) {
      // Car has arrived at spot 1
      total_estacionamentos_ocupados++;
      total_carros_no_estacionamento++;
      Serial.print("Park 1 is now occupied: ");
      Serial.print(distance1);
      Serial.println(" cm");
      car1_leaving = false;  // Signal that a car is leaving
    } else {
      // Car has left spot 1
      total_estacionamentos_ocupados--;
      Serial.println("Park 1 is now free");
      Serial.print(distance1);
      Serial.println(" cm");
      car1_leaving = true;  // Signal that a car is leaving
    }
    park1_occupied = current_park1_occupied;
  }

  // Check parking spot 2
  bool current_park2_occupied = (distance2 < threshold && distance2 > 0);

  // Only update count if state has changed
  if (current_park2_occupied != park2_occupied) {
    if (current_park2_occupied) {
      // Car has arrived at spot 2
      total_estacionamentos_ocupados++;
      total_carros_no_estacionamento++;
      Serial.print("Park 2 is now occupied: ");
      Serial.print(distance2);
      Serial.println(" cm");
      car2_leaving = false;  // Signal that a car is leaving
    } else {
      // Car has left spot 2
      total_estacionamentos_ocupados--;
      Serial.println("Park 2 is now free");
      Serial.print(distance2);
      Serial.println(" cm");
      car2_leaving = true;  // Signal that a car is leaving
    }
    park2_occupied = current_park2_occupied;
  }

  // Ensure counts don't go negative
  if (total_estacionamentos_ocupados < 0) {
    total_estacionamentos_ocupados = 0;
  }
  if (total_carros_no_estacionamento < 0) {
    total_carros_no_estacionamento = 0;
  }

  // Print total occupied spots
  Serial.print("Total parked cars: ");
  Serial.println(total_estacionamentos_ocupados);
  Serial.print("Total cars that entered the parking: ");
  Serial.println(total_carros_no_estacionamento);
}

// Function to measure distance using the second sensor
unsigned int measureDistance2() {
  // Clear the trigger pin
  PORTD &= ~(1 << PD7);
  delayMicroseconds(2);

  // Send 10µs pulse
  PORTD |= (1 << PD7);
  delayMicroseconds(10);
  PORTD &= ~(1 << PD7);

  // Measure the echo pulse duration manually
  unsigned long duration = pulseIn(ECHO_PIN2, HIGH, 30000);  // FIXED: Now using ECHO_PIN2

  // Convert to distance (cm)
  if (duration == 0) {
    return 0;  // No echo received
  } else {
    return duration / 58;  // Convert to cm (speed of sound)
  }
}

/* ISR interrupt vector for input capture (first sensor) */
ISR(TIMER1_CAPT_vect) {
  if (TCCR1B & (1 << ICES1)) {  // Rising edge detected
    start = ICR1;               // Save the timer value

    // Switch to falling edge detection
    TCCR1B &= ~(1 << ICES1);
  } else {  // Falling edge detected
    // Calculate pulse duration
    atraso = ICR1 - start;

    // Switch back to rising edge detection
    TCCR1B |= (1 << ICES1);
  }
}