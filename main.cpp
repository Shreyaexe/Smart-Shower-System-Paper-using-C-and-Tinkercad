
#include <Wire.h>
#include <LiquidCrystal_I2C.h> // Include the I2C LCD library
#include <Servo.h>             // Include the Servo library

// Pin definitions
#define trigPin 9
#define echoPin 10
#define motorIn1 5 // Motor 1 control (L293D)
#define motorIn2 6 // Motor 2 control (L293D)

const int force = A0;      // Force sensor pin
const int Motor = 2;       // Motor control pin

const int tempPin = A1;    // TMP36 temperature sensor pin
const int PIR_sensor = 12; // PIR sensor pin
const int lights = 8;      // Lights control pin
const int servoPin = 11;   // Servo motor pin for valve

// Initialize variables
int forceValue = 0;
int motorON = 0;
unsigned long lastMotionTime = 0; // To track the last motion time (in milliseconds)
unsigned long motionDelay = 5000; // 5 seconds delay after no motion (in milliseconds)

// Ultrasonic sensor variables
long duration, distance;

// Create Servo object
Servo valveServo; // This will control the servo motor

// Initialize the LCD (set the address 0x27 or 0x3F based on your LCD module)
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  // Set up input/output pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn2, OUTPUT);

  pinMode(Motor, OUTPUT);
  pinMode(force, INPUT);

  pinMode(PIR_sensor, INPUT);
  pinMode(lights, OUTPUT);

  // Initialize serial communication
  Serial.begin(9600);
  delay(2500); // Allow sensors to stabilize

  // Initialize the Servo motor
  valveServo.attach(servoPin);

  // Initialize the LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Starting...");
  delay(2000); // Display starting message
  lcd.clear();
}

void loop() {
  // PIR sensor functionality for automatic lights
  if (digitalRead(PIR_sensor) == HIGH) {
    digitalWrite(lights, HIGH);   // Turn ON lights
    lastMotionTime = millis();    // Update last motion time
    lcd.setCursor(0, 0);
    lcd.print("Motion: Detected  ");
  } else {
    if ((millis() - lastMotionTime) >= motionDelay) {
      digitalWrite(lights, LOW);  // Turn OFF lights after delay
      lcd.setCursor(0, 0);
      lcd.print("Motion: None     ");
    }
  }

  // Force sensor logic for motor control
  forceValue = analogRead(force);
  int threshold = 500;

  if (forceValue > threshold) {  // Force detected
    digitalWrite(Motor, HIGH);
    motorON = 1;
    valveServo.write(180); // Open valve fully
    lcd.setCursor(0, 1);
    lcd.print("Force: Detected  ");
  } else if (motorON == 1) {     // Force no longer detected
    digitalWrite(Motor, LOW);
    motorON = 0;
    valveServo.write(0); // Close valve
    lcd.setCursor(0, 1);
    lcd.print("Force: None      ");
  }

  // Ultrasonic sensor logic for distance measurement
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration == 0) ? 100 : duration * 0.034 / 2;

  Serial.print("Distance: ");
  Serial.println(distance); // Debugging output

  lcd.setCursor(0, 0);
  lcd.print("Distance: ");
  lcd.print(distance);
  lcd.print(" cm  ");

  // Adjust motor speed based on distance
  int motorSpeed;
  if (distance < 30) {
    motorSpeed = 255; // Full speed
  } else if (distance >= 30 && distance < 100) {
    motorSpeed = 150; // Medium speed
  } else {
    motorSpeed = 0;   // Motor off
  }

  setMotorSpeed(motorSpeed);

  // Temperature sensor logic for servo control
  int tempValue = analogRead(tempPin);
  float voltage = tempValue * (5.0 / 1023.0);
  float temperature = (voltage - 0.5) * 100.0;

  Serial.print("Temperature: ");
  Serial.println(temperature); // Debugging output

  lcd.setCursor(0, 1);
  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.print(" C ");

  if (temperature < 20) {
    valveServo.write(0); // Close valve
  } else if (temperature >= 20 && temperature <= 30) {
    valveServo.write(90); // Half-open valve
  } else if (temperature > 30) {
    valveServo.write(180); // Fully open valve
  }

  delay(100);
}

void setMotorSpeed(int speed) {
  if (speed > 0) {
    analogWrite(motorIn1, speed);
    digitalWrite(motorIn2, LOW); // Forward rotation
  } else {
    digitalWrite(motorIn1, LOW);
    digitalWrite(motorIn2, LOW); // Stop motor
  }
}

