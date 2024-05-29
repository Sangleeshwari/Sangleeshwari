#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Initialize the LCD, set the address to 0x27 for a 16x2 display
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Sensor pins
const int voltagePin = A0;
const int currentPin = A1;
const int tempPin = A2; // For LM35
// const int tempPin = 3; // For DS18B20

// Relay pin
const int relayPin = 4;

// Sensor calibration values
const float voltageDividerFactor = 11.0; // Adjust based on your voltage divider
const float currentSensorFactor = 0.185; // For ACS712 5A, adjust based on your sensor

// Safety thresholds
const float maxVoltage = 14.8; // Max safe voltage for battery
const float maxCurrent = 10.0; // Max safe current for battery
const float maxTemperature = 45.0; // Max safe temperature in Celsius

void setup() {
  // Initialize LCD
  lcd.begin();
  lcd.backlight();

  // Initialize relay pin
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW); // Start with the relay off

  // Begin serial communication for debugging
  Serial.begin(9600);

  // Initialize temperature sensor (if using DS18B20)
  // pinMode(tempPin, INPUT);
}

void loop() {
  // Read voltage
  int voltageRaw = analogRead(voltagePin);
  float batteryVoltage = (voltageRaw * (5.0 / 1023.0)) * voltageDividerFactor;

  // Read current
  int currentRaw = analogRead(currentPin);
  float batteryCurrent = (currentRaw * (5.0 / 1023.0) - 2.5) / currentSensorFactor;

  // Read temperature
  int tempRaw = analogRead(tempPin);
  float batteryTemperature = (tempRaw * (5.0 / 1023.0)) * 100.0; // For LM35
  // float batteryTemperature = getTemperature(); // For DS18B20

  // Display values on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("V: "); lcd.print(batteryVoltage); lcd.print("V");
  lcd.setCursor(0, 1);
  lcd.print("I: "); lcd.print(batteryCurrent); lcd.print("A T: ");
  lcd.print(batteryTemperature); lcd.print("C");

  // Print values to serial monitor
  Serial.print("Voltage: "); Serial.print(batteryVoltage); Serial.println(" V");
  Serial.print("Current: "); Serial.print(batteryCurrent); Serial.println(" A");
  Serial.print("Temperature: "); Serial.print(batteryTemperature); Serial.println(" C");

  // Safety checks
  if (batteryVoltage > maxVoltage || batteryCurrent > maxCurrent || batteryTemperature > maxTemperature) {
    digitalWrite(relayPin, LOW); // Turn off charging
    lcd.setCursor(0, 1);
    lcd.print("Charging Stopped!");
  } else {
    digitalWrite(relayPin, HIGH); // Turn on charging
  }

  // Delay before next reading
  delay(1000);
}

/*
// Function to read temperature from DS18B20
float getTemperature() {
  // Add code to read temperature from DS18B20
  // Return temperature in Celsius
}
*/
