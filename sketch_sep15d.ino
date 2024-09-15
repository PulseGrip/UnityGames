#include <ESP32Servo.h>

Servo myservo;

int pos = 0;
int potentiometerPin = 33;
int servoPin = 25;
int potentiometerValue;
int ECG_Sensor = 13;
int LEDPin = 27;

int servoMax = 130;
int servoMin = -20;

String state = "open";
const int sampleSize = 2500;
double servoPos = 0;
int speedServo = 1;

int upperBound = 2570;
int lowerBound = 2265;

float window[sampleSize];
int curIndex = 0; // initialize curIndex to 0

bool armIsFlexed = false;

unsigned long lastServoUpdateTime = 0;
const unsigned long servoUpdateInterval = 10;  // update servo every 20ms

float decayFactor = 0.983;  // Decay factor for the upper bound
float adaptiveUpperBound = upperBound;

void setup() {
  Serial.begin(115200);
  myservo.attach(servoPin);  // Attach the servo to GPIO pin 25 on ESP32
  pinMode(potentiometerPin, INPUT);
  pinMode(ECG_Sensor, INPUT);
  pinMode(LEDPin, OUTPUT);
}

void loop() {
  unsigned long currentTime = millis();
  int ECG_value = analogRead(ECG_Sensor);

  window[curIndex] = ECG_value;

  if (curIndex == sampleSize - 1) {
    float myAverage = rms();
    // Serial.println(myAverage);
    if (myAverage > adaptiveUpperBound) {
      armIsFlexed = true;
      adaptiveUpperBound = upperBound;  // Reset the adaptive upper bound
      Serial.println("1");
    } else if (myAverage < lowerBound) {
      armIsFlexed = false;
      Serial.println("0");
    }
    adaptiveUpperBound *= decayFactor;  // Apply decay to the upper bound
    curIndex = 0; // reset the index after reaching the sample size
  } else {
    curIndex++;
  }

  // Update the servo only if the specified interval has passed
  if (currentTime - lastServoUpdateTime >= servoUpdateInterval) {
    if (armIsFlexed) {
      digitalWrite(LEDPin, HIGH);
      servoPos -= speedServo;  // Move the servo backward
      if (servoPos < servoMin) {
        servoPos = servoMin;
      }
    } else {
      digitalWrite(LEDPin, LOW);
      servoPos += 0.76;  // Move the servo forward
      if (servoPos > servoMax) {
        servoPos = servoMax;
      }
    }
    myservo.write(servoPos);  // Update servo position
    //Serial.println(servoPos);
    lastServoUpdateTime = currentTime;  // Update the last update time
  }
}

float rms() {
  float tot = 0;
  for (int i = 0; i < sampleSize; i++) {
    window[i] = window[i] * window[i];
    tot += window[i];
  }

  float squaredAvg = tot / (float)(sampleSize);

  return sqrt(squaredAvg);
}