#include <Servo.h>

// Arduino pin assignment
#define PIN_LED   9   // LED (Active-Low)
#define PIN_TRIG  12  // sonar sensor TRIGGER
#define PIN_ECHO  13  // sonar sensor ECHO
#define PIN_SERVO 10  // servo motor

// configurable parameters for sonar
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25       // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 180.0   // minimum distance to be measured (unit: mm)
#define _DIST_MAX 360.0   // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL) // coefficent to convert duration to distance

// EMA filter alpha value (0.0 ~ 1.0)
// 값이 1에 가까우면 반응이 빠르고, 0에 가까우면 부드러워집니다.
#define _EMA_ALPHA 0.3

// duty duration for myservo.writeMicroseconds()
// NEEDS TUNING (servo by servo)
#define _DUTY_MIN 500 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1500 // servo neutral position (90 degree)
#define _DUTY_MAX 2550 // servo full counterclockwise position (180 degree)

// global variables
float dist_prev = (_DIST_MIN + _DIST_MAX) / 2.0; // Last valid distance (start at the middle)
float dist_ema = dist_prev;                      // EMA filtered distance
unsigned long last_sampling_time;                // unit: ms

Servo myservo;

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);
  digitalWrite(PIN_LED, HIGH); // Turn off LED at the beginning (Active-Low)

  myservo.attach(PIN_SERVO);
  // Set initial servo position to the middle (90 degrees)
  myservo.writeMicroseconds(map(dist_ema, _DIST_MIN, _DIST_MAX, _DUTY_MIN, _DUTY_MAX));

  // initialize serial port
  Serial.begin(57600);
}

void loop() {
  float dist_raw;
  long target_duty;

  // wait until next sampling time.
  if (millis() < last_sampling_time + INTERVAL)
    return;
  last_sampling_time = millis(); // More accurate than adding INTERVAL

  // 1. Get a distance reading from the USS
  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);

  // 2. Range Filter and LED control
  if (dist_raw >= _DIST_MIN && dist_raw <= _DIST_MAX) {
    // If the reading is within the valid range (18-36cm)
    digitalWrite(PIN_LED, LOW); // Turn the LED ON (Active-Low)
    dist_prev = dist_raw;       // Update the last valid distance
  } else {
    // If the reading is out of range or failed (0)
    digitalWrite(PIN_LED, HIGH); // Turn the LED OFF (Active-Low)
    // Do NOT update dist_prev, so the servo holds its last position
  }

  // 3. Apply EMA Filter to prevent jitter
  // Calculate a smoothed value (dist_ema) based on the last valid distance (dist_prev)
  dist_ema = (_EMA_ALPHA * dist_prev) + (1.0 - _EMA_ALPHA) * dist_ema;

  // 4. Proportional Servo Control
  // Map the final filtered distance to the servo's duty cycle
  target_duty = map(dist_ema, _DIST_MIN, _DIST_MAX, _DUTY_MIN, _DUTY_MAX);
  myservo.writeMicroseconds(target_duty);

  // output the distance to the serial port
  Serial.print("Dist(raw):");  Serial.print(dist_raw);      // Raw sensor reading
  Serial.print(", Dist(ema):"); Serial.print(dist_ema);    // Filtered value used for control
  Serial.print(", Servo:");    Serial.print(myservo.read()); // Current servo angle (0-180)
  Serial.println("");
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO) {
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  
  float duration = pulseIn(ECHO, HIGH, TIMEOUT);
  // If pulseIn times out, it returns 0.
  if (duration == 0) {
    return 0.0; // Return 0.0 on measurement failure
  }
  return duration * SCALE;
}
