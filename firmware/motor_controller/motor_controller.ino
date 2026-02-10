// Motor controller firmware for Arduino Nano (ATmega328P)
// Bidirectional serial with ROS 2 avr_interface_node
// Pi -> Nano: "M <left> <right>\n" with speeds in [-255, 255]
// Nano -> Pi: "S <battery_mv> <enc_left> <enc_right>\n" every 100ms
// Nano -> Pi: "READY\n" on startup
// Drives 2x L298N H-bridges (one per side, 3 motors in parallel each)

// Motor pin assignments
const int ENA = 9;   // Left motor PWM (Timer1)
const int IN1 = 8;   // Left motor direction
const int IN2 = 7;   // Left motor direction
const int ENB = 10;  // Right motor PWM (Timer1)
const int IN3 = 12;  // Right motor direction
const int IN4 = 11;  // Right motor direction

// Encoder pins (interrupt-capable)
const int ENC_LEFT_PIN = 2;   // INT0
const int ENC_RIGHT_PIN = 3;  // INT1

// Battery voltage divider on A0
// Adjust R1/R2 to match your voltage divider resistors
const long DIVIDER_R1 = 10000L;  // Top resistor (ohms)
const long DIVIDER_R2 = 3300L;   // Bottom resistor (ohms)
const int BATTERY_PIN = A0;

const unsigned long COMMAND_TIMEOUT_MS = 500;
const unsigned long TELEMETRY_INTERVAL_MS = 100;
const int SERIAL_BUF_SIZE = 32;

char serialBuf[SERIAL_BUF_SIZE];
int bufIndex = 0;
unsigned long lastCommandTime = 0;
unsigned long lastTelemetryTime = 0;

// Encoder tick counters (volatile for ISR access)
volatile long encLeftTicks = 0;
volatile long encRightTicks = 0;

// Current motor direction: 1 = forward, -1 = reverse, 0 = stopped
int leftDir = 0;
int rightDir = 0;

void leftEncoderISR() {
  encLeftTicks += (leftDir >= 0) ? 1 : -1;
}

void rightEncoderISR() {
  encRightTicks += (rightDir >= 0) ? 1 : -1;
}

void setMotor(int enPin, int in1Pin, int in2Pin, int speed, int *dirOut) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    analogWrite(enPin, speed);
    *dirOut = 1;
  } else if (speed < 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    analogWrite(enPin, -speed);
    *dirOut = -1;
  } else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    analogWrite(enPin, 0);
    *dirOut = 0;
  }
}

void stopMotors() {
  setMotor(ENA, IN1, IN2, 0, &leftDir);
  setMotor(ENB, IN3, IN4, 0, &rightDir);
}

void processCommand(const char *line) {
  int left, right;
  if (sscanf(line, "M %d %d", &left, &right) == 2) {
    setMotor(ENA, IN1, IN2, left, &leftDir);
    setMotor(ENB, IN3, IN4, right, &rightDir);
    lastCommandTime = millis();
  }
}

long readBatteryMv() {
  int adc = analogRead(BATTERY_PIN);
  return (adc * 5000L * (DIVIDER_R1 + DIVIDER_R2)) / (1024L * DIVIDER_R2);
}

void sendTelemetry() {
  // Read encoder ticks atomically
  noInterrupts();
  long left = encLeftTicks;
  long right = encRightTicks;
  interrupts();

  long batteryMv = readBatteryMv();

  Serial.print("S ");
  Serial.print(batteryMv);
  Serial.print(' ');
  Serial.print(left);
  Serial.print(' ');
  Serial.println(right);
}

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENC_LEFT_PIN, INPUT_PULLUP);
  pinMode(ENC_RIGHT_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_PIN), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_PIN), rightEncoderISR, RISING);

  stopMotors();

  Serial.begin(115200);
  Serial.println("READY");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      serialBuf[bufIndex] = '\0';
      processCommand(serialBuf);
      bufIndex = 0;
    } else if (bufIndex < SERIAL_BUF_SIZE - 1) {
      serialBuf[bufIndex++] = c;
    }
  }

  if (millis() - lastCommandTime > COMMAND_TIMEOUT_MS) {
    stopMotors();
  }

  if (millis() - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryTime = millis();
    sendTelemetry();
  }
}
