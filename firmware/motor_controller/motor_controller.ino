// Motor controller firmware for Arduino Nano (ATmega328P)
// Receives serial commands from ROS 2 motor_controller_node
// Protocol: "M <left> <right>\n" with speeds in [-255, 255]
// Drives 2x L298N H-bridges (one per side, 3 motors in parallel each)

// Pin assignments
const int ENA = 9;   // Left motor PWM (Timer1)
const int IN1 = 8;   // Left motor direction
const int IN2 = 7;   // Left motor direction
const int ENB = 10;  // Right motor PWM (Timer1)
const int IN3 = 12;  // Right motor direction
const int IN4 = 11;  // Right motor direction

const unsigned long COMMAND_TIMEOUT_MS = 500;
const int SERIAL_BUF_SIZE = 32;

char serialBuf[SERIAL_BUF_SIZE];
int bufIndex = 0;
unsigned long lastCommandTime = 0;

void setMotor(int enPin, int in1Pin, int in2Pin, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    analogWrite(enPin, speed);
  } else if (speed < 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    analogWrite(enPin, -speed);
  } else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    analogWrite(enPin, 0);
  }
}

void stopMotors() {
  setMotor(ENA, IN1, IN2, 0);
  setMotor(ENB, IN3, IN4, 0);
}

void processCommand(const char *line) {
  int left, right;
  if (sscanf(line, "M %d %d", &left, &right) == 2) {
    setMotor(ENA, IN1, IN2, left);
    setMotor(ENB, IN3, IN4, right);
    lastCommandTime = millis();
  }
}

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

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
}
