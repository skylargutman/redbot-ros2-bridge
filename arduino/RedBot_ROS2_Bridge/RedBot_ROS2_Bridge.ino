// RedBot_ROS2_Bridge.ino
// Complete Arduino firmware for ROS2 serial bridge

#include <RedBot.h>

// Motor controller
RedBotMotors motors;

// Line follower sensors
RedBotSensor lineCenter(A6);
RedBotSensor lineRight(A7);

// Wheel encoders
RedBotEncoder encoders(A2, A3);

// Accelerometer
RedBotAccel accel;

// Bumper sensors
const int BUMP_LEFT_PIN = A0;
const int BUMP_RIGHT_PIN = A1;

// Buzzer
const int BUZZER_PIN = 9;

// Motor speed storage
int leftSpeed = 0;
int rightSpeed = 0;

void setup() {
  delay(2000);  // Upload window
  
  Serial.begin(115200);
  
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUMP_LEFT_PIN, INPUT_PULLUP);
  pinMode(BUMP_RIGHT_PIN, INPUT_PULLUP);
  
  while (!Serial) {
    ;
  }
  
  Serial.println("RedBot ROS2 Bridge v1.0");
  Serial.println("Ready for commands");
}

void loop() {
  // Check for incoming commands
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    processCommand(command);
  }
  
  // Send sensor data at 20Hz
  static unsigned long lastSensorTime = 0;
  if (millis() - lastSensorTime >= 50) {
    publishSensorData();
    lastSensorTime = millis();
  }
}

void processCommand(String cmd) {
  cmd.trim();
  
  // MOTOR command
  if (cmd.startsWith("MOTOR,")) {
    cmd.remove(0, 6);
    int commaIndex = cmd.indexOf(',');
    if (commaIndex > 0) {
      leftSpeed = cmd.substring(0, commaIndex).toInt();
      rightSpeed = cmd.substring(commaIndex + 1).toInt();
      
      leftSpeed = constrain(leftSpeed, -255, 255);
      rightSpeed = constrain(rightSpeed, -255, 255);
      
      motors.leftMotor(-leftSpeed);
      motors.rightMotor(-rightSpeed);
      
      Serial.print("ACK,MOTOR,");
      Serial.print(leftSpeed);
      Serial.print(",");
      Serial.println(rightSpeed);
    }
  }
  
  // RESET_ENCODERS command
  if (cmd.equals("RESET_ENCODERS")) {
    encoders.clearEnc(LEFT);
    encoders.clearEnc(RIGHT);
    Serial.println("ACK,RESET_ENCODERS");
  }
  
  // BUZZ command
  if (cmd.startsWith("BUZZ,")) {
    cmd.remove(0, 5);
    int commaIndex = cmd.indexOf(',');
    if (commaIndex > 0) {
      int frequency = cmd.substring(0, commaIndex).toInt();
      int duration = cmd.substring(commaIndex + 1).toInt();
      
      tone(BUZZER_PIN, frequency, duration);
      
      Serial.print("ACK,BUZZ,");
      Serial.print(frequency);
      Serial.print(",");
      Serial.println(duration);
    }
  }
}

void publishSensorData() {
  // Read line sensors
  int center = lineCenter.read();
  int right = lineRight.read();
  
  // Read accelerometer
  accel.read();
  int ax = accel.x;
  int ay = accel.y;
  int az = accel.z;
  
  // Read encoders
  long encLeft = encoders.getTicks(LEFT);
  long encRight = encoders.getTicks(RIGHT);
  
  // Read bumpers
  int bumpL = (digitalRead(BUMP_LEFT_PIN) == LOW) ? 1 : 0;
  int bumpR = (digitalRead(BUMP_RIGHT_PIN) == LOW) ? 1 : 0;
  
  // Send data
  Serial.print("SENSORS,");
  Serial.print(center);
  Serial.print(",");
  Serial.print(right);
  Serial.print(",");
  Serial.print(ax);
  Serial.print(",");
  Serial.print(ay);
  Serial.print(",");
  Serial.print(az);
  Serial.print(",");
  Serial.print(encLeft);
  Serial.print(",");
  Serial.print(encRight);
  Serial.print(",");
  Serial.print(bumpL);
  Serial.print(",");
  Serial.println(bumpR);
}
