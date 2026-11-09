#include <ESP32Servo.h>

#define BASE_PIN       27
#define SHOULDER_PIN   26
#define ELBOW_PIN      25
#define WRIST1_PIN     33
#define WRIST2_PIN     32
#define GRIPPER_PIN    13

Servo baseServo, shoulderServo, elbowServo, wrist1Servo, wrist2Servo, gripperServo;

int baseAngle = 90, shoulderAngle = 90, elbowAngle = 90;
int wrist1Angle = 90, wrist2Angle = 90, gripperAngle = 90;

void setServoAngle(Servo &servo, int &currentAngle, int targetAngle)
{
  targetAngle = constrain(targetAngle, 0, 180);
  currentAngle = targetAngle;
  servo.write(targetAngle);
}

void processCommand(String command)
{
  command.trim();
  int separator = command.indexOf(':');
  if (separator == -1) return;

  String joint = command.substring(0, separator);
  int angle = constrain(command.substring(separator + 1).toInt(), 0, 180);

  if      (joint == "BASE")     setServoAngle(baseServo, baseAngle, angle);
  else if (joint == "SHOULDER") setServoAngle(shoulderServo, shoulderAngle, angle);
  else if (joint == "ELBOW")    setServoAngle(elbowServo, elbowAngle, angle);
  else if (joint == "WRIST1")   setServoAngle(wrist1Servo, wrist1Angle, angle);
  else if (joint == "WRIST2")   setServoAngle(wrist2Servo, wrist2Angle, angle);
  else if (joint == "GRIPPER")  setServoAngle(gripperServo, gripperAngle, angle);

  Serial.print("OK:"); Serial.print(joint); Serial.print(":"); Serial.println(angle);
}

void setup()
{
  Serial.begin(115200);
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  Servo* all[] = {&baseServo, &shoulderServo, &elbowServo, &wrist1Servo, &wrist2Servo, &gripperServo};
  int pins[]   = {BASE_PIN, SHOULDER_PIN, ELBOW_PIN, WRIST1_PIN, WRIST2_PIN, GRIPPER_PIN};
  for (int i = 0; i < 6; i++) {
    all[i]->setPeriodHertz(50);
    all[i]->attach(pins[i], 500, 2400);
    all[i]->write(90);
  }
  delay(1000);
  Serial.println("ESP32_ROBOT_READY");
}

void loop()
{
  if (Serial.available())
    processCommand(Serial.readStringUntil('\n'));
}
