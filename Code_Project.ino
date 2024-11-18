#include <ESP32Servo.h>
#include "ESC.h"

int escPin1 = 18; // PWM pin for ESC 1 (BLDC 1)
int escPin2 = 19; // PWM pin for ESC 2 (BLDC 2)
int escPin3 = 22; // PWM pin for SG90 servo 1
int escPin4 = 21; // PWM pin for SG90 servo 2

// ตัวแปรสำหรับเซ็นเซอร์
int frontSensorPin = 34; // IR sensor for the front
int leftSensorPin = 35;  // IR sensor for the left
int rightSensorPin = 32; // IR sensor for the right

// ประกาศตัวแปรสำหรับการควบคุม BLDC และ SG90
Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;

int pwmTarget = 1060; //ตั้งความเร็วสูงสุด
int pwmIncrement = 0.03; // ตั้งค่าความเร่งเพื่อให้รถวิ่งด้วยความ smooth
unsigned long previousMillis = 0;

const unsigned long pwmInterval = 50; 
const int frontThreshold = 99;        // กำหนด irfront ในระยะ 99 cm
const int leftSensorThreshold = 30;   // กำหนด irleft  ในระยะ 30 cm
const int rightSensorThreshold = 30;  // กำหนด irright ในระยะ 30 cm

#define SENSOR_HISTORY_SIZE 5  // ลดขนาด Sliding Window 5

int frontSensorHistory[SENSOR_HISTORY_SIZE] = {0}; 
int leftSensorHistory[SENSOR_HISTORY_SIZE] = {0};  
int rightSensorHistory[SENSOR_HISTORY_SIZE] = {0};  
int historyIndex = 0;
int frontSum = 0, leftSum = 0, rightSum = 0; 

// ฟังก์ชันการกรองค่าของเซ็นเซอร์ด้วย Moving Average
int filterSensorRead(int pin, int* sensorHistory, int& sum) {
  int newValue = analogRead(pin);  // อ่านค่าจากเซ็นเซอร์

  // หักค่าที่จะถูกแทนที่ออกจากผลรวม
  sum -= sensorHistory[historyIndex];

  // อัปเดตค่าใหม่ในประวัติและเพิ่มเข้าผลรวม
  sensorHistory[historyIndex] = newValue;
  sum += newValue;

  // หมุนตำแหน่ง index
  historyIndex = (historyIndex + 1) % SENSOR_HISTORY_SIZE;

  // คำนวณค่าเฉลี่ย
  return sum / SENSOR_HISTORY_SIZE;
}

void setup() {
  // Attach ESCs และเซอร์โว
  esc1.attach(escPin1);
  esc2.attach(escPin2);
  esc3.attach(escPin3);
  esc4.attach(escPin4);

  esc1.writeMicroseconds(1000); // ตั้วค่า BLDC ให้หยุดด้วย 1000 (1000 = หยุด)
  esc2.writeMicroseconds(1000); // ตั้วค่า BLDC ให้หยุดด้วย 1000 (1000 = หยุด)
  esc3.write(90);  // ตั้งค่า sg90 ที่ 90 องศา
  esc4.write(90);  // ตั้งค่า sg90 ที่ 90 องศา
  pinMode(frontSensorPin, INPUT);
  pinMode(leftSensorPin, INPUT);
  pinMode(rightSensorPin, INPUT);

  // setCpufrequency(240);
  Serial.begin(115200);  
  delay(2000);  

  // สร้าง function ในหาค่า average
  for (int i = 0; i < SENSOR_HISTORY_SIZE; i++) {
    frontSensorHistory[i] = analogRead(frontSensorPin);  // อ่านค่าเซ็นเซอร์หน้า 5  ครั้งแรก
    leftSensorHistory[i] = analogRead(leftSensorPin);    // อ่านค่าเซ็นเซอร์ซ้าย 5  ครั้งแรก
    rightSensorHistory[i] = analogRead(rightSensorPin);  // อ่านค่าเซ็นเซอร์ขวา 5  ครั้งแรก
    frontSum += frontSensorHistory[i];
    leftSum += leftSensorHistory[i];
    rightSum += rightSensorHistory[i];
    delay(10); 
  }
}

void loop() {
  unsigned long currentMillis = millis();

  // ใช้ฟังก์ชันกรองค่าแยกกันสำหรับแต่ละเซ็นเซอร์
  int frontDistance = filterSensorRead(frontSensorPin, frontSensorHistory, frontSum);  
  int leftDistance = filterSensorRead(leftSensorPin, leftSensorHistory, leftSum);  
  int rightDistance = filterSensorRead(rightSensorPin, rightSensorHistory, rightSum);  
  Serial.print(frontDistance);
  Serial.print("  :  ");

  // ตั้งค่า map เพื่อแปลงค่า analog เป็นหน่วย cm
  int frontDistance_G = map(frontDistance,500, 4095, 100, 0);
  int leftDistance_G = map(leftDistance, 0, 4000, 100, 0);
  int rightDistance_G = map(rightDistance, 0, 4000, 100, 0);
  
  frontDistance_G = constrain(frontDistance_G, 0, 100);
  leftDistance_G = constrain(leftDistance_G, 0, 100);
  rightDistance_G = constrain(rightDistance_G, 0, 100);


  // Serial.print("     ");  
  Serial.print("Front Distance: ");
  Serial.print(frontDistance_G);
  Serial.print(" cm   :   ");
  Serial.print("left Distance: ");
  Serial.print(leftDistance_G);
  Serial.print(" cm   :   ");
  Serial.print("right Distance: ");
  Serial.print(rightDistance_G);
  Serial.print(" cm                           ");

  if (frontDistance_G < frontThreshold) {
    // Stop if an object is detected in front
    esc1.writeMicroseconds(1000);
    esc2.writeMicroseconds(1000);
    esc3.write(45); // Center position 90
    esc4.write(135); // Center position 90
    Serial.println("Stopping - Obstacle ahead");
    // Serial.println("");
  } else if (leftDistance_G < leftSensorThreshold) {
    // Turn right if an obstacle is detected on the left
    esc3.write(55);   // SG90 servo 1: Turn to 65 degrees
    esc4.write(55);   // SG90 servo 2: Turn to 65 degrees
    Serial.println("Turning right - Obstacle on left");
    // Serial.println("");
  } else if (rightDistance_G < rightSensorThreshold) {
    // Turn left if an obstacle is detected on the right
    esc3.write(125);  // SG90 servo 1: Turn to 115 degrees
    esc4.write(125);  // SG90 servo 2: Turn to 115 degrees
    Serial.println("Turning left - Obstacle on right");
    // Serial.println("");
  } else {
    
    esc3.write(90);   // Center position 90
    esc4.write(90);   // Center position 90
    esc1.writeMicroseconds(pwmTarget);
    esc2.writeMicroseconds(pwmTarget);
    Serial.println("Moving straight");
  }

  // Small delay for stability
  delay(30);  // ลดเวลาหน่วงเพื่อเพิ่มความเร็ว
}