#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

typedef struct struct_message {
    int x;
    int y;
} struct_message;

const int LEFT_SPEED_PIN = 0;
const int RIGHT_SPEED_PIN = 1;
const int IN1 = 2;
const int IN2 = 3;
const int IN3 = 4;
const int IN4 = 5;
unsigned long lastReceivedTime = 0;
int deadzone = 30;
int leftSpeed, rightSpeed = 0;
int center_x = -45;
int center_y = -50;

struct_message incomingData;

void stopMotors() {
    ledcWrite(LEFT_SPEED_PIN, 0);
    ledcWrite(RIGHT_SPEED_PIN, 0);
}

void setMotorDirection(int pin1, int pin2, int speed) {
    digitalWrite(pin1, speed < 0);
    digitalWrite(pin2, speed >= 0);
}


void onDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
    lastReceivedTime = millis();
    memcpy(&incomingData, data, sizeof(incomingData));

    if (abs(incomingData.x - center_x) < deadzone && abs(incomingData.y - center_y) < deadzone) {
        stopMotors();
        return;
    }

    float deltaX = incomingData.x - center_x;
    float deltaY = incomingData.y - center_y;

    // Graded deadzone implementation
    float gradedDeadzone = 2 * deadzone;
    if (abs(deltaX) < gradedDeadzone) {
        float scale = (abs(deltaX) - deadzone) / (gradedDeadzone - deadzone);
        deltaX *= scale;
    }
    if (abs(deltaY) < gradedDeadzone) {
        float scale = (abs(deltaY) - deadzone) / (gradedDeadzone - deadzone);
        deltaY *= scale;
    }

    int mappedX = (int) (deltaY - deltaX);
    int mappedY = (int) (deltaY + deltaX);

    leftSpeed = constrain(map(mappedX, -179, 179, -255, 255), -255, 255);
    rightSpeed = constrain(map(mappedY, -179, 179, -255, 255), -255, 255);
    leftSpeed = -leftSpeed;

    Serial.printf("Data: X: %d, Y: %d, LeftSpeed: %d, RightSpeed: %d\n", incomingData.x, incomingData.y, leftSpeed,
                  rightSpeed);

    setMotorDirection(IN1, IN2, leftSpeed);
    setMotorDirection(IN3, IN4, rightSpeed);

    ledcWrite(LEFT_SPEED_PIN, abs(leftSpeed));
    ledcWrite(RIGHT_SPEED_PIN, abs(rightSpeed));
}

void setup() {
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    Serial.begin(115200);

    ledcSetup(0, 25000, 8);
    ledcAttachPin(LEFT_SPEED_PIN, 0);

    ledcSetup(1, 25000, 8);
    ledcAttachPin(RIGHT_SPEED_PIN, 1);

    WiFiClass::mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) return;

    esp_now_register_recv_cb(onDataRecv);
}

void loop() {
    if (millis() - lastReceivedTime > 100) {
        stopMotors();
    }
}
