#include "PID_v1.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include <SoftwareSerial.h>

// Motor Pins
#define ENA 9
#define IN1 5
#define IN2 4
#define ENB 10
#define IN3 6
#define IN4 7

// Bluetooth Module Pins (reassigned to avoid conflicts)
#define BT_RX 12 // Arduino TX -> HC-06 RX
#define BT_TX 11 // Arduino RX -> HC-06 TX

// MPU6050 and PID Variables
MPU6050 mpu;
volatile bool mpuInterrupt = false;
bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3]; // [yaw, pitch, roll]
double setpoint = 4.3; // Desired angle
double input, output;

// PID Tuning Parameters
double Kp = 20, Ki = 195, Kd = 1.5;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Bluetooth Communication
SoftwareSerial bluetooth(BT_RX, BT_TX); // RX, TX

// Timing for periodic updates
unsigned long lastUpdate = 0;
const unsigned long updateInterval = 1000; // 1 second

// Function Prototypes
void dmpDataReady();
void moveMotors(double leftSpeed, double rightSpeed);
void updatePIDConstant(char command, String valueStr);
void sendCurrentValues();

void setup() {
    // Initialize serial communication (for debugging)
    Serial.begin(115200);

    // Initialize Bluetooth communication
    bluetooth.begin(115200); // Default baud rate of HC-06
    Serial.println("Bluetooth ready!");

    // Initialize I2C and MPU6050
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock
    #endif

    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        while (1);
    }
    mpu.setXAccelOffset(126);
    mpu.setYAccelOffset(458);
    mpu.setZAccelOffset(893);
    mpu.setXGyroOffset(138);

    // Load and configure the DMP
    devStatus = mpu.dmpInitialize();
    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
        packetSize = mpu.dmpGetFIFOPacketSize();
        dmpReady = true;
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-255, 255);
        Serial.println("MPU6050 ready with DMP enabled!");
    } else {
        Serial.print("DMP Initialization failed (code ");
        Serial.print(devStatus);
        Serial.println(")");
        while (1);
    }

    // Set motor pins as outputs
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
}

void loop() {
    if (!dmpReady) return;

    // Check for Bluetooth commands
    if (bluetooth.available()) {
        char command = bluetooth.read();
        Serial.println(command);
        String valueStr = bluetooth.readStringUntil('\n');
        if (command == 'P' || command == 'I' || command == 'D' || command == 'O') {
            updatePIDConstant(command, valueStr);
            sendCurrentValues();
        }
    }

    // // Periodically send current PID values and input/output
    // if (millis() - lastUpdate >= updateInterval) {
    //     lastUpdate = millis();
    //     sendCurrentValues();
    // }

    // Wait for MPU interrupt or available packet
    while (!mpuInterrupt && fifoCount < packetSize);

    mpuInterrupt = false;
    fifoCount = mpu.getFIFOCount();

    if (fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println("FIFO overflow!");
        return;
    }

    if (fifoCount >= packetSize) {
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        input = ypr[1] * 180 / M_PI; // Pitch angle in degrees

        pid.Compute();

        // Move motors based on PID output
        double leftSpeed = constrain(output, -255, 255);
        double rightSpeed = constrain(output, -255, 255);
        moveMotors(leftSpeed, rightSpeed);
    }
}

void moveMotors(double leftSpeed, double rightSpeed) {
    // Left Motor Control
    if (leftSpeed > 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, leftSpeed);
    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(ENA, -leftSpeed);
    }

    // Right Motor Control
    if (rightSpeed > 0) {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(ENB, rightSpeed);
    } else {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        analogWrite(ENB, -rightSpeed);
    }
}

void updatePIDConstant(char command, String valueStr) {
    double value = valueStr.toFloat();

    if (command == 'P') {
        Kp = value;
        bluetooth.print("Updated Kp: ");
        bluetooth.println(Kp);
    } else if (command == 'I') {
        Ki = value;
        bluetooth.print("Updated Ki: ");
        bluetooth.println(Ki);
    } else if (command == 'D') {
        Kd = value;
        bluetooth.print("Updated Kd: ");
        bluetooth.println(Kd);
    } else if (command == 'O') {
        setpoint = value;
        bluetooth.print("Updated Setpoint: ");
        bluetooth.println(setpoint);
    }

    pid.SetTunings(Kp, Ki, Kd);
}

void sendCurrentValues() {
    bluetooth.print("Current Values - Kp: ");
    bluetooth.print(Kp);
    bluetooth.print(", Ki: ");
    bluetooth.print(Ki);
    bluetooth.print(", Kd: ");
    bluetooth.print(Kd);
    bluetooth.print(", Setpoint: ");
    bluetooth.print(setpoint);
    bluetooth.print(", Input: ");
    bluetooth.print(input);
    bluetooth.print(", Output: ");
    bluetooth.println(output);
}

void dmpDataReady() {
    mpuInterrupt = true;
}
