#ifndef IMU_BNO055
#define IMU_BNO055

#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include <vector>

#include "hardware/assembly/definitions.hpp"

class Imu_BNO055 : public Adafruit_BNO055 {
  public:
    const uint8_t PinSda;
    const uint8_t PinScl;

    Imu_BNO055(std::vector<uint8_t> pins, uint8_t address=BNO055_ADDRESS_A)
    : Adafruit_BNO055{address=address}, PinSda{pins[0]}, PinScl{pins[1]} {}
    ~Imu_BNO055() = default;

    float get_roll(void) { return Adafruit_BNO055::getVector(Adafruit_BNO055::VECTOR_EULER).x(); }
    float get_pitch(void) { return Adafruit_BNO055::getVector(Adafruit_BNO055::VECTOR_EULER).y(); }
    float get_yaw(void) { return Adafruit_BNO055::getVector(Adafruit_BNO055::VECTOR_EULER).z(); }
    
    void test() {
        //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
        sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
        getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
        getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
        getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
        getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
        getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

        printEvent(&orientationData);
        printEvent(&angVelocityData);
        printEvent(&linearAccelData);
        printEvent(&magnetometerData);
        printEvent(&accelerometerData);
        printEvent(&gravityData);

        int8_t boardTemp = getTemp();
        Serial.println();
        Serial.print(F("temperature: "));
        Serial.println(boardTemp);

        uint8_t system, gyro, accel, mag = 0;
        getCalibration(&system, &gyro, &accel, &mag);
        Serial.println();
        Serial.print("Calibration: Sys=");
        Serial.print(system);
        Serial.print(" Gyro=");
        Serial.print(gyro);
        Serial.print(" Accel=");
        Serial.print(accel);
        Serial.print(" Mag=");
        Serial.println(mag);

        Serial.println("--");
        delay(100);
    }
    void printEvent(sensors_event_t* event) {
        double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
        if (event->type == SENSOR_TYPE_ACCELEROMETER) {
            Serial.print("Accl:");
            x = event->acceleration.x;
            y = event->acceleration.y;
            z = event->acceleration.z;
        }
        else if (event->type == SENSOR_TYPE_ORIENTATION) {
            Serial.print("Orient:");
            x = event->orientation.x;
            y = event->orientation.y;
            z = event->orientation.z;
        }
        else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
            Serial.print("Mag:");
            x = event->magnetic.x;
            y = event->magnetic.y;
            z = event->magnetic.z;
        }
        else if (event->type == SENSOR_TYPE_GYROSCOPE) {
            Serial.print("Gyro:");
            x = event->gyro.x;
            y = event->gyro.y;
            z = event->gyro.z;
        }
        else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
            Serial.print("Rot:");
            x = event->gyro.x;
            y = event->gyro.y;
            z = event->gyro.z;
        }
        else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
            Serial.print("Linear:");
            x = event->acceleration.x;
            y = event->acceleration.y;
            z = event->acceleration.z;
        }
        else if (event->type == SENSOR_TYPE_GRAVITY) {
            Serial.print("Gravity:");
            x = event->acceleration.x;
            y = event->acceleration.y;
            z = event->acceleration.z;
        }
        else {
            Serial.print("Unk:");
        }

        Serial.print("\tx= ");
        Serial.print(x);
        Serial.print(" |\ty= ");
        Serial.print(y);
        Serial.print(" |\tz= ");
        Serial.println(z);
    }
};

#endif//IMU_BNO055