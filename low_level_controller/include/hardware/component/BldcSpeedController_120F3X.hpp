#ifndef BLDC_SPEED_CONTROLLER_120F3X
#define BLDC_SPEED_CONTROLLER_120F3X

#include <Arduino.h>
#include <vector>
#include <math.h>

#include "hardware/assembly/definitions.hpp"
#include "utilities.hpp"

#define ANALOG_WRITE_RES    500
#define MIN_WRITE_RANGE     pow
#define MAX_WRITE_RANGE

using RotDir = Sign;

class BldcSpeedController_120F3X : public PwmController {
    const uint8_t PinPwm;
    HardwareSerial* Telemetry;
    float offset;
    // Servo motor;
    using PwmController::attach;
    using PwmController::writeMicroseconds;
    using PwmController::detach;

    const int PWM_FREQUENCY = 500; // pulse width modulation with 2ms period: min 1000us - mid 1500us - max 2000us
    const int PWM_RESOLUTION = 16; // analog output resolution
    const int PWM_MAX_TCKS = pow(2, PWM_RESOLUTION); 
    const int PWM_MIN_TCKS = PWM_MAX_TCKS / 2; 
    const int PWM_MID_TCKS = 0.5*(PWM_MIN_TCKS + PWM_MAX_TCKS);
    const int PWM_TRIM = +20;

  public:

    BldcSpeedController_120F3X(const PinVector& pins, bool bidirectional, RotDir direction)
    : PinPwm{pins[0]}, Telemetry{nullptr}, bidirectional{bidirectional}, direction{direction},
      zero{bidirectional ? PWM_MID_TCKS : PWM_MIN_TCKS}, min_range{PWM_MIN_TCKS - zero}, max_range{PWM_MAX_TCKS - zero} {
        for(auto& pin : pins) pinMode(pin, OUTPUT_OPENDRAIN);
        analogWriteFrequency(PinPwm, PWM_FREQUENCY); // pulse width modulation with 2ms period
        if(bidirectional) zero += map(map(PWM_TRIM, -100, 100, MIN_PWM, MAX_PWM), MIN_PWM, MAX_PWM, PWM_MIN_TCKS, PWM_MAX_TCKS);
        analogWrite(PinPwm, zero);
    }
    BldcSpeedController_120F3X(const PinVector& pins, HardwareSerial& telemetry, bool bidirectional, RotDir direction)
    : PinPwm{pins[0]}, Telemetry{&telemetry}, bidirectional{bidirectional}, direction{direction},
      zero{bidirectional ? PWM_MID_TCKS : PWM_MIN_TCKS}, min_range{PWM_MIN_TCKS - zero}, max_range{PWM_MAX_TCKS - zero} {
        for(auto& pin : pins) pinMode(pin, OUTPUT_OPENDRAIN);
        if(bidirectional) zero += map(map(PWM_TRIM, -100, 100, MIN_PWM, MAX_PWM), MIN_PWM, MAX_PWM, PWM_MIN_TCKS, PWM_MAX_TCKS);
        analogWriteFrequency(PinPwm, PWM_FREQUENCY);
        analogWrite(PinPwm, zero);
    }
    ~BldcSpeedController_120F3X() = default;

    virtual void set_unit_signal(float value) {  
        uint32_t PREV_PWM_RESOLUTION = analogWriteResolution(PWM_RESOLUTION);   
        if(bidirectional) {
            // assertm(-1 <= value && value <= 1, "Invalid value for bidirectional 120F3X");
            // analogWrite(PinPwm, map(value * direction, -1., +1., 0., 256.));
            // writeMicroseconds(zero + map(value * direction, -1., +1., min_range, max_range));
            // analogWrite(zero + map(value * direction, -1., +1., min_range, max_range));
            value = constrain(value, -1., +1.);
            analogWrite(PinPwm, zero + map(value * direction, -1., +1., min_range, max_range));
        } else {
            // assertm(0 <= value && value <= 1, "Invalid value for 120F3X");
            value = constrain(value, 0., +1.);
            analogWrite(PinPwm, zero + map(value,  0., +1., min_range, min_range));
            // writeMicroseconds(zero + map(value, 0., +1., min_range, max_range));
        }
        // motor.writeMicroseconds(1550); // zero at 1514
        // motor.writeMicroseconds(MID); // zero at 1514 // from 1200 to 2400
        analogWriteResolution(PREV_PWM_RESOLUTION); 
    }

    void test(int id) {
        if(!id) return;
        switch(id) {
          case 1:
            // Check if data is available on UART4
            if (Serial4.available() >= 22) { // Assuming a packet size of 10 bytes
                // Read the data from UART4 into a buffer
                uint8_t buffer[22];
                Serial4.readBytes(buffer, sizeof(buffer));

                // Calculate and verify the checksum
                uint8_t checksum = 0;
                for (int i = 0; i < 9; i++) {
                checksum += buffer[i];
                }

                if (checksum == buffer[9]) {
                // Extract data
                int temperature = buffer[0]; // First byte is temperature
                int voltage = *((uint16_t*)(buffer + 1)); // 2 bytes for voltage
                int current = *((uint16_t*)(buffer + 3)); // 2 bytes for current
                int consumption = *((uint16_t*)(buffer + 5)); // 2 bytes for consumption
                int rpm = *((uint16_t*)(buffer + 7)); // 2 bytes for RPM

                // Print the received data to the serial monitor
                Serial.print("Temperature: ");
                Serial.println(temperature);
                Serial.print("Voltage (mV): ");
                Serial.println(voltage);
                Serial.print("Current (mA): ");
                Serial.println(current);
                Serial.print("Consumption (mW): ");
                Serial.println(consumption);
                Serial.print("RPM: ");
                Serial.println(rpm);
                }
            }
            break;
          case 2:
            Serial.print("PWM PIN: ");
            Serial.println(PinPwm);

            set_unit_signal(0.11);
            break;
          case 3: {
            uint8_t buffer[45] = {0};
            int index1=0, index2=0;
            Serial.print("num bytes A:");
            // Serial.print(Serial4.read());
            Serial.println(Serial4.available());
            while(Serial4.available() > 44) {
                Serial.print("num bytes:");
                Serial.println(Serial4.available());
                while(index1 < 44 && !(buffer[index1] == 0xFF && buffer[index1+1] == 0xFF)) {
                    index1++;
                }
                index1 += 2;
                index2 = index1;
                while(index2 < 44 && !(buffer[index2] == 0xFF && buffer[index2+1] == 0xFF)) {
                    index2++;
                }
                uint8_t valid[22] = {0};
                for(int j = 0, i = index1; i < index2; j++, index1++) {
                    valid[j] = buffer[i];
                }
                int rpm = (int)((valid[11] << 24) + (valid[10] << 16) + (valid[9] << 8) + (valid[8] << 0)); //ERPM output
                Serial.print("rpm: ");
                Serial.println(rpm);
                index1 = index2 = 0;
                // Serial4.flush();  
                for(int i = 0; i < 45; i++) { Serial.print(buffer[i]); } Serial.println();
                break;                  
            }




            // while(Serial4.available() >= 2) {
                
            //     uint8_t two_bytes[2];
            //     Serial4.readBytes(two_bytes, sizeof(two_bytes));
            //     buffer[index++] = two_bytes[0];
            //     buffer[index++] = two_bytes[1];
            //     Serial.println(int((two_bytes[1] << 8) + two_bytes[0]));
            //     if(two_bytes[0] == 255 && two_bytes[1] == 255) {
            //         flag = true;
            //     }

            //     else if(two_bytes[0] == 255 || two_bytes[1] == 255)
            //     {
            //         uint8_t temp[1];
            //         Serial4.readBytes(temp, sizeof(temp));
            //         index = 0;
            //     }

            //     if(flag && index == 22)
            //     {
            //         // Parse the data
            //         int rpm = (int)((buffer[11] << 24) + (buffer[10] << 16) + (buffer[9] << 8) + (buffer[8] << 0)); //ERPM output
            //         Serial.print("rpm: ");
            //         Serial.println(rpm);
            //         flag = false;
            //         index = 0;
            //     }

            //     else if(flag)
            //     {
            //         index = 0;
            //         flag = false;
            //     }
            // }
            // Serial.println("waiting");
            // delay(300);
            break;
        }
          case 4: {
            uint8_t x[8];
            while(Serial4.available() > 0) {
                Serial4.readBytes(x, 8);
                for(int i = 0; i < 8; i++) {
                    Serial.print(x[i]);
                    Serial.print(" ");
                }
                Serial.println();
            }
            break;
          }
        }
        while(Serial.available() > 0) {
            int us = Serial.parseInt();
            Serial.println(us);
            // delay(1000);
            writeMicroseconds(us);
            break;
        }
    }
  private:
    const bool bidirectional;
    const RotDir direction;

     int zero, min_range, max_range;
};

#endif//BLDC_SPEED_CONTROLLER_120F3X