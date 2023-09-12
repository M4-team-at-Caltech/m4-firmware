#ifndef BLDC_SPEED_CONTROLLER_120F3X
#define BLDC_SPEED_CONTROLLER_120F3X

#include <Arduino.h>
#include <vector>
#include <math.h>

#include "hardware/assembly/definitions.hpp"
#include "utilities.hpp"

using RotDir = Sign;

class BldcSpeedController_120F3X : public PwmController {
float f500 = 500., f400 = 400.;
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

  public:

    BldcSpeedController_120F3X(const PinVector& pins, bool bidirectional, RotDir direction, int trim=0, float safety_factor=1)
    : PinPwm{pins[0]}, Telemetry{nullptr}, bidirectional{bidirectional}, direction{direction}, trim{trim},
      zero{bidirectional ? PWM_MID_TCKS : PWM_MIN_TCKS}, min_range{PWM_MIN_TCKS - zero}, max_range{PWM_MAX_TCKS - zero} {
        for(auto& pin : pins) pinMode(pin, OUTPUT);
        // analogWriteFrequency(PinPwm, PWM_FREQUENCY); // pulse width modulation with 2ms period
        freq_multiplier = (bidirectional ? f500 : f400) / PWM_FREQUENCY;
        lower_safety_limit = freq_multiplier*(zero + safety_factor*min_range);
        upper_safety_limit = freq_multiplier*(zero + safety_factor*max_range);
        if(bidirectional) {
            // zero = map(0.15*trim, -100., 100., PWM_MIN_TCKS, PWM_MAX_TCKS); //! 0.15 is the max speed of interface
            analogWriteFrequency(PinPwm, f500);
        } else {
            // zero = map(900, 1000, 2000, PWM_MIN_TCKS, PWM_MAX_TCKS);
            analogWriteFrequency(PinPwm, f400);
        }
        

        uint32_t PREV_PWM_RESOLUTION = analogWriteResolution(PWM_RESOLUTION);   
        analogWrite(PinPwm, zero);
        analogWriteResolution(PREV_PWM_RESOLUTION); 
        #ifdef DEBUG
        init();
        #endif
    }
    BldcSpeedController_120F3X(const PinVector& pins, HardwareSerial& telemetry, bool bidirectional, RotDir direction, int trim=0, float safety_factor=1)
    : PinPwm{pins[0]}, Telemetry{&telemetry}, bidirectional{bidirectional}, direction{direction}, trim{trim},
      zero{bidirectional ? PWM_MID_TCKS : PWM_MIN_TCKS}, min_range{PWM_MIN_TCKS - zero}, max_range{PWM_MAX_TCKS - zero} {
        for(auto& pin : pins) pinMode(pin, OUTPUT);
        freq_multiplier = (bidirectional ? f500 : f400) / PWM_FREQUENCY;
        lower_safety_limit = freq_multiplier*(zero + safety_factor*min_range);
        upper_safety_limit = freq_multiplier*(zero + safety_factor*max_range);
        if(bidirectional) {
            // zero = map(0.15*trim, -100., 100., PWM_MIN_TCKS, PWM_MAX_TCKS); //! 0.15 is the max speed of interface
            analogWriteFrequency(PinPwm, f500);
        } else {
            // zero = map(900, 1000, 2000, PWM_MIN_TCKS, PWM_MAX_TCKS);
            analogWriteFrequency(PinPwm, f400);
        }


        uint32_t PREV_PWM_RESOLUTION = analogWriteResolution(PWM_RESOLUTION);   
        analogWrite(PinPwm, zero);
        analogWriteResolution(PREV_PWM_RESOLUTION); 
        #ifdef DEBUG
        init();
        #endif
    }
    ~BldcSpeedController_120F3X() = default;


    void init() {
        Serial.print("Pin ");
        Serial.print(PinPwm);
        Serial.print(" @ ");
        Serial.print(bidirectional ? f500 : f400);
        Serial.print("Hz: zero = ");
        Serial.print(zero);
        Serial.print(", min_range = ");
        Serial.print(zero + min_range);
        Serial.print(" (lim: ");
        Serial.print(lower_safety_limit);
        Serial.print("), max_range = ");
        Serial.print(zero + max_range);
        Serial.print(" (lim: ");
        Serial.print(upper_safety_limit);
        Serial.print(")");
        Serial.println();
    }
    virtual void set_unit_signal(float value) {  
        uint32_t PREV_PWM_RESOLUTION = analogWriteResolution(PWM_RESOLUTION);   
        float pwm_signal = zero;
        if(bidirectional) {
            value = constrain(value, -1., +1.);
            pwm_signal += map(value, -1., +1., min_range, max_range) * direction;
        } else {
            value = constrain(value, 0., +1.);
            pwm_signal += map(value, 0., +1., min_range, max_range);
            pwm_signal *= freq_multiplier;
        }
        #ifdef DEBUG
        Serial.print("Pin ");
        Serial.print(PinPwm);
        Serial.print(" SET ");
        Serial.print(value);
        Serial.print(" [-1,+1] --> ");
        Serial.print(pwm_signal);
        Serial.print(" [TCKS] is ");
        Serial.print(map(pwm_signal, PWM_MIN_TCKS, PWM_MAX_TCKS, bidirectional?1000:1250, bidirectional?2000:2500));
        Serial.print(" us");
        Serial.println();
        #endif

        pwm_signal = constrain(pwm_signal, lower_safety_limit, upper_safety_limit);
        analogWrite(PinPwm, pwm_signal);

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

    int trim;      // from experimentation
    int zero, min_range, max_range;
    float freq_multiplier;
    float lower_safety_limit, upper_safety_limit; // factor [0-1] of maximum
};

#endif//BLDC_SPEED_CONTROLLER_120F3X