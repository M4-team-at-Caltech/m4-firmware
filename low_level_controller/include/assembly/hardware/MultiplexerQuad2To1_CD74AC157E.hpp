#ifndef MULTIPLEXER_QUAD2TO1_CD74AC157E
#define MULTIPLEXER_QUAD2TO1_CD74AC157E

#include <Arduino.h>
#include <vector>

class MultiplexerQuad2To1_CD74AC157E {
  public:
    static const uint8_t IRRELEVANT_PIN = 255;
    const uint8_t PinSelect;
    const uint8_t Pin1A;        
    const uint8_t Pin1B;        
    const uint8_t Pin1Y;        
    const uint8_t Pin2A;        
    const uint8_t Pin2B;       
    const uint8_t Pin2Y;
    const uint8_t Pin3A;       
    const uint8_t Pin3B;       
    const uint8_t Pin3Y;       
    const uint8_t Pin4A;
    const uint8_t Pin4B;       
    const uint8_t Pin4Y;
    const uint8_t PinStrobe;

    MultiplexerQuad2To1_CD74AC157E(std::vector<uint8_t> pins)
    : PinSelect{pins[0]}, Pin1A{pins[1]}, Pin1B{pins[2]}, Pin1Y{pins[3]},
      Pin2A{pins[4]}, Pin2B{pins[5]}, Pin2Y{pins[6]}, 
      Pin3A{pins[7]}, Pin3B{pins[8]}, Pin3Y{pins[9]}, 
      Pin4A{pins[10]}, Pin4B{pins[11]}, Pin4Y{pins[12]}, PinStrobe{pins[13]} {
        for(auto& pin : pins) pinMode(pin, OUTPUT);
      }
    ~MultiplexerQuad2To1_CD74AC157E() = default;

    void select_channel(uint8_t channel) {
        // channel assignment: 0 -> A, 1 -> B
        digitalWrite(PinSelect, channel);
    }
    void ground_signals(bool state) {
        // set all pins to low value if state is high, otherwise neutral
        digitalWrite(PinStrobe, state);
    }
};

#endif//MULTIPLEXER_QUAD2TO1_CD74AC157E