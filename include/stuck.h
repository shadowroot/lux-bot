#include <config.h>

#ifndef STUCK_H
#define STUCK_H
class Stuck {
    //pullup resistor
    public:
        Stuck(const int pin, const char * name): pin(pin) {}
        void setup_hook(){
            pinMode(pin, INPUT_PULLUP);
        }
        bool is_stuck(){
            return digitalRead(pin) == LOW;
        }
        void loop_hook(){
            if(is_stuck()){
                Serial.print("LOG:Stuck ");
                Serial.println(name);
            }
        }
    private:
        const int pin;
        const char * name;
}; 

#endif //STUCK_H