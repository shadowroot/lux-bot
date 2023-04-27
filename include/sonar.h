#include <config.h>

#ifdef HC_SR04_USE
#ifndef SONAR_H

class Sonar {
    public:
        Sonar(const int trigger_pin, const int echo_pin): trigger_pin(trigger_pin), echo_pin(echo_pin) {}
        void sonar_setup_hook(){
            pinMode(trigger_pin, OUTPUT);
            pinMode(echo_pin, INPUT);
        }
        int get_distance(){
            digitalWrite(trigger_pin, LOW);
            delayMicroseconds(2);
            digitalWrite(trigger_pin, HIGH);
            delayMicroseconds(10);
            digitalWrite(trigger_pin, LOW);
            int duration = pulseIn(echo_pin, HIGH);
            //34cm/ms
            int distance = (duration * 0.034) / 2; //cm 
            return distance;
        }
        void print_distance(){
            Serial.print("LOG: Distance: ");
            Serial.print(get_distance());
            Serial.println(" cm");
        }
        void loop_hook(){
            print_distance();
        }
    private:    
        const int trigger_pin;
        const int echo_pin;
        int distance;
};
#endif //SONAR_H

#endif //HC_SR04_USE