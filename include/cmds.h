#include <config.h>
#include <Arduino.h>
#include <simpleRPC.h>
#include <inercial_nav.h>

#ifndef CMDS_H
#define CMDS_H

class CMDS{
    public:
        CMDS(){}
        void setup_hook(void) {
            Serial.begin(9600);
        }

        void loop_hook(void) {
            interface(
                Serial,
                pack(&inercial_nav, &InercialNav::moveForward), F("moveForward: Move bot forward. @distance : Value in m")
            );
            interface(
                Serial, 
                pack(&inercial_nav, &InercialNav::rotateLeft), F("rotateLeft: Rotate bot left. @angle : Value in degrees")
            );
            interface(
                Serial, 
                pack(&inercial_nav, &InercialNav::rotateRight), F("rotateRight: Rotate bot right. @angle : Value in degrees")
                );
        }
    private:
        InercialNav inercial_nav;
};

#endif //CMDS_H