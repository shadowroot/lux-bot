#include <config.h>
#include <Arduino.h>
#include <simpleRPC.h>
#include <inercial_nav.h>

#ifndef CMDS_H
#define CMDS_H

class CMDS{
    public:
        CMDS(): inercial_nav(lr_motor,  hover_motor, brush_motor), timer(0){}
        void setup_hook(void) {
            Serial.begin(9600);
            if(DEBUG) Serial.println(F("LOG: Starting..."));
            lr_motor.setup_hook();
            hover_motor.setup_hook();
            brush_motor.setup_hook();
            inercial_nav.setup_hook();
        }

        void loop_hook(void) {
            inercial_nav.loop_hook();

            if(DEBUG && (millis() - timer > 1000)){
                Serial.println(F("LOG: Waiting for command..."));
                timer = millis();
            }
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
            interface(
                Serial, 
                pack(&brush_motor, &BrushMotor::brush), F("Run brush motor. @speed : Value in 0-255")
                );
            interface(
                Serial, 
                pack(&brush_motor, &BrushMotor::stop), F("Stop brush motor.")
                );
            interface(
                Serial, 
                pack(&hover_motor, &HoverMotor::hover), F("Run hover motor. @speed : Value in 0-255")
                );
            interface(
                Serial, 
                pack(&hover_motor, &HoverMotor::hover_stop), F("Stop hover motor.")
                );
            interface(
                Serial, 
                pack(&inercial_nav, &InercialNav::randomHover), F("randomHover: Hover bot in random direction.")
                );
            interface(
                Serial, 
                pack(&inercial_nav, &InercialNav::randomSweep), F("randomSweep: Sweep bot in random direction.")
                );
            interface(
                Serial, 
                pack(&inercial_nav, &InercialNav::stop), F("Stop: Stop bot.")
                );
        }
    private:
        InercialNav inercial_nav;
        LRMotor lr_motor;
        BrushMotor brush_motor;
        HoverMotor hover_motor;
        long timer;
};

#endif //CMDS_H