#include <Arduino.h>
#include "cmds.h"

CMDS cmds;

void setup() {
  // put your setup code here, to run once:
  cmds.setup_hook();
}

void loop() {
  // put your main code here, to run repeatedly:
  cmds.loop_hook();
}