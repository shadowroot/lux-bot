
// Adafruit Motor shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!
#include <config.h>
#include <AFMotor.h>

#ifdef MOTOR_SHIELD_USE

class LRMotor {
public:
  LRMotor(const int left_motor, const int right_motor): motor_left(left_motor), motor_right(right_motor) {}
  
  void motor_setup_hook(){
    motor_stop();
  }
  
  void move_left(int speed){
    motor_right.setSpeed(speed);
    motor_right.run(FORWARD);
    motor_left.run(RELEASE);
  }

  void move_right(int speed){
    motor_left.setSpeed(speed);
    motor_left.run(FORWARD);
    motor_right.run(RELEASE);
  }

  void rotate_left(int speed){
    motor_right.setSpeed(speed);
    motor_left.setSpeed(speed);
    motor_right.run(FORWARD);
    motor_left.run(BACKWARD);
  }

  void rotate_right(int speed){
    motor_right.setSpeed(speed);
    motor_left.setSpeed(speed);
    motor_right.run(BACKWARD);
    motor_left.run(FORWARD);
  }

  void motor_stop(){
    motor_left.run(RELEASE);
    motor_right.run(RELEASE);
  }

private:
  AF_DCMotor motor_left;
  AF_DCMotor motor_right;
}

#endif