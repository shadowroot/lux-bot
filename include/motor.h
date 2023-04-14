
// Adafruit Motor shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!
#include <config.h>
#include <AFMotor.h>

#ifdef MOTOR_SHIELD_USE

/**
 * @brief Motor class for left and right motor for movement.
 * 
*/
class LRMotor {
public:
  LRMotor(const int left_motor_channel=1, const int right_motor_channel=2): motor_left(left_motor_channel), motor_right(right_motor_channel) {}
  
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

  void move_forward(int speed){
    motor_right.setSpeed(speed);
    motor_left.setSpeed(speed);
    motor_right.run(FORWARD);
    motor_left.run(FORWARD);
  }

  void move_backward(int speed){
    motor_right.setSpeed(speed);
    motor_left.setSpeed(speed);
    motor_right.run(BACKWARD);
    motor_left.run(BACKWARD);
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
};

class BrushMotor{
  public:
    BrushMotor(const int motor_channel=3): brushMotor(motor_channel) {}
    
    void motor_setup_hook(){
      stop();
    }
    
    void brush(int speed=255){
      brushMotor.setSpeed(speed);
      brushMotor.run(FORWARD);
    }

    void stop(){
      brushMotor.run(RELEASE);
    }
  private:
    AF_DCMotor brushMotor;
};

class HoverMotor{
  public:
    HoverMotor(const int motor_channel=4): hoverMotor(motor_channel) {}
    
    void motor_setup_hook(){
      hover_stop();
    }
    
    void hover(int speed=255){
      hoverMotor.setSpeed(speed);
      hoverMotor.run(FORWARD);
    }

    void hover_stop(){
      hoverMotor.run(RELEASE);
    }
  private:
    AF_DCMotor hoverMotor;
};

#endif