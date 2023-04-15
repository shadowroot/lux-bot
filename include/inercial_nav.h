#include <config.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <motor.h>
#include <math.h>
#include <stuck.h>

#ifdef MPU_6050_USE

#ifndef INERCIAL_NAV_H
#define INERCIAL_NAV_H
class Vector2D{
    public:
        Vector2D(float x, float y): x(x), y(y) {}
        float x;
        float y;
};
class WayPoint{
    public:
        WayPoint(): x(0), y(0) {}
        WayPoint(float x, float y): x(x), y(y) {}
        void addVector(const Vector2D& vector);
        float x;
        float y;
};

void WayPoint::addVector(const Vector2D& vector){
    x += vector.x;
    y += vector.y;
}
class Path{
    public:
        Path();
        void addWaypoint(const WayPoint& waypoint);
        void addVector(const Vector2D& vector);
        Vector2D planPath(const WayPoint& waypoint);
        WayPoint getCurrentWaypoint();
    private:
        int current_waypoint;
        //3600 seconds of waypoints
        WayPoint waypoints[3600];
        //100 ms
        WayPoint waypoint_next_second;
        int waypoint_next_second_counter;
};

Path::Path(): current_waypoint(0), waypoint_next_second_counter(0) {
    for(int i=0; i<3600; i++){
        waypoints[i] = WayPoint(0,0);
    }
    waypoint_next_second = WayPoint(0,0);
}

void Path::addWaypoint(const WayPoint& waypoint){
  waypoints[current_waypoint] = waypoint;
  current_waypoint++;
  if(current_waypoint >= 3600){
    current_waypoint = 0;
  }
}

void Path::addVector(const Vector2D& vector){
  if(waypoint_next_second_counter < 10){
    waypoint_next_second.addVector(vector);
  }else{
    addWaypoint(waypoint_next_second);
    waypoint_next_second_counter = 0;
  }
}

WayPoint Path::getCurrentWaypoint(){
  return waypoint_next_second;
}

Vector2D Path::planPath(const WayPoint& waypoint){
  Vector2D vector(waypoint.x - waypoints[current_waypoint-1].x, waypoint.y - waypoints[current_waypoint-1].y);
  return vector;
}

enum Movement{
    MOVING_FORWARD,
    MOVING_BACKWARD,
    ROTATING_LEFT,
    ROTATING_RIGHT,
    STOPPED
};

/**
 * @brief Inercial Navigation class for MPU6050 and logging into Serial.
 * 
*/
class InercialNav{
    public:
        InercialNav(int update_rate_ms=100);
        void setup_hook();
        void loop_hook();
        Vector2D vectorTraveled();
        void moveForward(float distance);
        void moveBackward(float distance);
        void rotateLeft(float angle);
        void rotateRight(float angle);
        void rotate(float angle);
        void home();
        void travelToVector(const Vector2D& vector);
        double calcAngleDeg(const Vector2D& vector);
        double calcDistance(const Vector2D& vector);
        int detectStuck();
        int detectSwitchStuck();
        void stuckAvoidance();
        void sweep();
        void hover();
    private:
        MPU6050 mpu;
        float current_rotation_pitch;
        float timer;
        float current_angle_x;
        float previous_angle_x;
        float total_angle_x;
        float acceleration_x;
        float acceleration_y;
        float acceleration_z;
        float previous_velocity_x;
        float velocity_y;
        float previous_timer;
        int update_rate_ms;
        float time_delta;
        float t_squared;
        float minimal_move_distance;
        float current_distance;
        float previous_distance;
        float previous_angle;
        float current_angle;
        Movement state;
        int previous_stuck;

        Path path;
        LRMotor motor;
        HoverMotor hover_motor;
        BrushMotor brush_motor;
        #ifdef SWITCH_FRONT_LEFT_USE
        Stuck stuck_front_left; //1
        #endif
        #ifdef SWITCH_FRONT_RIGHT_USE
        Stuck stuck_front_right; //2
        #endif
};

InercialNav::InercialNav(int update_rate_ms): mpu(Wire), update_rate_ms(update_rate_ms), minimal_move_distance(0.1), stuck_front_left(12, "front_left"), stuck_front_right(13, "front_right"){
    time_delta = update_rate_ms/1000;
    t_squared = time_delta*time_delta;
}

void InercialNav::setup_hook(){
    Wire.begin();
    //not much sense, but it's init ... I'll leave it for now ... It's in constructor.
    //Default setup 2g and 500 deg/s
    MPU6050 mpu(Wire);

    long timer = 0;
    
    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    while(status!=0){ } // stop everything if could not connect to MPU6050
    
    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(1000);
    mpu.calcOffsets(true,true); // gyro and accelero
    Serial.println("Done!\n");
    #ifdef SWITCH_FRONT_LEFT_USE
      stuck_front_left.setup_hook();
    #endif
    #ifdef SWITCH_FRONT_RIGHT_USE
      stuck_front_right.setup_hook();
    #endif
    motor.setup_hook();
    hover_motor.setup_hook();
    brush_motor.setup_hook();
}

void InercialNav::loop_hook(){
  mpu.update();
  //Accelerometer values
  //10m/s^2
  // 0 - -2g
  // 4 - +2g
  //position updater and if position job is running, then functions for that.
  if((millis() - timer) > update_rate_ms){ // update based on update_rate_ms
    acceleration_x = mpu.getAccX();
    acceleration_y = mpu.getAccY();
    acceleration_z = mpu.getAccZ();

    Serial.print(F("LOG:TEMPERATURE: "));Serial.println(mpu.getTemp());
    Serial.print(F("ACCEL  X: "));Serial.print(acceleration_x);
    Serial.print("\tY: ");Serial.print(acceleration_y);
    Serial.print("\tZ: ");Serial.println(acceleration_z);
  
    Serial.print(F("GYRO      X: "));Serial.print(mpu.getGyroX());
    Serial.print("\tY: ");Serial.print(mpu.getGyroY());
    Serial.print("\tZ: ");Serial.println(mpu.getGyroZ());
  
    Serial.print(F("ACC ANGLE X: "));Serial.print(mpu.getAccAngleX());
    Serial.print("\tY: ");Serial.println(mpu.getAccAngleY());
    
    //Angles in degrees
    previous_timer = timer;
    previous_angle_x = current_angle_x;
    current_angle_x = mpu.getAngleX();
    //total_angle_x = current_angle_x+previous_angle_x;
    current_distance = calcDistance(vectorTraveled());

    stuckAvoidance();

    Serial.print(F("ANGLE     X: "));Serial.print(current_angle_x);
    Serial.print("\tY: ");Serial.print(mpu.getAngleY());
    Serial.print("\tZ: ");Serial.println(mpu.getAngleZ());
    //Serial.println(F("=====================================================\n"));
    
    previous_distance = current_distance;
    previous_angle = current_angle_x;
    //add next vector to waypoints
    path.addVector(vectorTraveled());

    timer = millis();
  }

}

Vector2D InercialNav::vectorTraveled(){
  // distance traveled = 1/2at^2
  //Accelerometer values
  //10m/s^2
  // 0 - -2g
  // 4 - +2g
  float g2 = 20.0f;//m*s^2
  float ax = (acceleration_x - 2) * g2;
  float ay = (acceleration_y - 2) * g2;
  return Vector2D(ax*t_squared, ay*t_squared);
}

void InercialNav::rotateLeft(float angle){
  state = ROTATING_LEFT;
  motor.rotate_left(255);
  while(current_angle_x > angle){
    loop_hook();
  }
  motor.motor_stop();
  state = STOPPED;
}

void InercialNav::rotateRight(float angle){
  state = ROTATING_RIGHT;
  motor.rotate_right(255);
  while(current_angle_x < angle){
    loop_hook();
  }
  motor.motor_stop();
  state = STOPPED;
}

void InercialNav::rotate(float angle){
  if(angle > current_angle_x){
    rotateLeft(angle);
  }else{
    rotateRight(angle);
  }
}

void InercialNav::moveForward(float distance){
  state = MOVING_FORWARD;
  motor.move_forward(255);
  current_distance = 0;
  previous_distance = 0;
  while(current_distance < distance){
    stuckAvoidance();
    loop_hook();
    previous_distance = current_distance;
    current_distance += calcDistance(vectorTraveled());
  }
  motor.motor_stop();
  state = STOPPED;
}

void InercialNav::stuckAvoidance(){
    Movement current_state = state;
    int stuck = detectStuck();
    switch (stuck)
    {
      case 1:
        //try rotate left
        rotate(current_angle_x + 90);
        break;
      case 2:
        //try rotate right
        rotate(current_angle_x - 90);
        break;
      case 3:
        //try move backward
        moveBackward(minimal_move_distance);
        break;
      //default:
      //  break;
      case 4:
        //try move forward
        moveForward(minimal_move_distance);
        break;
    }
    state = current_state;
    previous_stuck = stuck;
}



void InercialNav::moveBackward(float distance){
  state = MOVING_BACKWARD;
  motor.move_backward(255);
  float current_distance = 0;
  while(current_distance < distance){
    loop_hook();
    current_distance += calcDistance(vectorTraveled());
  }
  motor.motor_stop();
  state = STOPPED;
}

double InercialNav::calcDistance(const Vector2D& vector){
  return sqrt(vector.x*vector.x + vector.y*vector.y);
}

void InercialNav::home(){
  Vector2D direction = path.planPath(WayPoint(0,0));
  travelToVector(direction);
}

double InercialNav::calcAngleDeg(const Vector2D& vector){
  double rads = atan2(vector.y, vector.x);
  if(vector.y < 0){
    rads += PI;
  }
  return 57.2958 * rads;
}

void InercialNav::travelToVector(const Vector2D& vector){
  //rotation part
  double angle = calcAngleDeg(vector);
  if((angle - current_angle_x) > 0){
    rotateLeft(angle);
  }else{
    rotateRight(angle);
  }
  //movement part
  moveForward(calcDistance(vector));
}

int InercialNav::detectStuck(){
  int stuck = 0;
  stuck = detectSwitchStuck();
  switch (state)
  {
  case MOVING_FORWARD:
    if(previous_distance == current_distance){
      //rotate the same way
      stuck = 1;
    }
    break;
  case MOVING_BACKWARD:
    if(previous_distance == current_distance){
      stuck = 1;
    }
    break;
  case ROTATING_LEFT:
    if(previous_angle == current_angle_x){
      stuck = 3;
    }
    break;
  case ROTATING_RIGHT:
    if(previous_angle == current_angle_x){
      stuck = 3;
    }
    break;
  default:
    break;
  }
  if(previous_stuck >= 3){
    //was stucked before? - avoid being stucked in different way
    stuck += 1;
  }
  return stuck;
}

int InercialNav::detectSwitchStuck(){
  int stuck = 0;
  #ifdef SWITCH_FRONT_LEFT_USE
    if(stuck_front_left.is_stuck()){
      stuck = 1;
    }
  #endif
  #ifdef SWITCH_FRONT_RIGHT_USE
    if(stuck_front_right.is_stuck()){
      stuck += 2;
    }
  #endif
  return stuck;
}

void InercialNav::sweep(){
  while(detectStuck() == 0){
    moveForward(minimal_move_distance);
  }
}

void InercialNav::hover(){
  hover_motor.hover();
  sweep();
  hover_motor.hover_stop();
}

#endif // INERCIAL_NAV_H

#endif // MPU_6050_USE