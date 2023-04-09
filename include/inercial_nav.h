#include <config.h>
#include <Wire.h>
#include <MPU6050_light.h>

#ifdef MPU_6050_USE

/**
 * @brief Inercial Navigation class for MPU6050 and logging into Serial.
 * 
*/
class InercialNav{
    public:
        InercialNav(int update_rate_ms=500): mpu(Wire), update_rate_ms(update_rate_ms){};
        void setup_hook();
        void loop_hook();


    private:
        MPU6050 mpu;
        float current_rotation_pitch;
        float timer;
        float current_angle_x;
        float previous_angle_x;
        float previous_timer;
        int update_rate_ms;
};

void InercialNav::setup_hook(){
    Wire.begin();
    //not much sense, but it's init ... I'll leave it for now ... It's in constructor.
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

}

void InercialNav::loop_hook(){
  mpu.update();

  if(millis() - timer > update_rate_ms){ // update based on update_rate_ms
    Serial.print(F("TEMPERATURE: "));Serial.println(mpu.getTemp());
    Serial.print(F("ACCELERO  X: "));Serial.print(mpu.getAccX());
    Serial.print("\tY: ");Serial.print(mpu.getAccY());
    Serial.print("\tZ: ");Serial.println(mpu.getAccZ());
  
    Serial.print(F("GYRO      X: "));Serial.print(mpu.getGyroX());
    Serial.print("\tY: ");Serial.print(mpu.getGyroY());
    Serial.print("\tZ: ");Serial.println(mpu.getGyroZ());
  
    Serial.print(F("ACC ANGLE X: "));Serial.print(mpu.getAccAngleX());
    Serial.print("\tY: ");Serial.println(mpu.getAccAngleY());
    
    //Angles in degrees
    previous_timer = timer;
    previous_angle_x = current_angle_x;
    current_angle_x = mpu.getAngleX();

    Serial.print(F("ANGLE     X: "));Serial.print(current_angle_x);
    Serial.print("\tY: ");Serial.print(mpu.getAngleY());
    Serial.print("\tZ: ");Serial.println(mpu.getAngleZ());
    Serial.println(F("=====================================================\n"));
    timer = millis();
  }

}

#endif