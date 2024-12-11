#include <Arduino.h>
#include <Wire.h>
#include <smartmotor.h>

// MOTOR PROPERTIES
#define GEAR_RATIO 150           // MOTOR GEAR RATIO
#define ENCODER_TICKS_PER_REV 12 // NO. OF HIGH PULSES PER ROTATION
const int32_t ENCODER_TICKS_PER_SHAFT_REV= ENCODER_TICKS_PER_REV * GEAR_RATIO;
#define DELAY_PERIOD 0
#define WHEEL_DIAMETER 57

// INIT SMART MOTORS
SmartMotor motors[] = {0x0C, 0x0B}; // INIT MOTOR W/ DEFAULT ADDRESS
// SmartMotor motors[] = {0x05,0x06,0x07}; // INIT MOTOR W/ DEFAULT ADDRESS
const int MOTOR_NUM = sizeof(motors)/sizeof(motors[0]);

int metersToTicks = (1000/(WHEEL_DIAMETER*3.1415926))*ENCODER_TICKS_PER_SHAFT_REV;
int ticksPerCm = 141.6;
int ticksPerM = 14164;
bool turn = false;
float kP = 15;
float kI = 1;

void setup() {
    Serial.begin(115200);
    Wire.begin(); // INIT DEVICE AS I2C CONTROLLER

    for (int i = 0; i < MOTOR_NUM; i++) {
      motors[i].reset();
      if (motors[i].get_address() == 12) {
            motors[i].tune_pos_pid(-1*kP, kI, 0);
      } else {
        motors[i].tune_pos_pid(kP, kI, 0);
      }
      
    }
}

int32_t target_pos= 15000; // INIT TARGET DISPLACEMENT
int32_t turn_amount = 734;
int32_t target_pos_neg = target_pos*-1;
int32_t target_vel = 30;
int32_t target_vel_neg = -30;
bool target = false;

void loop() {
    // FOR EACH MOTOR ADDRESS

      for (int i = 0; i < MOTOR_NUM; i++) {
          //motors[i].tune_pos_pid(kP, kI, 0);
          Serial.print("Writing to motor: ");
          Serial.println(motors[i].get_address());

          Serial.print("Target position: ");
          Serial.println(target_pos);
          uint8_t status;
            if (motors[i].get_address() == 12) {
              status= motors[i].set_position(target_pos_neg);
            } else {
              status= motors[i].set_position(target_pos);
            }
            
          Serial.print("Status: ");
          Serial.println(status);
  
          // READ MOTOR VARIABLES IF TRANSMISSION IS SUCCESSFUL
          if (status < 1) {
              delay(DELAY_PERIOD);

              // PRINT CURRENT POSITION
              Serial.print("Position: ");
              Serial.println(motors[i].get_position());
          }
          Serial.println();
          //if (abs(target_pos - motors[i].get_position()) < 100) {
          //  turn = true;
          //}
      }

    //target_pos+= ENCODER_TICKS_PER_SHAFT_REV;
    delay(DELAY_PERIOD);
} 