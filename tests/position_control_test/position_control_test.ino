#include <Arduino.h>
#include <Wire.h>
#include <smartmotor.h>

// MOTOR PROPERTIES
#define GEAR_RATIO 150           // MOTOR GEAR RATIO
#define ENCODER_TICKS_PER_REV 12 // NO. OF HIGH PULSES PER ROTATION
const int32_t ENCODER_TICKS_PER_SHAFT_REV= ENCODER_TICKS_PER_REV * GEAR_RATIO;
#define DELAY_PERIOD 10
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
            motors[i].tune_pos_pid(-1*kP, kI+0.05, 0);
      } else {
        motors[i].tune_pos_pid(kP+1.5, kI, 0);
      }
      
    }
}

int32_t target_vel = 30;
int32_t target_vel_neg = -30;
bool target = false;

void driveStraight(int32_t target_pos, int32_t target_pos_neg, int32_t check_val) {
  Serial.println("Target FALSE");
  target = false;
  while (!target){
    // Serial.println("Target FALSE");
    // FOR EACH MOTOR ADDRESS
    for (int i = 0; i < MOTOR_NUM; i++) {
        //motors[i].tune_pos_pid(kP, kI, 0);
        // Serial.print("Writing to motor: ");
        // Serial.println(motors[i].get_address());

        // Serial.print("Target position: ");
        // Serial.println(target_pos);
        uint8_t status;
        if ((motors[i].get_rpm() < 3) && (abs(motors[i].get_position()) > check_val)) {
          Serial.println("Target TRUE");
          target = true;
        }
          if (motors[i].get_address() == 12) {
            status= motors[i].set_position(target_pos_neg);
          } else {
            status= motors[i].set_position(target_pos);
          }
          
        // Serial.print("Status: ");
        // Serial.println(status);
 
        // READ MOTOR VARIABLES IF TRANSMISSION IS SUCCESSFUL
        if (status < 1) {
            delay(DELAY_PERIOD);

            // PRINT CURRENT POSITION
            // Serial.print("Position: ");
            // Serial.println(motors[i].get_position());
        }
        // Serial.println();
    }
    
    //target_pos+= ENCODER_TICKS_PER_SHAFT_REV;
    delay(DELAY_PERIOD);
  }
}

void turnTicks(int32_t target, int32_t target_neg) {
  // Serial.print("Target position: ");
  // Serial.println(target_pos);
  // Serial.print("Target Neg position: ");
  // Serial.println(target_pos_neg);

  // Adjust the target based on the input 
  // target_pos = target_pos - turn_val;
  // target_pos_neg = target_pos_neg - turn_val;
  // target_pos = 0 - turn_val;
  // target_pos_neg = 0 - turn_val;

  // Serial.print("Target position: ");
  // Serial.println(target_pos);
  // Serial.print("Target Neg position: ");
  // Serial.println(target_pos_neg);

  // bool turn = false;

  Serial.println("turn FALSE");
  while (!turn) {
    // FOR EACH MOTOR ADDRESS
    for (int i = 0; i < MOTOR_NUM; i++) {
        //motors[i].tune_pos_pid(kP, kI, 0);
        Serial.print("Writing to motor: ");
        Serial.println(motors[i].get_address());

        // Serial.print("Target position: ");
        // Serial.println(target);
        uint8_t status;
        if (motors[i].get_rpm() < 3) {
          turn = true;
          Serial.println("turn TRUE");
        }
          if (motors[i].get_address() == 12) {
            status= motors[i].set_position(target_neg);
            Serial.print("Target position: ");
            Serial.println(target_neg);
          } else {
            status= motors[i].set_position(target);
            Serial.print("Target position: ");
            Serial.println(target);
          }
          
        // Serial.print("Status: ");
        // Serial.println(status);
 
        // READ MOTOR VARIABLES IF TRANSMISSION IS SUCCESSFUL
        if (status < 1) {
            delay(DELAY_PERIOD);

            // PRINT CURRENT POSITION
            // Serial.print("Position: ");
            // Serial.println(motors[i].get_position());
        }
        // Serial.println();
    }
    
    //target_pos+= ENCODER_TICKS_PER_SHAFT_REV;
    delay(DELAY_PERIOD);
  }
}

void loop() {
  int32_t target_pos= 15000; // INIT TARGET DISPLACEMENT
  int32_t target_pos_neg = target_pos*-1;

  // Drive
  for (int i = 0; i < MOTOR_NUM; i++) {
    if (motors[i].get_address() == 12) {
      motors[i].set_position(target_pos_neg);
    } else {
      motors[i].set_position(target_pos);
    }
  }

  delay(11000);

  // now turn
  int32_t turn_ticks = 1468;
  target_pos = target_pos - turn_ticks;
  target_pos_neg = target_pos_neg - turn_ticks;

  for (int i = 0; i < MOTOR_NUM; i++) {
    if (motors[i].get_address() == 12) {
      motors[i].set_position(target_pos_neg);
    } else {
      motors[i].set_position(target_pos);
    }
  }

  delay(3000);

  // Now drive
  target_pos = target_pos + 15000;
  target_pos_neg = target_pos_neg - 15000;

  for (int i = 0; i < MOTOR_NUM; i++) {
    if (motors[i].get_address() == 12) {
      motors[i].set_position(target_pos_neg);
    } else {
      motors[i].set_position(target_pos);
    }
  }

  while(true);

  // int32_t target_pos= 15000; // INIT TARGET DISPLACEMENT
  // int32_t target_pos_neg = target_pos*-1;
  // Send the motor to drive straight
  driveStraight(target_pos, target_pos_neg, 1000);

  Serial.println("WAIT TO TURN");
  delay(15000);
  Serial.println("NOW TURN");

  // now turn
  turn_ticks = 1468;
  target_pos = target_pos - turn_ticks;
  target_pos_neg = target_pos_neg - turn_ticks;
  turnTicks(target_pos, target_pos_neg);
  
  Serial.println("WAIT TO DRIVE");
  delay(5000);
  Serial.println("NOW DRIVE");

  target_pos = target_pos + 15000;
  target_pos_neg = target_pos_neg - 15000;
  for (int i = 0; i < MOTOR_NUM; i++) {
    if (motors[i].get_address() == 12) {
      motors[i].set_position(target_pos_neg);
    } else {
      motors[i].set_position(target_pos);
    }
  }
  // driveStraight(target_pos, target_pos_neg, 17000);

  // Stop the function
  while(true);

    // FOR EACH MOTOR ADDRESS
    
    for (int i = 0; i < MOTOR_NUM; i++) {
        //motors[i].tune_pos_pid(kP, kI, 0);
        Serial.print("Writing to motor: ");
        Serial.println(motors[i].get_address());

        Serial.print("Target position: ");
        Serial.println(target_pos);
        uint8_t status;
        if (target_pos - motors[i].get_position() < abs(10)) {
          target = true;
        }
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
    }
    
    //target_pos+= ENCODER_TICKS_PER_SHAFT_REV;
    delay(DELAY_PERIOD);
} 
