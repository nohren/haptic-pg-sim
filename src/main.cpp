#include <Arduino.h>
#include <SimpleFOC.h>
#include "main.h"

int PWM_A_0 = 5;
int PWM_B_0 = 9;
int PWM_C_0 = 6;
int ENABLE_0 = 8;
int ENCODER_A_0 = 2;
int ENCODER_B_0 = 3;
int PPR = 2048;
int pole_pairs = 11;
// float phase_resistance = 11.1; //don't know if this is correct
// SensorState sv; //  allocated to stack... not needed right now

//-------------------------SIMULATION-------------------------------
SimulationState sim_state;

unsigned long stateStartTime = 0; // track time we entered the current state
bool phaseComplete = false;
//float currentPosition_0 = 0.0;
float targetPosition_0 = 0.0;
//float currentVelocity_0 = 0.0;
float targetVelocity_0 = 0.0;
//float currentPosition_1 = 0.0;
float targetPosition_1 = 0.0;
//float currentVelocity_1 = 0.0;
float targetVelocity_1 = 0.0;

//motor movement 
float targetAngle = 0.0;


//------------------------------ENCODER--------------------------
Encoder encoder = Encoder(ENCODER_A_0, ENCODER_B_0, PPR);


//----------------------------------MOTOR-----------------------
//SensorState sensorPointer = &sensorState; 
//8 is enable, these pin numbers are on the back of the motor driver corresponding to a,b,c
BLDCDriver3PWM driver = BLDCDriver3PWM(PWM_A_0, PWM_B_0, PWM_C_0, ENABLE_0);


//uses up a lot of memory and not working for the params.  Too much to learn for too short a time!
//InlineCurrentSense current_sense  = InlineCurrentSense(0.01, 20, A0, A1, A2);

// motor info!
// GM4108H
// https://shop.iflight.com/gimbal-motors-cat44/ipower-motor-gm4108h-120t-brushless-gimbal-motor-pro217?srsltid=AfmBOoqxeW2Hb-j6NUhU7VB4WXt1Kw264wGX11YsdpgPQk_aYoP7Nqlo
// 22 poles, 11 poll pairs
// Resistance(Ω): 11.1Ω±5%
// Load Current (A): 1.5A
// diameter 47mm or 0.047m
// radius 0.0235
// circumference 0.1476 or 2pi radians
// 6.77 full rotations or 42.56 radians

BLDCMotor motor = BLDCMotor(pole_pairs);

Commander command = Commander(Serial); //remove to save memory
void doMotor(char* cmd) { command.motor(&motor, cmd); }
void doAngle(char* cmd) { command.scalar(&targetAngle, cmd); }


void setup() {
  randomSeed(analogRead(A0));
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);
  encoder.init(); 
  encoder.enableInterrupts(doA, doB); 
  motor.linkSensor(&encoder); 
  driver.voltage_power_supply = 11.1;
  driver.voltage_limit = 11.1; //amps = voltage / ohms  11.1v battery / 11 ohms = 1amp < 1.5 amp ... we good

  if(!driver.init()) {
    Serial.println("Driver init failed!");
    return;
  }
  motor.linkDriver(&driver);

  motor.torque_controller = TorqueControlType::voltage; 
  motor.controller = MotionControlType::torque;
  //PID default 0.05,10,0.001
  // motor.PID_velocity.P = 0.1; //proportional gain - used to correct current and target angle/velocity. higher is more aggressive.
  // motor.PID_velocity.I = 10; //for correcting velocity error
  // motor.PID_velocity.D = 0.001; //damps sudden changes to angle velocity
  // // jerk control using voltage voltage ramp
  // // default value is 300 volts per sec  ~ 0.3V per millisecond
  // motor.PID_velocity.output_ramp = 1000;

  // motor.P_angle.P = 5;   // angle P controller -  default P=20
  // // velocity low pass filtering
  // // default 5ms - try different values to see what is the best. 
  // // the lower the less filtered
  // motor.LPF_velocity.Tf = 0.01;
  // motor.velocity_limit = targetVelocity_0; //maximal velocity of the position control

  //key for reducing torque!
  motor.voltage_limit = 3;

  if(!motor.init()){
    Serial.println("Motor init failed!");
    return;
  }
  if(!motor.initFOC()){
    Serial.println("FOC init failed!");
    return;
  }
 
  command.add('M', doMotor, "Motor");
  command.add('A', doAngle, "angle");
  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target using serial terminal and command M:"));
  _delay(1000);
}

void loop() {
  // if (sim_state.current != sim_state.previous) {
  //   stateStartTime = micros();
  //   sim_state.previous = sim_state.current;

  //   if (sim_state.current == Simulation::RANDOM_NOISE) {
  //     sim_state.random_noise_rand = randomRange(5, 26);
  //   }
  // }
  
  // if (motor.velocity_limit != targetVelocity_0) {
  //   motor.velocity_limit = targetVelocity_0;
  // }

  // switch (sim_state.current){
  //   case Simulation::IDLE:
  //     // perceive/detect inputs & set state to calibration
  //     if (micros() - stateStartTime > 1000.0*1e6) {
  //       //Serial.println("timer finished");
  //       //updatePositionAndSpeed(6.28, 4, MotorID::MOTOR_ONE, MountSide::RIGHT);
  //       //sim_state.current = Simulation::RANDOM_NOISE;
  //     }
  //     if (millis() - stateStartTime > 40000) {
  //       // Serial.println("update position -6.28");
  //       // updatePositionAndSpeed(-6.28, 1, MotorID::MOTOR_ONE, MountSide::RIGHT);
  //       //sim_state.current = Simulation::RANDOM_NOISE;
  //     }
  //     break;
  //   case Simulation::CALIBRATION:
  //   Serial.println("calibration");
  //     // 5 seconds of no perceived inputs
  //     if (millis() - stateStartTime > 5000) {
  //       sim_state.current = Simulation::RANDOM_NOISE;
  //     }
  //     break;
  //   case Simulation::RANDOM_NOISE:
  //     // implemente random noise of the break (oren's hands shaked)
  //     if (millis() - stateStartTime > sim_state.random_noise_rand * 1000) {
  //       sim_state.current = Simulation::INCIDENT;
  //     }

  //     break;
  //   case Simulation::INCIDENT:
  //     // change target position value  
  //     break;
  //   case Simulation::RESPONSE:
  //   if (millis() - stateStartTime > 3000) {
  //     if (false) { // perceived inputs to declare success

  //     } else {
  //       sim_state.current = Simulation::RANDOM_NOISE;
  //     }
  //   } 
  //     break;
  //   case Simulation::INVALID:
  //     break;
  //   default:
  //     // weird simulation state detected
  //     break;
  //   }
  // encoder.update();
  // Serial.print(encoder.getAngle());
  // Serial.print("\t");
  // Serial.println(encoder.getVelocity());
  updateStuff();
  motor.loopFOC();
  motor.move();
  //motor.monitor();
  // user communication
  //command.run();
}

void doA(){encoder.handleA();}
void doB(){encoder.handleB();}
// void doIndex(){encoder.handleIndex();}

void updatePositionAndSpeed(float targetPosition, float targetVelocity, MotorID motor, MountSide side) {
  if (motor == MotorID::MOTOR_ZERO) {
    targetVelocity_0 = targetVelocity;
    targetPosition_0 = side == MountSide::LEFT ? -targetPosition: targetPosition;
    //convert targetPosition meters to radians - 2pi radians or 6.28 is approx equal to 144mm
    //motor.controller = MotionControlType::angle;
    //motor.velocity_limit = someMaxSpeed;
    //adjust 
  } else {
    targetVelocity_1 = targetVelocity;
    targetPosition_1 = side == MountSide::LEFT ? -targetPosition: targetPosition;
  }
}

// this function is designed to sit in the update, update values and apply forces as needed
void move(float targetPosition, float currentPosition, float targetVelocity, float currentVelocity, MotorID motor, MountSide side) {
  return;
  //need circumerfence length in meters which corresponds to 2pi radians
  //1 rotation is 147.65 mm
  
}

/*
  start - inclusive
  end - exclusive
*/
unsigned int randomRange(int start, int end) {
  return random(start, end);
}

MotorID selectMotor() {
  return static_cast<MotorID>( randomRange(0,2) );
}

void updateStuff() {
  encoder.update();
  float currentAngle = encoder.getAngle();
  float currentVelocity = encoder.getVelocity();
  // Serial.print(currentAngle);
  // Serial.print("\t");
  // Serial.println();

  //if current angle is not equal to target angle, go to target angle
  //positive voltage is CCW
  //negative is CW
  if (abs(currentAngle - targetAngle) > 0.5) {
    if (targetAngle < currentAngle) {
      motor.target = -2;
    } else {
      motor.target = 2;
    }
  } else {
    motor.target = 0;
  }
}
