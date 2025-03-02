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
float targetVoltage = 0.0;


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

  // Handle and initialize encoder
  encoder.init(); 
  encoder.enableInterrupts(doA, doB); 

  // Handle and initialize driver
  driver.voltage_power_supply = 11.1;
  driver.voltage_limit = 11.1; //amps = voltage / ohms  11.1v battery / 11 ohms = 1amp < 1.5 amp ... we good

  if(!driver.init()) {
    Serial.println("Driver init failed!");
    return;
  }
  
  // Handle and initialize motor
  motor.torque_controller = TorqueControlType::voltage; 
  motor.controller = MotionControlType::angle;
  
  //PID default 0.05,10,0.001
  motor.PID_velocity.P = 0.1; //proportional gain - used to correct current and target angle/velocity. higher is more aggressive.
  motor.PID_velocity.I = 10; //for correcting velocity error
  motor.PID_velocity.D = 0.001; //damps sudden changes to angle velocity
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;

  motor.P_angle.P = 5;   // angle P controller -  default P=20
  // velocity low pass filtering
  // default 5ms - try different values to see what is the best. 
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01;
  motor.velocity_limit = targetVelocity_0; //maximal velocity of the position control

  //key for reducing torque!
  motor.voltage_limit = 4;

  motor.linkSensor(&encoder); 
  motor.linkDriver(&driver);

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
  //Serial.println(F("Set the target using serial terminal and command M:"));
  _delay(1000);
}
long timestamp_us = _micros();
void loop() {
  if (sim_state.current != sim_state.previous) {
    stateStartTime = _micros();
    Serial.println("set start time");
    Serial.println(stateStartTime);
    sim_state.previous = sim_state.current;

    if (sim_state.current == Simulation::RANDOM_NOISE) {
      sim_state.random_noise_rand = randomRange(5, 26) * 1000000UL;
    }
  }
  
  if (motor.velocity_limit != targetVelocity_0) {
    motor.velocity_limit = targetVelocity_0;
  }

  switch (sim_state.current){
    case Simulation::IDLE:
      Serial.println("Idle State");
      // TODO - check if motor is moving rather than if 5 seconds has passed
      // Compare motor.shaftAngle with command of motor.  I.e is there torque being applied?
      // this can detect pull forces
      if ((_micros() - stateStartTime) > 5000000UL) {
        sim_state.current = Simulation::CALIBRATION;
      }
      break;
    case Simulation::CALIBRATION:
      Serial.println("Calibration State");
      //TODO - make MOTOR relaxed. Meaning you can pull it into a different position, no torque.
      if ((_micros() - stateStartTime) > 5000000UL) {
        sim_state.current = Simulation::RANDOM_NOISE;
      } else if (false) { // TODO - implement motor movement detection (similar with IDLE)
        stateStartTime = _micros();
      }
      break;
    case Simulation::RANDOM_NOISE:
      
      if ((_micros() - stateStartTime) > sim_state.random_noise_rand) {
        sim_state.current = Simulation::INCIDENT;
      } else if((_micros() - timestamp_us) > 1000000UL) {
        timestamp_us = _micros();
        // TODO - implement shaking of brakes using a progressive timer (oren's hands shaked)
        // https://docs.simplefoc.com/angle_loop
        // invert the signal changing direction every half second to four seconds on each brake randomly 
        // inverse angle in radians
        targetPosition_0 = -targetPosition_0;   
      }
      break;
    case Simulation::INCIDENT:
      // change target position value  
      // TODO - make motor moved random
      // 0.5m, 0.15m/s
      moveBrake(0.5, 0.15, MotorID::MOTOR_ONE, MountSide::RIGHT);
      sim_state.current = Simulation::RESPONSE;
      break;
    case Simulation::RESPONSE:
      // TODO - detect if motor has arrived incident location
      bool arrived = false;
      if (false) {
        arrived = true;
      }

      if (!arrived || (_micros() - stateStartTime) <= 3000000UL) {
        // TODO - implement the other motor movement detection as user response time
        if (false) {
          // TODO - set success if the movement is greater than a threshold. i.e. non random movement; otherwise, no action taken
          bool success = false;
          if (success) {
            // TODO - implement success by adding a new state reset and all reset state does is to bring the motors back to 
            // their original position and then go to IDLE state. We only go to IDLE state if the motors are back to their original position physically
            sim_state.current = Simulation::RESET;
          } // no action taken otherwise
        }

        if (!arrived) {
          stateStartTime = _micros(); // reset the timer
        }
      } else { // no action within 3 seconds after motor arrival
        // TODO - implement failure 
      }
      break;
    case Simulation::RESET:
      // TODO - detect if motor has arrived at the original position
      if (false) {
        sim_state.current = Simulation::IDLE;
      }
      break;  
    case Simulation::INVALID:
      break;
    default:
      // weird simulation state detected
      break;
    }

    // TODO
  // encoder.update();
  // Serial.print(encoder.getAngle());
  // Serial.print("\t");
  // Serial.println(encoder.getVelocity());

  // motor.loopFOC();
  // motor.velocity_limit = targetVelocity_0;
  // motor.move(targetPosition_0);
  //motor.monitor();
  // user communication
  command.run();
}

void doA(){encoder.handleA();}
void doB(){encoder.handleB();}
void doIndex(){encoder.handleIndex();}

//TODO - Rahul ... input is meters, output needs to be in radians
// diameter 47mm or 0.047m
// radius 0.0235m
// circumference 2pi*r = 0.1476m = 2pi radians = 1 rotation
// 1 Meter = 6.77 rotations or 42.54 radians
void moveBrake(float targetPosition, float targetVelocity, MotorID motor, MountSide side) {
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
