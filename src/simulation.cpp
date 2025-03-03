#include "simulation.h"
#include "math.h"

void Simulation::newState() {
  if (current != previous) {
    newStateStartTime = _micros();
    previous = current;

    if (current == SimulationState::RANDOM_NOISE) {
      random_noise_rand = random(5, 26) * 1000000UL;
    }

    // TODO(confirm) - record the beginning motor positions
    begining_motor_0_position = motor_0->shaftAngle();
    begining_motor_1_position = motor_1->shaftAngle();
    simulation_state_stablized = false;
  }
}    

bool Simulation::motionDetectedForMotor(ComponentID motor_id) {
    BLDCMotor* cur_motor = getMotor(motor_id);
    float current_position = cur_motor->shaftAngle();
    // TODO(confirm) - implement the movement detection
    if (abs(current_position - begining_motor_0_position) > movementThreshold) {
        return true;
    }
    return false;
}

void Simulation::SetMotorPosition(ComponentID motor_id, float position) {
    BLDCMotor* cur_motor = getMotor(motor_id);
    // TODO(confirm) - is this correct?
    cur_motor->target = position;
}

bool Simulation::AtLocationForMotor(ComponentID motor_id) {
    // TODO(confirm) - implement the motor location detection with threshold
    return false;
}

// Resting state for all components and waiting for motion
void Simulation::updateIDLE() {
    if (motionDetectedForMotor(ComponentID::ZERO) || motionDetectedForMotor(ComponentID::ONE)) {
        current = SimulationState::CALIBRATION;
    }
}

void Simulation::updateCALIBRATION() {
    // hard set the calibration position
    calibrated_position = 0.0;
    SetMotorPosition(ComponentID::ZERO, 0);
    SetMotorPosition(ComponentID::ONE, 0);
    current = SimulationState::RESET;
}

void Simulation::updateRESET() {
    // TODO - set the motor to start rotating back to the calibrated position
    // moveBrakeForMotor(motor_id, calibrated_position, 0.15, MountSide::RIGHT);
    if (!AtLocationForMotor(ComponentID::ZERO) || !AtLocationForMotor(ComponentID::ONE)) {
        // do nothing and wait
    } else if (!simulation_state_stablized){
        simulation_state_stablized = true;
        newStateStartTime = _micros();
    } else if (_micros() - newStateStartTime > reset_state_wait) {
        current = SimulationState::RANDOM_NOISE;
    }
}

void Simulation::updateRANDOM_NOISE() {
    unsigned long current_time = _micros();
    if (motionDetectedForMotor(ComponentID::ZERO) || motionDetectedForMotor(ComponentID::ONE)) {
        current = SimulationState::RESET;
    } else if ((current_time - newStateStartTime) > random_noise_rand) {
        current = SimulationState::INCIDENT;
    } else if((current_time - perLoopStartTime) > 1000000UL) {
        perLoopStartTime = current_time;
        // TODO - implement shaking of brakes using a progressive timer (oren's hands shaked)
        // https://docs.simplefoc.com/angle_loop
        // invert the signal changing direction every half second to four seconds on each brake randomly 
        // inverse angle in radians
        targetPosition_0 = -targetPosition_0;   
        targetPosition_1 = -targetPosition_1;
    }
}
void Simulation::updateINCIDENT() {
    SetMotorPosition(ComponentID::ZERO, 0);
    // TODO - make motor moved random
    // 0.5m, 0.15m/s
    // moveBrake(0.5, 0.15, MountSide::RIGHT);
    if (!AtLocationForMotor(ComponentID::ZERO)) {
        // do nothing and wait
    } else if (!simulation_state_stablized){
        simulation_state_stablized = true;
        current = SimulationState::RESPONSE;
    }
}

void Simulation::updateRESPONSE() {
    unsigned long current_time = _micros();
    if ((current_time - newStateStartTime) <= response_time_wait) {
        // TODO - implement the other motor movement detection as user response time
        if (motionDetectedForMotor(ComponentID::ONE)) { // there is user movement
            current = SimulationState::RESET;
          } // no action taken otherwise
    } else { // no action within 3 seconds after motor arrival
        // TODO - implement failure 
    }
}

//TODO - Rahul ... input is meters, output needs to be in radians
// diameter 47mm or 0.047m
// radius 0.0235m
// circumference 2pi*r = 0.1476m = 2pi radians = 1 rotation
// 1 Meter = 6.77 rotations or 42.54 radians
void Simulation::moveBrake(float targetPosition, float targetVelocity, ComponentID motor) {
  targetAngle = targetPosition / 0.15
  if (motor == ComponentID::ZERO) {
    targetVelocity_0 = side == ComponentID::ZERO ? -targetVelocity : targetVelocity;
    targetPosition_0 = side == ComponentID::ZERO ? -targetPosition: targetPosition;
    //convert targetPosition meters to radians - 2pi radians or 6.28 is approx equal to 144mm
    motor_0.controller = MotionControlType::angle;
    motor_0.velocity_limit = targetVelocity / 0.15;
    motor_0.move()
  } else {
    targetVelocity_1 = targetVelocity;
    targetPosition_1 = side == ComponentID::ZERO ? targetPosition: -targetPosition;
    //convert targetPosition meters to radians - 2pi radians or 6.28 is approx equal to 144mm
    motor_1.controller = MotionControlType::angle;
    motor_1.velocity_limit = targetVelocity_1 / 0.15;
    motor_1.move()
  }
}

/*
MotorState Simulation::getMotorState() {}

MotorState Simulation::getMotorStateGivenID(ComponentID motor) {}

void Simulation::TrigerMotorToMoveToLocation(ComponentID motor) {}

bool Simulation::AreMotorsStillAtLocationWithinTime(unsigned long current_time, unsigned long wait_time) {}
*/