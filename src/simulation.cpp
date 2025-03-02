#include "simulation.h"
#include "math.h"

void Simulation::newState() {
  if (current != previous) {
    newStateStartTime = _micros();
    previous = current;

    if (current == SimulationState::RANDOM_NOISE) {
      random_noise_rand = random(5, 26) * 1000000UL;
    }
  }
}    

void Simulation::updateIDLE() {
    /*
    if (getMotorState() == MotorState::IDLE) {
         current = SimulationState::CALIBRATION;
    }*/
}

void Simulation::updateCALIBRATION() {
    /*
    unsigned long current_time = _micros();
    // TODO - triger both motors to move to a specific position and if hand movement detected, continue in pulling/calibration
    // If no movement detected for 5 seconds, go to RANDOM_NOISE
    if (AreMotorsStillAtLocationWithinTime(current_time, calibration_wait)) {
        current = SimulationState::RANDOM_NOISE;
    } else {
        
    }
    TrigerMotorToMoveToLocation(ComponentID::ZERO);
    TrigerMotorToMoveToLocation(ComponentID::ONE);*/
    if ((_micros() - newStateStartTime) > 5000000UL) {
        current = SimulationState::RANDOM_NOISE;
      } else if (false) { // TODO - implement motor movement detection (similar with IDLE)
        newStateStartTime = _micros();
      }
}

void Simulation::updateRANDOM_NOISE() {
    unsigned long current_time = _micros();
    if ((current_time - newStateStartTime) > random_noise_rand) {
        current = SimulationState::INCIDENT;
      } else if((current_time - perLoopStartTime) > 1000000UL) {
        perLoopStartTime = current_time;
        // TODO - implement shaking of brakes using a progressive timer (oren's hands shaked)
        // https://docs.simplefoc.com/angle_loop
        // invert the signal changing direction every half second to four seconds on each brake randomly 
        // inverse angle in radians
        targetPosition_0 = -targetPosition_0;   
      }
}
void Simulation::updateINCIDENT() {
    // change target position value  
    // TODO - make motor moved random
    // 0.5m, 0.15m/s
    moveBrake(0.5, 0.15, MountSide::RIGHT);
    current = SimulationState::RESPONSE;
}

void Simulation::updateRESPONSE() {
    unsigned long current_time = _micros();
    // TODO - detect if motor has arrived incident location
    bool arrived = false;
    if (false) {
        arrived = true;
    }

    if (!arrived || (current_time - newStateStartTime) <= 3000000UL) {
        // TODO - implement the other motor movement detection as user response time
        if (false) { // there is user movement
          // TODO - set success if the movement is greater than a threshold. i.e. non random movement; otherwise, no action taken
          bool success = false;
          if (success) {
            // TODO - implement success by adding a new state reset and all reset state does is to bring the motors back to 
            // their original position and then go to IDLE state. We only go to IDLE state if the motors are back to their original position physically
            current = SimulationState::RESET;
          } // no action taken otherwise
        }

        if (!arrived) {
          newStateStartTime = current_time; // reset the timer
        }
      } else { // no action within 3 seconds after motor arrival
        // TODO - implement failure 
      }
}

void Simulation::updateRESET() {
    // TODO - detect if motor has arrived at the original position
    if (false) {
        current = SimulationState::IDLE;
    }
}

//TODO - Rahul ... input is meters, output needs to be in radians
// diameter 47mm or 0.047m
// radius 0.0235m
// circumference 2pi*r = 0.1476m = 2pi radians = 1 rotation
// 1 Meter = 6.77 rotations or 42.54 radians
void Simulation::moveBrake(float targetPosition, float targetVelocity, MountSide side, ComponentID motor) {
  if (motor == ComponentID::ZERO) {
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
MotorState Simulation::getMotorState() {}

MotorState Simulation::getMotorStateGivenID(ComponentID motor) {}

void Simulation::TrigerMotorToMoveToLocation(ComponentID motor) {}

bool Simulation::AreMotorsStillAtLocationWithinTime(unsigned long current_time, unsigned long wait_time) {}*/