#include "simulation.h"
#include "math.h"

void Simulation::newState() {
  if (current != previous) {
    newStateStartTime = _micros();
    previous = current;

    if (current == SimulationState::RANDOM_NOISE) {
      random_noise_wait = random(5, 26) * 1000000UL;
    }

    begining_motor_0_position = motor_0->shaftAngle();
    begining_motor_1_position = motor_1->shaftAngle();
    simulation_state_stablized = false;
  }
}    

bool Simulation::motionDetectedForMotor(ComponentID motor_id) {
    float begining_position = motor_id == ComponentID::ZERO ? begining_motor_0_position : begining_motor_1_position;
    BLDCMotor* cur_motor = getMotor(motor_id);
    float current_position = cur_motor->shaftAngle();
    if (abs(current_position - begining_position) > movement_threshold) {
        return true;
    }
    return false;
}

void Simulation::setMotorPosition(ComponentID motor_id, float position) {
    BLDCMotor* cur_motor = getMotor(motor_id);
    cur_motor->target = position;
}

bool Simulation::atLocationForMotor(ComponentID motor_id, float position) {
    BLDCMotor* cur_motor = getMotor(motor_id);
    float current_position = cur_motor->shaftAngle();
    if (abs(current_position - position) < stablized_threshold) {
        return true;
    }
    return false;
}

void Simulation::moveMotor(ComponentID motor_id, float targetPosition, float targetVelocity) {
    BLDCMotor* cur_motor = getMotor(motor_id);
    cur_motor->velocity_limit = targetVelocity;
    cur_motor->move(targetPosition);
}

ComponentID Simulation::selectRandomMotor() {
    return random(0, 2) == 0 ? ComponentID::ZERO : ComponentID::ONE;
}

void Simulation::updateIDLE() {
    if (motionDetectedForMotor(ComponentID::ZERO) || motionDetectedForMotor(ComponentID::ONE)) {
        current = SimulationState::CALIBRATION;
    }
}

void Simulation::updateCALIBRATION() {
    calibrated_position = 0.0;
    setMotorPosition(ComponentID::ZERO, calibrated_position);
    setMotorPosition(ComponentID::ONE, calibrated_position);
    current = SimulationState::RESET;
}

void Simulation::updateRESET() {
    moveMotor(ComponentID::ZERO, calibrated_position, reset_velocity);
    moveMotor(ComponentID::ONE, calibrated_position, reset_velocity);
    if (!atLocationForMotor(ComponentID::ZERO, calibrated_position) || !atLocationForMotor(ComponentID::ONE, calibrated_position)) {
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
    } else if ((current_time - newStateStartTime) > random_noise_wait) {
        current = SimulationState::INCIDENT;
    } else if((current_time - perLoopStartTime) > 1000000UL) {
        perLoopStartTime = current_time;
        float new_target = calibrated_position + random_noise_threshold * last_random_noise_direction;
        last_random_noise_direction = -last_random_noise_direction;
        moveMotor(ComponentID::ZERO, new_target, random_noise_velocity);
        moveMotor(ComponentID::ONE, new_target, random_noise_velocity);
    }
}
void Simulation::updateINCIDENT() {
    incident_motor = selectRandomMotor();
    moveMotor(incident_motor, incident_position, incident_velocity);
    if (!atLocationForMotor(incident_motor, incident_position)) {
        // do nothing and wait
    } else if (!simulation_state_stablized){
        simulation_state_stablized = true;
        current = SimulationState::RESPONSE;
    }
}

void Simulation::updateRESPONSE() {
    ComponentID react_motor = incident_motor == ComponentID::ZERO ? ComponentID::ONE : ComponentID::ZERO;
    unsigned long current_time = _micros();
    if ((current_time - newStateStartTime) <= response_time_wait) {
        if (motionDetectedForMotor(react_motor)) { // there is user movement
            current = SimulationState::RESET;
          } // no action taken otherwise
    } else { // no action within 3 seconds after motor arrival
        // TODO - implement failure by buzzing the motor or something
    }
}